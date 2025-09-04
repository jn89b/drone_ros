#!/usr/bin/env python3
import os
import sys
import threading
import time
from typing import Dict, Tuple, Optional, Iterable, Set

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

# ---------------------------
# Helpers for MAVLink <-> ROS
# ---------------------------

# Default: only mirror these, add more later on demand
DEFAULT_WHITELIST = {"SID_DONE", "SID_TYPE"}

_MAV_TYPE_TO_PY = {
    mavlink2.MAV_PARAM_TYPE_UINT8:   int,
    mavlink2.MAV_PARAM_TYPE_INT8:    int,
    mavlink2.MAV_PARAM_TYPE_UINT16:  int,
    mavlink2.MAV_PARAM_TYPE_INT16:   int,
    mavlink2.MAV_PARAM_TYPE_UINT32:  int,
    mavlink2.MAV_PARAM_TYPE_INT32:   int,
    mavlink2.MAV_PARAM_TYPE_REAL32:  float,
    mavlink2.MAV_PARAM_TYPE_REAL64:  float,
}

def py_to_mav_value(value, mav_type: int) -> float:
    if mav_type in (mavlink2.MAV_PARAM_TYPE_REAL32, mavlink2.MAV_PARAM_TYPE_REAL64):
        return float(value)
    return float(int(value))

def normalized_param_name(name: str) -> str:
    return name.lower()

def _decode_param_id(pid) -> str:
    if isinstance(pid, bytes):
        return pid.decode('utf-8', errors='ignore').rstrip('\x00')
    elif isinstance(pid, str):
        return pid.rstrip('\x00')
    return str(pid)

# ---------------------------
# Node
# ---------------------------

class MavlinkParamBridge(Node):
    """
    Mirrors only a selected set of MAVLink parameters into ROS 2 parameters.
    """

    def __init__(self):
        super().__init__('mavlink_param_bridge')

        # ROS 2 config parameters
        self.declare_parameter('connection_url', os.environ.get('MAVLINK_URL', 'udp:127.0.0.1:14554'))
        self.declare_parameter('target_sysid',   1)
        self.declare_parameter('target_compid',  1)
        self.declare_parameter('pull_timeout_sec', 5.0)
        self.declare_parameter('heartbeat_wait_sec', 10.0)
        self.declare_parameter('log_raw_rx', False)
        self.declare_parameter('save_path', '')
        # optional: override whitelist at runtime: e.g. ["SID_DONE","SID_TYPE","ARMING_CHECK"]
        self.declare_parameter('whitelist', list(DEFAULT_WHITELIST))

        self._conn_url = self.get_parameter('connection_url').get_parameter_value().string_value
        self._sysid    = int(self.get_parameter('target_sysid').value)
        self._compid   = int(self.get_parameter('target_compid').value)
        self._pull_to  = float(self.get_parameter('pull_timeout_sec').value)
        self._hb_wait  = float(self.get_parameter('heartbeat_wait_sec').value)
        self._log_raw  = bool(self.get_parameter('log_raw_rx').value)
        self._whitelist: Set[str] = set(self.get_parameter('whitelist').get_parameter_value().string_array_value or [])
        if not self._whitelist:
            self._whitelist = set(DEFAULT_WHITELIST)

        # MAVLink connection
        self.mav: Optional[mavutil.mavfile] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.rx_stop = threading.Event()

        # Param store: ROS-safe name -> (original_name, value, mav_type, idx, cnt)
        self.param_store: Dict[str, Tuple[str, float, int, int, int]] = {}
        self.orig_to_ros: Dict[str, str] = {}

        # Guard for synchronous PARAM_SET awaiting echo
        self.set_lock = threading.Lock()
        self.pending_set: Optional[str] = None
        self.pending_set_result: Optional[bool] = None

        # Subscribe to ROS parameter changes
        self.add_on_set_parameters_callback(self._on_ros_param_set)

        # Connect + pull only the whitelist
        self._connect()
        self._pull_selected_params(self._whitelist)

        # Spin up RX loop
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"MAVLink Param Bridge (whitelist: {sorted(self._whitelist)}) ready.")

    # ---------------------------
    # Connection & Pull (selected)
    # ---------------------------

    def _connect(self):
        self.get_logger().info(f"Connecting to MAVLink: {self._conn_url}")
        self.mav = mavutil.mavlink_connection(self._conn_url)
        try:
            self.mav.wait_heartbeat(timeout=self._hb_wait)
        except Exception as e:
            self.get_logger().warn(f"No heartbeat within {self._hb_wait}s: {e}. Proceeding anyway.")
        self.get_logger().info("Connected (or proceeding without heartbeat).")

    def _pull_selected_params(self, names: Iterable[str]):
        """
        Request each whitelisted parameter individually using PARAM_REQUEST_READ.
        """
        if self.mav is None:
            self.get_logger().error("MAVLink connection not established.")
            return

        deadline = time.time() + self._pull_to
        pending = set(names)

        # Send initial requests
        for n in pending:
            self._request_read(n)

        # Collect until all received or timeout
        while pending and time.time() < deadline:
            msg = self.mav.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() != 'PARAM_VALUE':
                continue

            name = _decode_param_id(msg.param_id)
            if name not in pending:
                # Ignore non-whitelisted or already processed
                continue

            value = float(msg.param_value)
            ptype = int(msg.param_type)
            idx   = int(getattr(msg, 'param_index', -1))
            cnt   = int(getattr(msg, 'param_count', 0))

            ros_name = normalized_param_name(name)
            self.param_store[ros_name] = (name, value, ptype, idx, cnt)
            self.orig_to_ros[name] = ros_name
            self._declare_or_update_ros_param(ros_name, value, ptype)

            pending.discard(name)

        if pending:
            self.get_logger().warn(f"Timeout pulling params (missing): {sorted(pending)}")
        else:
            self.get_logger().info("Pulled selected parameters successfully.")

    def _request_read(self, name: str):
        """
        PARAM_REQUEST_READ: set param_index=-1 and provide the param_id.
        """
        # param_request_read_send(target_system, target_component, param_id, param_index)
        self.mav.mav.param_request_read_send(
            self._sysid, self._compid, name.encode('utf-8'), -1
        )

    def _declare_or_update_ros_param(self, ros_name: str, value: float, ptype: int):
        py_type = _MAV_TYPE_TO_PY.get(ptype, float)
        casted = py_type(value)

        if not self.has_parameter(ros_name):
            # First time we see it: declare with the inferred type
            try:
                self.declare_parameter(ros_name, casted)
                return
            except Exception:
                # If it raced or a different type slipped in, fall through and set
                pass

        # Already declared (or declaration failed due to type mismatch): set it
        self.set_parameters([
            Parameter(
                ros_name,
                Parameter.Type.DOUBLE if isinstance(casted, float) else Parameter.Type.INTEGER,
                casted
            )
        ])

    def _declare_or_update_ros_param(self, ros_name: str, value: float, ptype: int):
        py_type = _MAV_TYPE_TO_PY.get(ptype, float)
        casted = py_type(value)
        if not self.has_parameter(ros_name):
            try:
                self.declare_parameter(ros_name, casted)
            except Exception:
                self.set_parameters([Parameter(
                    ros_name,
                    Parameter.Type.DOUBLE if isinstance(casted, float) else Parameter.Type.INTEGER,
                    casted
                )])
        else:
            self.set_parameters([Parameter(
                ros_name,
                Parameter.Type.DOUBLE if isinstance(casted, float) else Parameter.Type.INTEGER,
                casted
            )])

    # ---------------------------
    # RX Loop (only care about whitelisted names)
    # ---------------------------

    def _rx_loop(self):
        if self.mav is None:
            return
        while not self.rx_stop.is_set():
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.5)
            except Exception:
                msg = None

            if msg is None:
                continue

            if self._log_raw:
                self.get_logger().debug(f"RX: {msg}")

            if msg.get_type() == 'PARAM_VALUE':
                name = _decode_param_id(msg.param_id)
                if name not in self._whitelist:
                    continue  # ignore non-whitelisted params

                value = float(msg.param_value)
                ptype = int(msg.param_type)
                idx   = int(getattr(msg, 'param_index', -1))
                cnt   = int(getattr(msg, 'param_count', 0))

                ros_name = self.orig_to_ros.get(name, normalized_param_name(name))
                self.param_store[ros_name] = (name, value, ptype, idx, cnt)
                self._declare_or_update_ros_param(ros_name, value, ptype)

                # If we are waiting on a set, mark success when echo arrives
                with self.set_lock:
                    if self.pending_set == name:
                        self.pending_set_result = True
                        self.pending_set = None

    # ---------------------------
    # ROS Parameter Set Callback (only whitelisted)
    # ---------------------------

    def _on_ros_param_set(self, params):
        for p in params:
            ros_name = p.name
            if ros_name not in self.param_store:
                # not mirrored => reject to avoid surprises, or allow local-only? Here we reject.
                return SetParametersResult(successful=False, reason=f"{ros_name} is not mirrored/whitelisted")

            orig_name, _, ptype, _, _ = self.param_store[ros_name]
            try:
                self._send_param_set(orig_name, p.value, ptype)
            except Exception as e:
                self.get_logger().error(f"Failed to set {orig_name}: {e}")
                return SetParametersResult(successful=False, reason=str(e))

        return SetParametersResult(successful=True)

    def _send_param_set(self, orig_name: str, value, ptype: int):
        if self.mav is None:
            raise RuntimeError("No MAVLink connection")
        if orig_name not in self._whitelist:
            raise RuntimeError(f"{orig_name} not in whitelist")

        mav_val = py_to_mav_value(value, ptype)
        with self.set_lock:
            self.pending_set = orig_name
            self.pending_set_result = None

        self.get_logger().info(f"PARAM_SET {orig_name}={value} (mav_type={ptype})")
        self.mav.mav.param_set_send(
            self._sysid, self._compid, orig_name.encode('utf-8'), float(mav_val), ptype
        )

        # wait briefly for echo
        t0 = time.time()
        while time.time() - t0 < 3.0:
            with self.set_lock:
                if self.pending_set_result is True:
                    return
            time.sleep(0.05)

        self.get_logger().warn(f"No PARAM_VALUE echo for {orig_name}; proceeding optimistically.")

    # ---------------------------
    # Shutdown
    # ---------------------------

    def destroy_node(self):
        self.rx_stop.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        if self.mav:
            try:
                self.mav.close()
            except Exception:
                pass
        super().destroy_node()

# ---------------------------
# Main
# ---------------------------

def main(argv=None):
    rclpy.init(args=argv)
    node = MavlinkParamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
