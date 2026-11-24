"""Publish cached MAVLink telemetry with explicit source and receipt timing."""

import math
import time
from typing import Callable

from drone_interfaces.msg import Telem
from pymavlink import mavutil
from rclpy.clock import Clock


FIELD_MAPS = {
    "GLOBAL_POSITION_INT": {
        "lat": "lat", "lon": "lon", "alt": "alt", "heading": "hdg",
    },
    "ATTITUDE": {
        "roll": "roll", "pitch": "pitch", "yaw": "yaw",
        "roll_rate": "rollspeed", "pitch_rate": "pitchspeed",
        "yaw_rate": "yawspeed",
    },
    "LOCAL_POSITION_NED": {
        "x": "x", "y": "y", "z": "z", "vx": "vx", "vy": "vy",
        "vz": "vz",
    },
}


class DroneInfo:
    """Request and publish telemetry without treating a snapshot as atomic.

    Existing position/attitude fields retain their original units. The ROS
    header timestamps publication; local_time_boot_ms timestamps the source
    LOCAL_POSITION_NED message. VFR_HUD has no source timestamp, so its age
    is measured on the receiving host's monotonic clock.
    """

    def __init__(
        self,
        master,
        telem_publisher,
        drone_info_frequency: int,
        *,
        clock: Clock,
        frame_id: str = "local_ned",
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Use the owning ROS node's clock and an injectable receipt clock."""
        if not math.isfinite(drone_info_frequency):
            raise ValueError("telemetry frequency must be finite")
        if drone_info_frequency <= 0:
            raise ValueError("telemetry frequency must be positive")
        self.master = master
        self.telem_publisher = telem_publisher
        self.drone_info_frequency = drone_info_frequency
        self.clock = clock
        self.frame_id = frame_id
        self._monotonic_clock = monotonic_clock
        self._streams_requested = False
        self._airspeed = math.nan
        self._airspeed_received_at = None

    def _request_streams(self) -> None:
        """Request streams once per connection, without flooding each cycle.

        Requests are not acknowledgement checks. Verify actual stream rates
        in logs; create a new DroneInfo after reconnecting or rebooting.
        """
        if self._streams_requested:
            return
        for name in (*FIELD_MAPS, "VFR_HUD"):
            message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                1e6 / self.drone_info_frequency,
                0, 0, 0, 0, 0,
            )
        self._streams_requested = True

    def _snapshot(self, received) -> Telem:
        """Copy available cached fields and identify fresh position samples."""
        output = Telem()
        for message_type, fields in FIELD_MAPS.items():
            source = self.master.messages.get(message_type)
            if source is not None:
                for destination, field in fields.items():
                    setattr(output, destination, getattr(source, field))

        position = self.master.messages.get("LOCAL_POSITION_NED")
        if position is not None:
            output.local_time_boot_ms = position.time_boot_ms
            output.local_position_valid = all(
                math.isfinite(getattr(position, field))
                for field in FIELD_MAPS["LOCAL_POSITION_NED"]
            )
        output.local_position_updated = (
            received.get_type() == "LOCAL_POSITION_NED"
        )

        now = self._monotonic_clock()
        if received.get_type() == "VFR_HUD":
            self._airspeed = float(received.airspeed)
            self._airspeed_received_at = now
        output.airspeed_mps = self._airspeed
        output.airspeed_received = self._airspeed_received_at is not None
        output.airspeed_age_s = math.nan
        if output.airspeed_received:
            output.airspeed_age_s = now - self._airspeed_received_at

        heartbeat = self.master.messages.get("HEARTBEAT")
        if heartbeat is not None:
            output.mode = mavutil.mode_string_v10(heartbeat)
        return output

    def publish_telemetry(self) -> Telem | None:
        """Receive with a bounded wait and stamp a snapshot before publishing.

        A receive timeout returns None and publishes nothing; it must not make
        stale cache values appear to be a newly received state observation.
        """
        self._request_streams()
        received = self.master.recv_match(
            type=[*FIELD_MAPS, "VFR_HUD"], blocking=True, timeout=1.0,
        )
        if received is None:
            return None
        output = self._snapshot(received)
        output.header.stamp = self.clock.now().to_msg()
        output.header.frame_id = self.frame_id
        self.telem_publisher.publish(output)
        return output

    def publishTelemInfo(self) -> Telem | None:
        """Preserve the existing caller API."""
        return self.publish_telemetry()
