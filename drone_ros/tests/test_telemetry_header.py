"""Test telemetry timestamp semantics without a MAVLink connection."""

from types import SimpleNamespace
from unittest.mock import Mock
import math

import pytest
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time_source import TimeSource
from rclpy.serialization import deserialize_message, serialize_message

from drone_interfaces.msg import Telem
from drone_ros.DroneInfo import DroneInfo


@pytest.fixture
def telemetry():
    """Supply cached MAVLink fields and a deterministic ROS clock."""
    master = Mock()
    master.messages = {
        "GLOBAL_POSITION_INT": SimpleNamespace(lat=1, lon=2, alt=3, hdg=4),
        "ATTITUDE": SimpleNamespace(
            roll=0.1, pitch=0.2, yaw=0.3,
            rollspeed=0.4, pitchspeed=0.5, yawspeed=0.6,
        ),
        "LOCAL_POSITION_NED": SimpleNamespace(
            x=10.0, y=20.0, z=-30.0, vx=1.0, vy=2.0, vz=3.0,
            time_boot_ms=12000,
        ),
    }
    master.recv_match.return_value = SimpleNamespace(
        get_type=lambda: "LOCAL_POSITION_NED",
    )
    publisher = Mock()
    clock = Clock(clock_type=ClockType.ROS_TIME)
    source = TimeSource()
    source.attach_clock(clock)
    source.ros_time_is_active = True
    clock.set_ros_time_override(Time(seconds=12, nanoseconds=345))
    info = DroneInfo(master, publisher, 30, clock=clock)
    return info, master, publisher, clock, source


def test_publication_stamp_and_serialization(telemetry):
    """Stamp each publication and preserve existing NED fields on the wire."""
    info, master, publisher, clock, _ = telemetry
    output = info.publishTelemInfo()
    assert output.header.stamp.sec == 12
    assert output.header.stamp.nanosec == 345
    assert output.header.frame_id == "local_ned"
    assert (output.x, output.y, output.z) == (10.0, 20.0, -30.0)
    publisher.publish.assert_called_once_with(output)
    decoded = deserialize_message(serialize_message(output), Telem)
    assert decoded.header == output.header
    assert decoded.local_time_boot_ms == 12000
    assert decoded.local_position_valid
    assert decoded.local_position_updated
    assert decoded.x == output.x
    assert math.isnan(decoded.airspeed_mps)
    clock.set_ros_time_override(Time(seconds=15, nanoseconds=678))
    later = info.publishTelemInfo()
    assert later.header.stamp.sec == 15
    assert later.header.stamp.nanosec == 678
    assert master.recv_match.call_count == 2


def test_stamp_is_taken_after_receive(telemetry):
    """A blocking receive must not leave a pre-receive timestamp."""
    info, master, _, clock, _ = telemetry

    def receive(**kwargs):
        clock.set_ros_time_override(Time(seconds=20))
        return SimpleNamespace(get_type=lambda: "LOCAL_POSITION_NED")

    master.recv_match.side_effect = receive
    assert info.publishTelemInfo().header.stamp.sec == 20


def test_partial_snapshot_still_has_publication_header(telemetry):
    """The header does not imply all cached fields were available."""
    info, master, _, _, _ = telemetry
    master.messages.clear()
    output = info.publishTelemInfo()
    assert output.header.stamp.sec == 12
    assert output.header.frame_id == "local_ned"
    assert not output.local_position_valid


def test_node_clock_is_used(telemetry):
    """Use the node's clock rather than silently creating another clock."""
    _, master, publisher, _, _ = telemetry
    rclpy.init()
    node = Node("telemetry_header_test")
    try:
        info = DroneInfo(master, publisher, 30, clock=node.get_clock())
        before = node.get_clock().now().nanoseconds
        output = info.publishTelemInfo()
        after = node.get_clock().now().nanoseconds
        stamp = Time.from_msg(output.header.stamp).nanoseconds
        assert before <= stamp <= after
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_airspeed_receipt_age_and_no_groundspeed_substitution(telemetry):
    """Cache reported airspeed with host age independently of ground speed."""
    info, master, _, _, _ = telemetry
    receipt_clock = Mock(return_value=100.0)
    info._monotonic_clock = receipt_clock
    first = info.publish_telemetry()
    assert not first.airspeed_received
    assert math.isnan(first.airspeed_mps)
    master.recv_match.return_value = SimpleNamespace(
        get_type=lambda: "VFR_HUD", airspeed=18.5,
    )
    hud = info.publish_telemetry()
    assert hud.airspeed_received
    assert hud.airspeed_mps == 18.5
    assert hud.airspeed_age_s == 0.0
    assert not hud.local_position_updated
    receipt_clock.return_value = 100.25
    master.recv_match.return_value = SimpleNamespace(
        get_type=lambda: "LOCAL_POSITION_NED",
    )
    later = info.publish_telemetry()
    assert later.airspeed_mps == 18.5
    assert later.airspeed_age_s == 0.25
    assert later.local_position_updated
    assert master.mav.command_long_send.call_count == 4


def test_timeout_does_not_publish_cached_state(telemetry):
    """Loss of input must not manufacture fresh telemetry samples."""
    info, master, publisher, _, _ = telemetry
    master.recv_match.return_value = None
    assert info.publish_telemetry() is None
    publisher.publish.assert_not_called()


def test_nonfinite_position_flag(telemetry):
    """Do not mark a nonfinite position as usable for analysis."""
    info, master, _, _, _ = telemetry
    master.messages["LOCAL_POSITION_NED"].x = math.nan
    assert not info.publish_telemetry().local_position_valid
