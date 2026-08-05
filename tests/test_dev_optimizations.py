import platform
import struct
import time

import can
import pytest

from piper_sdk.hardware_port.can_encapsulation_v0_4_0 import C_STD_CAN
from piper_sdk.interface import C_PiperInterface_V2
from piper_sdk.piper_msgs.msg_v2 import PiperMessage
from piper_sdk.piper_msgs.msg_v2.can_id import CanIDPiper
from piper_sdk.utils import C_FPSCounter


class FakeBus(can.BusABC):
    '''Records state reads, traffic and lifecycle for C_STD_CAN.'''

    def __init__(self, channel=None, bustype=None, bitrate=None,
                 receive_own_messages=None, local_loopback=None):
        super().__init__(channel=channel, bustype=bustype, bitrate=bitrate,
                         receive_own_messages=receive_own_messages,
                         local_loopback=local_loopback)
        self.state_reads = 0
        self.recv_calls = 0
        self.shutdown_calls = 0
        self._state = can.BusState.ACTIVE

    @property
    def state(self):
        self.state_reads += 1
        return self._state

    @state.setter
    def state(self, new_state):
        self._state = new_state

    def recv(self, timeout=None):
        self.recv_calls += 1
        return None

    def send(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        self.shutdown_calls += 1


@pytest.fixture
def bus_factory(monkeypatch):
    created = []

    def make_bus(**kwargs):
        bus = FakeBus(**kwargs)
        created.append(bus)
        return bus

    monkeypatch.setattr(can.interface, "Bus", make_bus)
    monkeypatch.setattr(platform, "system", lambda: "Linux")
    return created


def _make_can(bus_factory):
    return C_STD_CAN(channel_name="can0", bustype="socketcan",
                     expected_bitrate=1000000, judge_flag=False, auto_init=True)


# ---------------------------------------------------------------------------
# Bus throttle: is_can_bus_ok runs only every 50th read, not every read.
# ---------------------------------------------------------------------------
def test_read_bus_state_check_is_throttled_to_every_50th_read(bus_factory):
    can_obj = _make_can(bus_factory)
    recv_bus, _ = bus_factory
    assert can_obj.ReadCanMessage() == C_STD_CAN.CAN_STATUS.READ_CAN_MSG_TIMEOUT

    for _ in range(49):
        can_obj.ReadCanMessage()
    assert recv_bus.state_reads == 1, "only the 50th read should check bus state"

    for _ in range(50):
        can_obj.ReadCanMessage()
    assert recv_bus.state_reads == 2, "100 reads should check bus state exactly twice"
    assert recv_bus.recv_calls == 100


# ---------------------------------------------------------------------------
# FPS toggle
# ---------------------------------------------------------------------------
def test_fps_counter_can_be_disabled():
    counter = C_FPSCounter(enabled=False)
    counter.add_variable("ArmJoint_12")
    counter.increment("ArmJoint_12")
    assert counter.get_fps("ArmJoint_12") == 0

    counter.set_enabled(True)
    counter.add_variable("ArmJoint_12")
    counter.start()
    try:
        counter.increment("ArmJoint_12")
        time.sleep(0.15)
        assert counter.get_fps("ArmJoint_12") > 0
    finally:
        counter.stop()


def test_fps_counter_variables_survive_disable_re_enable():
    counter = C_FPSCounter(enabled=False)
    counter.add_variable("CanMonitor")
    counter.set_enabled(True)
    counter.start()
    try:
        counter.increment("CanMonitor")
        time.sleep(0.15)
        assert counter.get_fps("CanMonitor") > 0, \
            "variables added while disabled must track after re-enable"
    finally:
        counter.stop()


def test_fps_counter_re_enable_restarts_calculation_thread():
    counter = C_FPSCounter(enabled=True)
    counter.add_variable("x")
    counter.start()
    try:
        time.sleep(0.05)
        counter.increment("x")
        counter.set_enabled(False)
        assert counter.running is False

        counter.set_enabled(True)
        assert counter.running is True, "re-enabling must restart the calculation thread"
        counter.increment("x")
        time.sleep(0.15)
        assert counter.get_fps("x") > 0, "FPS must resume after re-enable"
    finally:
        counter.stop()


def _make_interface(can_name, minimal_feedback_mode=False):
    return C_PiperInterface_V2(can_name=can_name, can_auto_init=False,
                               judge_flag=False, log_to_file=False,
                               enable_performance_metrics=True,
                               minimal_feedback_mode=minimal_feedback_mode)


def _joint_frame_12(j1, j2, timestamp=1.0):
    data = struct.pack(">ii", j1, j2)
    return can.Message(arbitration_id=CanIDPiper.ARM_JOINT_FEEDBACK_12.value,
                       data=data, timestamp=timestamp)


def _gripper_frame(angle, effort, timestamp=2.0):
    data = struct.pack(">ih", angle, effort) + bytes([0, 0])
    return can.Message(arbitration_id=CanIDPiper.ARM_GRIPPER_FEEDBACK.value,
                       data=data, timestamp=timestamp)


def _high_spd_frame_1(speed, current, pos, timestamp=3.0):
    data = struct.pack(">hhi", speed, current, pos)
    return can.Message(arbitration_id=CanIDPiper.ARM_INFO_HIGH_SPD_FEEDBACK_1.value,
                       data=data, timestamp=timestamp)


def _low_spd_frame_1(foc_status_code, timestamp=4.0):
    data = struct.pack(">hhBBh", 0, 0, 0, foc_status_code & 0xFF, 0)
    return can.Message(arbitration_id=CanIDPiper.ARM_INFO_LOW_SPD_FEEDBACK_1.value,
                       data=data, timestamp=timestamp)


# ---------------------------------------------------------------------------
# Type-dispatch: only the handler matching the received frame runs.
# ---------------------------------------------------------------------------
def test_type_handlers_dispatch_only_matching_update():
    from piper_sdk.piper_msgs.msg_v2.arm_msg_type import ArmMsgType
    piper = _make_interface("test_dispatch_001")
    assert piper.rx_msg is not None
    assert piper.tx_msg is not None

    called = []
    piper._C_PiperInterface_V2__type_handlers = {
        ArmMsgType.PiperMsgGripperFeedBack: [lambda msg: called.append("gripper")],
        ArmMsgType.PiperMsgJointFeedBack_12: [lambda msg: called.append("joint_12")],
        ArmMsgType.PiperMsgStatusFeedback: [lambda msg: called.append("status")],
    }

    piper.ParseCANFrame(_gripper_frame(1000, 5))
    assert called == ["gripper"], f"only the gripper handler should run, got {called}"


def test_type_handlers_full_map_has_all_feedback_types():
    from piper_sdk.piper_msgs.msg_v2.arm_msg_type import ArmMsgType
    piper = _make_interface("test_dispatch_002")
    handlers = piper._C_PiperInterface_V2__type_handlers
    expected = {
        ArmMsgType.PiperMsgStatusFeedback,
        ArmMsgType.PiperMsgEndPoseFeedback_1,
        ArmMsgType.PiperMsgEndPoseFeedback_2,
        ArmMsgType.PiperMsgEndPoseFeedback_3,
        ArmMsgType.PiperMsgJointFeedBack_12,
        ArmMsgType.PiperMsgJointFeedBack_34,
        ArmMsgType.PiperMsgJointFeedBack_56,
        ArmMsgType.PiperMsgGripperFeedBack,
        ArmMsgType.PiperMsgHighSpdFeed_1,
        ArmMsgType.PiperMsgHighSpdFeed_2,
        ArmMsgType.PiperMsgHighSpdFeed_3,
        ArmMsgType.PiperMsgHighSpdFeed_4,
        ArmMsgType.PiperMsgHighSpdFeed_5,
        ArmMsgType.PiperMsgHighSpdFeed_6,
        ArmMsgType.PiperMsgLowSpdFeed_1,
        ArmMsgType.PiperMsgLowSpdFeed_2,
        ArmMsgType.PiperMsgLowSpdFeed_3,
        ArmMsgType.PiperMsgLowSpdFeed_4,
        ArmMsgType.PiperMsgLowSpdFeed_5,
        ArmMsgType.PiperMsgLowSpdFeed_6,
        ArmMsgType.PiperMsgFeedbackCurrentMotorAngleLimitMaxSpd,
        ArmMsgType.PiperMsgFeedbackCurrentMotorMaxAccLimit,
        ArmMsgType.PiperMsgFeedbackCurrentEndVelAccParam,
        ArmMsgType.PiperMsgCrashProtectionRatingFeedback,
        ArmMsgType.PiperMsgGripperTeachingPendantParamFeedback,
        ArmMsgType.PiperMsgFeedbackRespSetInstruction,
        ArmMsgType.PiperMsgFirmwareRead,
    }
    assert expected <= set(handlers.keys())


# ---------------------------------------------------------------------------
# Minimal feedback mode + snapshot + enable-status shortcut
# ---------------------------------------------------------------------------
def test_minimal_feedback_mode_populates_snapshot_and_enable_status():
    piper = _make_interface("test_minimal_001", minimal_feedback_mode=True)

    piper.ParseCANFrame(_joint_frame_12(1000, 2000, timestamp=1.0))
    piper.ParseCANFrame(_gripper_frame(3000, 5, timestamp=2.0))
    piper.ParseCANFrame(_high_spd_frame_1(10, 4, 5000, timestamp=3.0))
    piper.ParseCANFrame(_low_spd_frame_1(1 << 6, timestamp=4.0))

    snap = piper.GetArmStateSnapshot()
    assert snap["joint_deg"][0] == 1000
    assert snap["joint_deg"][1] == 2000
    assert snap["gripper_angle"] == 3000
    assert snap["joint_velocity"][0] == 10
    assert snap["joint_effort"][0] == 4 * 1.18125
    assert snap["driver_enable"][0] is True
    assert snap["driver_enable"][1] is False

    enable = piper.GetArmEnableStatus()
    assert isinstance(enable, list) and len(enable) == 6
    assert enable[0] is True


def test_minimal_mode_skips_wrapper_updates_and_shortcuts_getters():
    piper = _make_interface("test_minimal_002", minimal_feedback_mode=True)

    piper.ParseCANFrame(_joint_frame_12(12345, 6789))
    assert piper.GetArmJointMsgs().joint_state.joint_1 != 12345, \
        "minimal mode should skip the wrapper-object joint update"

    piper.ParseCANFrame(_gripper_frame(2222, 1))
    assert piper.GetArmGripperMsgs().gripper_state.grippers_angle != 2222

    assert piper.GetArmEnableStatus() == [False] * 6

    piper.ParseCANFrame(_low_spd_frame_1(1 << 6))
    assert piper.GetArmEnableStatus()[0] is True


def test_non_minimal_mode_uses_wrapper_updates():
    piper = _make_interface("test_nonminimal_001")

    piper.ParseCANFrame(_joint_frame_12(12345, 6789))
    assert piper.GetArmJointMsgs().joint_state.joint_1 == 12345

    piper.ParseCANFrame(_gripper_frame(2222, 1))
    assert piper.GetArmGripperMsgs().gripper_state.grippers_angle == 2222


def test_non_minimal_get_arm_state_snapshot_reads_wrapper_state():
    piper = _make_interface("test_nonminimal_snap_001")

    piper.ParseCANFrame(_joint_frame_12(1000, 2000, timestamp=1.0))
    piper.ParseCANFrame(_gripper_frame(3000, 5, timestamp=2.0))
    piper.ParseCANFrame(_high_spd_frame_1(10, 4, 5000, timestamp=3.0))
    piper.ParseCANFrame(_low_spd_frame_1(1 << 6, timestamp=4.0))

    snap = piper.GetArmStateSnapshot()
    assert snap["joint_deg"][0] == 1000
    assert snap["joint_deg"][1] == 2000
    assert snap["gripper_angle"] == 3000
    assert snap["joint_velocity"][0] == 10
    assert snap["joint_effort"][0] == 4 * 1.18125
    assert snap["driver_enable"][0] is True

    assert piper.GetArmEnableStatus()[0] is True

    reused = piper.GetArmStateSnapshot()
    piper.GetArmStateSnapshot(snapshot=reused)
    assert reused["joint_deg"][0] == 1000
