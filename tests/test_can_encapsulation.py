import platform

import can
import pytest

from piper_sdk.hardware_port.can_encapsulation_v0_4_0 import C_STD_CAN


class FakeBus(can.BusABC):
    '''Records bus lifecycle and message traffic for C_STD_CAN.'''

    def __init__(self, channel=None, bustype=None, bitrate=None,
                 receive_own_messages=None, local_loopback=None):
        super().__init__(channel=channel, bustype=bustype, bitrate=bitrate,
                         receive_own_messages=receive_own_messages,
                         local_loopback=local_loopback)
        self.channel_arg = channel
        self.bustype_arg = bustype
        self.receive_own_messages_arg = receive_own_messages
        self.local_loopback_arg = local_loopback
        self.shutdown_calls = 0
        self.recv_calls = 0
        self.send_calls = 0
        self._state = can.BusState.ACTIVE

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        self._state = new_state

    def recv(self, timeout=None):
        self.recv_calls += 1
        return None

    def send(self, *args, **kwargs):
        self.send_calls += 1

    def shutdown(self, *args, **kwargs):
        self.shutdown_calls += 1


@pytest.fixture
def bus_factory(monkeypatch):
    created = []

    def make_bus(**kwargs):
        bus = FakeBus(**kwargs)
        created.append(bus)
        return bus

    def fail_bus(**kwargs):
        raise can.CanError("simulated init failure")

    monkeypatch.setattr(can.interface, "Bus", make_bus)
    monkeypatch.setattr(platform, "system", lambda: "Linux")
    return created


def _make_can(bus_factory):
    return C_STD_CAN(channel_name="can0", bustype="socketcan",
                     expected_bitrate=1000000, judge_flag=False, auto_init=True)


def test_linux_opens_two_buses_and_routes_traffic(bus_factory):
    can_obj = _make_can(bus_factory)
    assert len(bus_factory) == 2
    recv_bus, send_bus = bus_factory

    assert can_obj.ReadCanMessage() == C_STD_CAN.CAN_STATUS.READ_CAN_MSG_TIMEOUT
    assert recv_bus.recv_calls == 1
    assert send_bus.recv_calls == 0

    assert can_obj.SendCanMessage(0x101, [0] * 8) == C_STD_CAN.CAN_STATUS.SEND_MESSAGE_SUCCESS
    assert send_bus.send_calls == 1
    assert recv_bus.send_calls == 0

    assert can_obj.Close() == C_STD_CAN.CAN_STATUS.CLOSE_CAN_BUS_CONNECT_SHUT_DOWN
    assert recv_bus.shutdown_calls == 1
    assert send_bus.shutdown_calls == 1


def test_windows_shares_single_bus_and_shuts_down_once(monkeypatch):
    created = []

    def make_bus(**kwargs):
        bus = FakeBus(**kwargs)
        created.append(bus)
        return bus

    monkeypatch.setattr(can.interface, "Bus", make_bus)
    for os_name in ("Windows", "Darwin"):
        monkeypatch.setattr(platform, "system", lambda: os_name)
        created.clear()

        can_obj = C_STD_CAN(channel_name="can0", judge_flag=False, auto_init=True)
        assert len(created) == 1, f"{os_name} must share a single bus"

        assert can_obj.Close() == C_STD_CAN.CAN_STATUS.CLOSE_CAN_BUS_CONNECT_SHUT_DOWN
        assert created[0].shutdown_calls == 1


def test_bus_created_without_self_receive_or_loopback(bus_factory):
    _make_can(bus_factory)
    for bus in bus_factory:
        assert bus.receive_own_messages_arg is False
        assert bus.local_loopback_arg is False


def test_init_failure_cleans_up_partial_buses(monkeypatch, bus_factory):
    def fail_second_bus(**kwargs):
        if len(bus_factory) == 0:
            bus = FakeBus(**kwargs)
            bus_factory.append(bus)
            return bus
        raise can.CanError("simulated send bus failure")

    monkeypatch.setattr(can.interface, "Bus", fail_second_bus)
    can_obj = C_STD_CAN(channel_name="can0", judge_flag=False, auto_init=True)

    assert can_obj.Init() == C_STD_CAN.CAN_STATUS.INIT_CAN_BUS_OPENED_FAILED
    assert len(bus_factory) == 1
    assert bus_factory[0].shutdown_calls == 1


def test_interface_exposes_rx_tx_message_instances():
    from piper_sdk.interface import C_PiperInterface_V2
    from piper_sdk.piper_msgs.msg_v2 import PiperMessage

    piper = C_PiperInterface_V2(can_name="test_port_062", can_auto_init=False,
                                judge_flag=False, log_to_file=False)
    assert isinstance(piper.rx_msg, PiperMessage)
    assert isinstance(piper.tx_msg, PiperMessage)
