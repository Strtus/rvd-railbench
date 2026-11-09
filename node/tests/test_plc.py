from __future__ import annotations

import copy
import struct

import pytest

from rvd_railnode.backend import InterlockError
from rvd_railnode.plc import Modbus, ModbusError, PlcBackend
from rvd_railnode.profile import ProfileError, validate

# Synthetic register map; it does not describe any real installation.
PROFILE = validate({
    "driver": "modbus",
    "modbus": {"host": "plc.invalid", "d_base": 1000, "m_base": 100},
    "interlock": {"identity": {"d": 9, "value": 42}, "emergency_m": 0},
    "axes": {
        "x": {"type": "pulse", "target_d": 10, "speed_d": 11, "encoder_d": 12, "start_m": 3, "stop_m": 40,
              "scale": 1e-4, "zero": 0, "min_count": 0, "max_count": 100000, "max_speed": 0.2, "tolerance": 1e-3},
        "z": {"type": "servo", "target_d": 20, "speed_d": 21, "encoder_d": 22, "enable_m": 50, "stop_m": 52,
              "quickstop_m": 51, "enabled_d": 23, "closed_d": 24, "pending_d": 25, "sample_count_d": 26,
              "scale": 1e-5, "zero": 50000, "min_count": 50000, "max_count": 90000, "max_speed": 0.05,
              "tolerance": 1e-4, "ready": [{"d": 27, "value": 2}]},
    },
})


class FakePlc:
    """Register-level stand-in for the PLC: start edges move pulse axes, enable and stop bits drive servo state."""

    def __init__(self, profile=PROFILE):
        self.axes = profile["axes"]
        self.d = {9: 42, 27: 2}
        self.m = {}
        self.writes = []
        self.generation = 1
        self.connected = True
        self.reachable = True
        self.fail_next = 0
        for spec in self.axes.values():
            self.d[spec["encoder_d"]] = spec["min_count"] + 1000
            if spec["type"] == "servo":
                self.d.update({spec["enabled_d"]: 0, spec["closed_d"]: 1, spec["pending_d"]: 0,
                               spec["sample_count_d"]: 7})

    def _io(self):
        if not self.connected:
            raise ModbusError("PLC is unreachable")
        if self.fail_next:
            self.fail_next -= 1
            self.connected = False
            raise ModbusError("PLC I/O failed: timed out")

    def ensure(self, retry_now=False):
        if not self.connected:
            if not (retry_now and self.reachable):
                raise ModbusError("PLC is unreachable")
            self.connected = True
            self.generation += 1

    def close(self):
        self.connected = False

    def read_d(self, d):
        self._io()
        return self.d.get(d, 0)

    def read_m(self, m):
        self._io()
        return self.m.get(m, False)

    def write_d(self, d, value):
        self._io()
        self.writes.append(("d", d, value))
        self.d[d] = value

    def write_m(self, m, value):
        self._io()
        self.writes.append(("m", m, value))
        before, self.m[m] = self.m.get(m, False), value
        if value and not before:
            self._edge(m)

    def pulse_m(self, m):
        self.write_m(m, False)
        self.write_m(m, True)

    def _edge(self, m):
        for spec in self.axes.values():
            if spec["type"] == "pulse" and m == spec["start_m"]:
                self.d[spec["encoder_d"]] = self.d[spec["target_d"]]
            elif spec["type"] == "servo" and m == spec["enable_m"]:
                self.d[spec["enabled_d"]], self.d[spec["closed_d"]] = 1, 0
            elif spec["type"] == "servo" and m == spec["stop_m"]:
                self.d[spec["enabled_d"]], self.d[spec["closed_d"]] = 0, 1


def backend(plc, motion=True):
    return PlcBackend(PROFILE, motion, io=plc, sleep=lambda _: None)


def test_reading_maps_counts_to_physical_positions():
    plc = FakePlc()
    sample = backend(plc, motion=False).read()
    assert sample.positions == pytest.approx({"x": 0.1, "z": 0.01})
    assert sample.raw == {"x": 1000, "z": 51000}
    assert plc.writes == []


def test_a_read_only_backend_never_writes():
    plc = FakePlc()
    plc_backend = backend(plc, motion=False)
    with pytest.raises(ModbusError, match="read-only"):
        plc_backend.arm(["x"])
    with pytest.raises(ModbusError, match="read-only"):
        plc_backend.command({"x": 0.2}, {"x": 0.1})
    plc_backend.stop(force=True)
    plc_backend.close()
    assert plc.writes == []


@pytest.mark.parametrize("register, value, message", [((9,), 41, "expects 42"), (("m", 0), True, "emergency")])
def test_interlocks_block_reads_and_motion(register, value, message):
    plc = FakePlc()
    if register[0] == "m":
        plc.m[register[1]] = value
    else:
        plc.d[register[0]] = value
    plc_backend = backend(plc)
    with pytest.raises(InterlockError, match=message):
        plc_backend.read()
    with pytest.raises(InterlockError, match=message):
        plc_backend.arm(["x", "z"])
    assert plc.writes == []


def test_arming_a_servo_holds_its_position_and_waits_for_the_drive():
    plc = FakePlc()
    plc_backend = backend(plc)
    plc_backend.arm(["x", "z"])
    assert plc.d[20] == 51000 and plc.d[23] == 1
    assert plc_backend.armed == {"x", "z"} and plc_backend.active
    assert ("m", 3, True) not in plc.writes  # a pulse axis gets no start edge until it has a target


def test_an_axis_that_is_not_ready_is_never_enabled():
    plc = FakePlc()
    plc.d[27] = 0
    with pytest.raises(ModbusError, match="not ready"):
        backend(plc).arm(["z"])
    assert plc.writes == []


def test_an_enable_that_never_arrives_is_reported():
    plc = FakePlc()
    plc._edge = lambda m: None
    clock = iter(range(100))
    plc_backend = PlcBackend(PROFILE, True, io=plc, sleep=lambda _: None, clock=lambda: float(next(clock)))
    with pytest.raises(ModbusError, match="enable was not confirmed"):
        plc_backend.arm(["z"])
    assert plc_backend.active  # the node must follow with a stop


def test_commands_convert_units_and_restart_pulse_axes():
    plc = FakePlc()
    plc_backend = backend(plc)
    plc_backend.arm(["x", "z"])
    plc.writes.clear()
    plc_backend.command({"x": 0.25, "z": 0.02}, {"x": 0.1, "z": 0.01})
    assert plc.d[10] == 2500 and plc.d[11] == 1000
    assert plc.d[20] == 52000 and plc.d[21] == 1000
    assert ("m", 3, True) in plc.writes and plc.d[12] == 2500
    assert ("m", 50, True) not in plc.writes


@pytest.mark.parametrize("targets, speeds, message", [
    ({"x": 20.0}, {"x": 0.1}, "outside"),
    ({"x": 0.2}, {"x": 0.3}, "speed"),
    ({"z": 0.02}, {"z": 0.01}, "not armed"),
])
def test_bad_commands_write_nothing(targets, speeds, message):
    plc = FakePlc()
    plc_backend = backend(plc)
    plc_backend.arm(["x"])
    plc.writes.clear()
    with pytest.raises(ModbusError, match=message):
        plc_backend.command(targets, speeds)
    assert plc.writes == []


def test_stop_is_confirmed_and_leaves_no_pending_target():
    plc = FakePlc()
    plc_backend = backend(plc)
    plc_backend.arm(["x", "z"])
    plc_backend.command({"x": 0.25}, {"x": 0.1})
    plc_backend.stop()
    assert not plc_backend.active and not plc_backend.armed
    assert plc.d[23] == 0 and plc.d[24] == 1
    assert plc.d[10] == plc.d[12] and plc.d[11] == 0 and plc.d[21] == 0


def test_stop_reconnects_after_the_link_dropped():
    plc = FakePlc()
    plc_backend = backend(plc)
    plc_backend.arm(["x", "z"])
    plc_backend.command({"x": 0.25}, {"x": 0.1})
    plc.fail_next = 1
    with pytest.raises(ModbusError):
        plc_backend.read()
    assert not plc.connected
    plc_backend.stop()
    assert plc.connected and not plc_backend.active and plc.d[23] == 0


def test_stop_reports_failure_when_the_plc_stays_unreachable():
    plc = FakePlc()
    plc_backend = backend(plc)
    plc_backend.arm(["x"])
    plc.connected, plc.reachable = False, False
    with pytest.raises(ModbusError, match="stop not delivered"):
        plc_backend.stop()
    assert plc_backend.active


class Wire:
    """A socket that answers like a PLC and can time out once."""

    def __init__(self, words=(0, 0)):
        self.words, self.sent, self.reply, self.timeouts = words, [], b"", 0

    def settimeout(self, _):
        pass

    def sendall(self, frame):
        if self.timeouts:
            self.timeouts -= 1
            raise TimeoutError("timed out")
        self.sent.append(frame)
        tid, _, _, unit = struct.unpack(">HHHB", frame[:7])
        pdu = frame[7:]
        if pdu[0] == 3:
            count = struct.unpack(">H", pdu[3:5])[0]
            body = bytes([3, 2 * count]) + struct.pack(f">{count}H", *self.words[:count])
        elif pdu[0] == 16:
            body = pdu[:5]
        else:
            body = bytes([pdu[0] | 0x80, 1])
        self.reply = struct.pack(">HHHB", tid, 0, len(body) + 1, unit) + body

    def recv(self, n):
        chunk, self.reply = self.reply[:n], self.reply[n:]
        return chunk

    def close(self):
        pass


def wire_modbus(wire, **options):
    connects = []

    def connect(address, timeout):
        connects.append(address)
        return wire

    return Modbus("plc.invalid", d_base=6842, m_base=250, connect=connect, retry_s=0.0, **options), connects


@pytest.mark.parametrize("order, words, value", [("low_first", (0xFFFE, 0xFFFF), -2),
                                                 ("high_first", (0x0001, 0x0002), 0x10002)])
def test_word_order(order, words, value):
    modbus, _ = wire_modbus(Wire(words), word_order=order)
    assert modbus.read_d(199) == value


def test_writes_are_blocked_on_a_read_only_link_before_anything_is_sent():
    wire = Wire()
    modbus, _ = wire_modbus(wire)
    with pytest.raises(ModbusError, match="disabled"):
        modbus.write_d(100, 7)
    assert wire.sent == []
    modbus.allow_writes = True
    modbus.write_d(100, 0x12345678)
    assert wire.sent[-1][-4:] == b"\x56\x78\x12\x34"


def test_an_exception_reply_keeps_the_link_but_a_timeout_reconnects():
    wire = Wire()
    modbus, connects = wire_modbus(wire, allow_writes=True)
    with pytest.raises(ModbusError, match="rejected function 6"):
        modbus.transact(struct.pack(">BHH", 6, 250, 1))
    assert modbus.connected and len(connects) == 1
    wire.timeouts = 1
    with pytest.raises(ModbusError, match="timed out"):
        modbus.read_d(1)
    assert not modbus.connected
    modbus.read_d(1)
    assert modbus.connected and len(connects) == 2


def test_a_held_servo_stays_enabled_where_it_stopped():
    plc = FakePlc()
    plc_backend = backend(plc)
    plc_backend.arm(["x", "z"])
    plc_backend.command({"x": 0.25, "z": 0.02}, {"x": 0.1, "z": 0.01})
    plc.d[22] = 51500  # the servo is on its way
    plc.writes.clear()
    plc_backend.stop(keep={"z"})
    assert plc.d[23] == 1 and plc.d[20] == 51500 and plc.d[21] != 0
    assert plc_backend.armed == {"z"} and plc_backend.active
    assert ("m", 3, False) in plc.writes and ("m", 50, False) not in plc.writes
    plc_backend.stop()
    assert plc.d[23] == 0 and not plc_backend.active


def test_only_servo_axes_may_hold():
    profile = copy.deepcopy(PROFILE)
    profile["axes"]["x"]["hold"] = True
    with pytest.raises(ProfileError, match="only servo"):
        validate(profile)
