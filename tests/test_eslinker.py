import asyncio
from unittest.mock import AsyncMock, MagicMock

import pytest

from pyftms.client.backends.eslinker import (
    CHALLENGE_SEED,
    HANDSHAKE,
    HELLO,
    ESLinkerController,
    button_event,
    challenge_reply,
    inclination_frame,
    speed_frame,
    split_frames,
)
from pyftms.client.backends.updater import DataUpdater
from pyftms.models import ResultCode, TreadmillData


@pytest.mark.parametrize(
    "speed, frame",
    [
        (0, b"\x01\x01\x00"),
        (0.8, b"\x01\x01\x08"),
        (1.5, b"\x01\x01\x0f"),
        (2.0, b"\x01\x01\x14"),
        (12.0, b"\x01\x01\x78"),
    ],
)
def test_speed_frame(speed: float, frame: bytes):
    assert speed_frame(speed) == frame


def test_inclination_frame():
    assert inclination_frame() == b"\x04\x01\x00"
    assert inclination_frame(3) == b"\x04\x01\x03"


@pytest.mark.parametrize(
    "data, reply",
    [
        (b"\x08\x01\xff", b"\x08\x01\x01"),
        (bytes([8, 3, 7, 7 ^ 245, 7 ^ 222]), bytes([8, 3, 7 ^ 222, 7 ^ 222 ^ 245, 7 ^ 222 ^ 66])),
        (b"\x08\x03\x07\x00\x00", None),
        (b"\x08\x04\x01\x00\x00\x01", None),
        (b"\x02\x01\x00", None),
    ],
)
def test_challenge_reply(data: bytes, reply: bytes | None):
    assert challenge_reply(data) == reply


def test_split_frames():
    assert split_frames(b"\x05\x01\x03\x09\x02\x08\x78") == [b"\x05\x01\x03", b"\x09\x02\x08\x78"]
    assert split_frames(b"\x08\x04\x01\x00\x00\x01") == [b"\x08\x04\x01\x00\x00\x01"]
    assert split_frames(b"\x02\x01\x00\x05\x01\x02") == [b"\x02\x01\x00", b"\x05\x01\x02"]
    assert split_frames(b"") == []


def test_button_event():
    assert button_event(b"\x05\x01\x02") == 2
    assert button_event(b"\x02\x01\x00") is None


def _client(name: str = "ESLinker-31B33D", vendor: bool = True) -> MagicMock:
    cli = MagicMock()
    cli.is_connected = True
    cli.services.get_characteristic = lambda uuid: (
        object() if vendor and uuid in ("fff1", "fff2") else None
    )
    cli.write_gatt_char = AsyncMock()
    cli.start_notify = AsyncMock()
    return cli


def test_supported():
    assert ESLinkerController.supported(_client(), "ESLinker-31B33D")
    assert not ESLinkerController.supported(_client(), "FS-12345")  # FitShow, same UUIDs
    assert ESLinkerController.supported(_client(), "FS-12345", force=True)
    assert not ESLinkerController.supported(_client(vendor=False), "ESLinker-31B33D")
    assert not ESLinkerController.supported(_client(vendor=False), "ESLinker-31B33D", force=True)


def test_protocol_option():
    from bleak.backends.device import BLEDevice

    from pyftms import Protocol, get_client
    from pyftms.client.machines import Treadmill
    from pyftms.client.properties import MachineType

    dev = BLEDevice("04:57:91:31:B3:3D", "ESLinker-31B33D", None)
    assert get_client(dev, MachineType.TREADMILL).protocol == "auto"

    for protocol in ("auto", "ftms", "eslinker"):
        tm = Treadmill(dev, protocol=protocol)
        assert tm.protocol == protocol
        assert tm._eslinker is None  # detection happens on connect

    assert Protocol.__value__.__args__ == ("auto", "ftms", "eslinker")


def test_set_speed_handshake_and_events():
    async def run():
        events = []
        cli = _client()
        ctl = ESLinkerController(events.append, keepalive=0.05)

        assert not ctl.active
        assert await ctl.set_speed(cli, 2.0) == ResultCode.SUCCESS
        assert ctl.active and ctl.speed == 2.0

        writes = [call.args[1] for call in cli.write_gatt_char.call_args_list]
        assert writes == [HELLO, *HANDSHAKE, CHALLENGE_SEED, speed_frame(2.0)]
        cli.start_notify.assert_awaited_once()

        assert [e.event_id for e in events] == ["update", "setup", "update", "start"]
        assert events[0].event_data == {"distance_total": 0, "time_elapsed": 0}
        assert events[1].event_data == {"target_speed": 2.0}
        assert events[2].event_data == {"speed_instant": 2.0}

        # keepalive uses the inclination frame, never repeats the speed
        await asyncio.sleep(0.12)
        tail = [call.args[1] for call in cli.write_gatt_char.call_args_list][6:]
        assert tail and set(tail) == {inclination_frame()}

        # second command: no handshake again, stop event on transition to 0
        events.clear()
        await ctl.stop(cli)
        assert cli.write_gatt_char.call_args_list[-1].args[1] == speed_frame(0)
        assert cli.start_notify.await_count == 1
        assert [e.event_id for e in events] == ["setup", "update", "stop"]

        ctl.reset()
        assert not ctl.active and ctl.speed == 0

    asyncio.run(run())


def test_notify_challenge_and_console_buttons():
    async def run():
        events, speeds = [], []
        cli = _client()
        ctl = ESLinkerController(
            events.append, on_data=lambda **v: speeds.append(v["speed_instant"]) if "speed_instant" in v else None, keepalive=10
        )
        await ctl.set_speed(cli, 1.5)
        events.clear()

        ctl._on_notify(MagicMock(), bytearray(b"\x08\x01\xff"))
        await asyncio.sleep(0)
        assert cli.write_gatt_char.call_args_list[-1].args[1] == HELLO

        async def press(n: int):
            ctl._on_notify(MagicMock(), bytearray([5, 1, n]))
            await asyncio.sleep(0.01)
            return cli.write_gatt_char.call_args_list[-1].args[1]

        # concatenated notification: button + ranges reply
        ctl._on_notify(MagicMock(), bytearray(b"\x05\x01\x03\x09\x02\x08\x78"))
        await asyncio.sleep(0.01)
        assert cli.write_gatt_char.call_args_list[-1].args[1] == speed_frame(1.6)
        assert await press(4) == speed_frame(1.5)

        # the console only reports buttons, so we must write the resulting speed
        assert await press(3) == speed_frame(1.6)  # speed up
        assert await press(4) == speed_frame(1.5)  # speed down
        assert await press(2) == speed_frame(0)  # stop
        assert ctl.speed == 0
        assert await press(1) == speed_frame(0.8)  # start: minimum speed, like the app
        n = len(cli.write_gatt_char.call_args_list)
        await press(1)  # start while running: ignored
        assert len(cli.write_gatt_char.call_args_list) == n

        assert [e.event_id for e in events if e.event_id in ("start", "stop")] == ["stop", "start"]
        assert all(e.event_source == "user" for e in events if e.event_id in ("start", "stop"))
        assert speeds == [1.5, 1.6, 1.5, 1.6, 1.5, 0.0, 0.8]

        # speed-reached report, heart rate, fault
        events.clear()
        ctl._on_notify(MagicMock(), bytearray(b"\xe0\x01\x0c"))
        assert ctl.speed == 1.2 and events[-1].event_data == {"speed_instant": 1.2}
        ctl._on_notify(MagicMock(), bytearray(b"\x02\x01\x00"))
        ctl._on_notify(MagicMock(), bytearray(b"\x02\x01\x48"))
        assert events[-1].event_data == {"heart_rate": 72}
        ctl._on_notify(MagicMock(), bytearray(b"\x03\x01\x07"))  # safety key off
        assert ctl.speed == 0
        assert events[-1] == ("stop", "safety")
        n = len(cli.write_gatt_char.call_args_list)
        await asyncio.sleep(0.01)
        assert len(cli.write_gatt_char.call_args_list) == n  # nothing written on a fault

        ctl.reset()

    asyncio.run(run())


def test_speed_clamped_to_range():
    async def run():
        cli = _client()
        ctl = ESLinkerController(lambda e: None, min_speed=0.8, max_speed=12.0, keepalive=10)
        await ctl.set_speed(cli, 0.3)
        assert cli.write_gatt_char.call_args_list[-1].args[1] == speed_frame(0.8)
        await ctl.set_speed(cli, 20)
        assert cli.write_gatt_char.call_args_list[-1].args[1] == speed_frame(12.0)
        await ctl.set_speed(cli, 0)
        assert cli.write_gatt_char.call_args_list[-1].args[1] == speed_frame(0)

        # range reported by the machine replaces the defaults
        ctl._on_notify(MagicMock(), bytearray(b"\x09\x02\x0a\x3c\x09\x01\x00"))
        assert ctl.min_speed == 1.0
        await ctl.set_speed(cli, 20)
        assert cli.write_gatt_char.call_args_list[-1].args[1] == speed_frame(6.0)
        ctl.reset()

    asyncio.run(run())


def test_updater_override():
    events = []
    updater = DataUpdater(TreadmillData, events.append)
    updater.set_override(speed_instant=2.0)

    # flags 0x00 (instantaneous speed present), speed 0.8 km/h, stale
    updater._on_notify(MagicMock(), bytearray(b"\x00\x00\x50\x00"))

    assert events[0].event_data["speed_instant"] == 2.0

    updater.reset()
    updater._on_notify(MagicMock(), bytearray(b"\x00\x00\x50\x00"))

    assert events[-1].event_data["speed_instant"] == 0.8


def test_session_distance_and_time():
    async def run():
        events, data = [], {}
        cli = _client()
        ctl = ESLinkerController(events.append, on_data=lambda **v: data.update(v), keepalive=10)

        await ctl.set_speed(cli, 3.6)  # 1 m/s
        t0 = ctl._last_tick
        assert data["distance_total"] == 0 and data["time_elapsed"] == 0

        ctl._tick(t0 + 10)
        assert data == {"speed_instant": 3.6, "distance_total": 10, "time_elapsed": 10}
        assert events[-1].event_data == {"distance_total": 10, "time_elapsed": 10}

        # speed change accounts for the time at the old speed first
        ctl._last_tick = t0  # pretend 10 s since the last tick again
        ctl._tick(t0 + 10)
        assert data["distance_total"] == 20
        await ctl.set_speed(cli, 7.2)
        ctl._tick(ctl._last_tick + 5)
        assert data["distance_total"] == 30 and data["time_elapsed"] == 25

        # pause keeps the session, stop ends it; the next start resets
        await ctl.pause(cli)
        ctl._tick(ctl._last_tick + 60)
        assert data["distance_total"] == 30 and data["time_elapsed"] == 25
        await ctl.set_speed(cli, 3.6)
        assert data["distance_total"] == 30
        await ctl.stop(cli)
        assert data["distance_total"] == 30  # final value stays until the next start
        await ctl.set_speed(cli, 3.6)
        assert data["distance_total"] == 0 and data["time_elapsed"] == 0

        # console stop ends the session too
        ctl._on_notify(MagicMock(), bytearray(b"\x05\x01\x02"))
        await asyncio.sleep(0.01)
        assert ctl.speed == 0 and not ctl._session

        ctl.reset()

    asyncio.run(run())
