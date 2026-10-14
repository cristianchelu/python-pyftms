# Copyright 2024-2025, Sergey Dudanov
# SPDX-License-Identifier: Apache-2.0

"""
Speed control for treadmills with ESLinker BLE modules (YIJIAN and others).

These modules implement FTMS, but the Control Point only honours Start/Resume and
Stop/Pause: Set Target Speed is silently ignored. The vendor app ("HK Treadmill",
Shanghai Hankang, binds to names containing "ESLinker") sets speed through the
module's vendor service instead. Protocol from that app's source; qdomyos-zwift has
a partial version of it as its "Cadenza T45" variant.

    Service FFF0: FFF1 notify (machine -> client), FFF2 write (client -> machine).
    Frame: <cmd> <len> <payload...>

    -> 08 01 01                 hello        <- 08 04 01 00 00 01
    -> 08 04 01 00 00 01        handshake    <- 08 02 xx xx (optional)
    -> 09 01 01, 09 01 02       ranges       <- 09 02 <min> <max> (0.1 km/h), 09 01 <max incl>
    -> 08 03 x x^245 x^66       challenge reply, x from the challenge below (0 unsolicited)
    -> 01 01 <km/h * 10>        target speed, 0 stops the belt. There is no start command.
    -> 04 01 <%>                target inclination
    <- 08 01 FF                 challenge, reply 08 01 01
    <- 08 03 a b c              challenge (b = a ^ 245, c = a ^ 222), reply 08 03 c c^245 c^66
    <- 05 01 n                  console button: 1 start, 2 stop, 3 speed+, 4 speed-, 5 incl+, 6 incl-
    <- E0 01 <km/h * 10>        speed reached (not seen on the YIJIAN 42AN)
    <- E1 01 <%>                inclination reached
    <- 03 01 <code>             fault; the console has stopped
    <- 02 01 <bpm>              heart rate, every second

The app sends speed and inclination alternately every 500 ms, speed 0 while not
started, and restarts at the minimum speed after every stop. Its 3-2-1 countdown is
app-side; the console does not beep before a vendor-channel start. There is no
distance or time report either: the app integrates the commanded speed, and so does
this controller (session distance and elapsed time, reset on the start after a stop,
kept across a pause).

Observed on a YIJIAN 42AN (ESLinker F-9788, firmware 6.1.2):

* A speed command starts the belt directly at that speed, no countdown.
* The console stops the belt about half a minute after the last frame, so something
  must be written periodically. The inclination frame is used for that, so a stop
  pressed on the console is never overridden by a repeated speed command.
* Once anything has been written to FFF2, the FTMS Control Point ignores
  Start/Resume for the rest of the connection and the FTMS Treadmill Data speed
  freezes. From then on this controller owns start/stop and reports the
  commanded speed itself.
* In that mode the console no longer acts on its own buttons (remote included):
  it reports them as `05 01 n` and expects the client to apply them. This
  controller does, so a stop press still stops the belt.
"""

import asyncio
import logging
import time
from typing import Any, Callable

from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

from ...models import ResultCode
from ..const import (
    DISTANCE_TOTAL,
    HEART_RATE,
    SPEED_INSTANT,
    TARGET_SPEED,
    TIME_ELAPSED,
)
from .event import ControlEvent, ControlSource, FtmsCallback, SetupEvent, UpdateEvent

_LOGGER = logging.getLogger(__name__)

ESLINKER_SERVICE_UUID = "fff0"
ESLINKER_NOTIFY_UUID = "fff1"
ESLINKER_WRITE_UUID = "fff2"

ESLINKER_NAME_PREFIX = "ESLINKER"
"""Advertised name prefix. FitShow machines use the same UUIDs, so the name matters."""

HELLO = bytes([0x08, 0x01, 0x01])
HELLO_RESPONSE = bytes([0x08, 0x04, 0x01, 0x00, 0x00, 0x01])
HANDSHAKE = (
    bytes([0x08, 0x04, 0x01, 0x00, 0x00, 0x01]),
    bytes([0x09, 0x01, 0x01]),
    bytes([0x09, 0x01, 0x02]),
)
CHALLENGE_SEED = bytes([0x08, 0x03, 0x00, 0x00 ^ 245, 0x00 ^ 66])

BUTTON_START, BUTTON_STOP, BUTTON_SPEED_UP, BUTTON_SPEED_DOWN = 1, 2, 3, 4

FAULTS = {
    1: "communication failure",
    2: "IGBT short circuit",
    3: "speed sensor failure",
    4: "incline self-test failure",
    5: "overload",
    7: "safety key off",
    8: "drive failure",
    9: "no data returned",
}
"""Fault codes of `03 01 <code>`, from the HK Treadmill app (E01..E09 on the console)."""

BUTTON_SPEED_STEP = 0.1
"""Speed change per console speed button event (a short press). Units: `km/h`."""

KEEPALIVE_INTERVAL = 2.0
"""Seconds between keepalive frames. The console gives up after roughly 30 s."""


def speed_frame(speed: float) -> bytes:
    """Target speed frame. Units: `km/h`, 0.1 km/h resolution."""
    return bytes([0x01, 0x01, max(0, min(255, round(speed * 10)))])


def inclination_frame(inclination: float = 0) -> bytes:
    """Target inclination frame. Units: `%`."""
    return bytes([0x04, 0x01, max(0, min(255, round(inclination)))])


def challenge_reply(data: bytes) -> bytes | None:
    """Reply for a module challenge frame, or `None` if `data` is not one."""
    if data == b"\x08\x01\xff":
        return HELLO

    if (
        len(data) == 5
        and data[:2] == b"\x08\x03"
        and data[3] == data[2] ^ 245
        and data[4] == data[2] ^ 222
    ):
        x = data[4]
        return bytes([0x08, 0x03, x, x ^ 245, x ^ 66])

    return None


def split_frames(data: bytes) -> list[bytes]:
    """Split a notification into `<cmd> <len> <payload>` frames (they get concatenated)."""
    frames = []

    while len(data) >= 2:
        frame, data = data[: 2 + data[1]], data[2 + data[1] :]
        frames.append(frame)

    return frames


def button_event(data: bytes) -> int | None:
    """Console button code from a `05 01 n` frame, or `None`."""
    if len(data) == 3 and data[:2] == b"\x05\x01":
        return data[2]

    return None


def value_event(data: bytes, cmd: int) -> int | None:
    """Payload of a one-byte frame `<cmd> 01 <value>`, or `None`."""
    if len(data) == 3 and data[0] == cmd and data[1] == 1:
        return data[2]

    return None


class ESLinkerController:
    """Vendor-channel speed control for ESLinker modules."""

    _cli: BleakClient | None

    def __init__(
        self,
        callback: FtmsCallback,
        *,
        on_data: Callable[..., None] | None = None,
        min_speed: float = 0.8,
        max_speed: float = 12.0,
        keepalive: float = KEEPALIVE_INTERVAL,
    ) -> None:
        """`on_data(**values)` receives every value this controller reports."""
        self._cb = callback
        self._on_data = on_data
        self._min_speed = min_speed
        self._max_speed = max_speed
        self._keepalive = keepalive
        self._cli = None
        self._ready = False
        self._speed = 0.0
        self._task: asyncio.Task[None] | None = None
        self._lock = asyncio.Lock()

        # Session counters, integrated from the commanded speed
        self._session = False
        self._distance = 0.0
        self._time = 0.0
        self._last_tick = time.monotonic()
        self._published: dict[str, Any] = {}

    @staticmethod
    def supported(cli: BleakClient, name: str, *, force: bool = False) -> bool:
        """
        `True` if the connected device is an ESLinker module: the vendor
        characteristics are present and the name says so (`force` skips the name).
        """
        return (force or name.upper().startswith(ESLINKER_NAME_PREFIX)) and all(
            cli.services.get_characteristic(uuid) is not None
            for uuid in (ESLINKER_NOTIFY_UUID, ESLINKER_WRITE_UUID)
        )

    @property
    def active(self) -> bool:
        """`True` once the vendor channel has been used on this connection."""
        return self._ready

    @property
    def speed(self) -> float:
        """Current commanded speed, `0` when stopped. Units: `km/h`."""
        return self._speed

    @property
    def min_speed(self) -> float:
        """Minimum speed, as reported by the machine. Units: `km/h`."""
        return self._min_speed

    @property
    def distance(self) -> float:
        """Session distance, integrated from the commanded speed. Units: `m`."""
        return self._distance

    @property
    def time_elapsed(self) -> float:
        """Session time with the belt moving. Units: `s`."""
        return self._time

    def reset(self) -> None:
        """Resetting state. Call while disconnection event."""
        if self._task:
            self._task.cancel()

        self._task = None
        self._cli = None
        self._ready = False
        self._speed = 0.0

    async def set_speed(
        self,
        cli: BleakClient,
        speed: float,
        *,
        source: ControlSource = "callback",
        end_session: bool = True,
    ) -> ResultCode:
        """
        Sets target speed. Starts the belt if it is stopped, stops it with `0`.
        A stop ends the session unless `end_session` is `False` (pause).

        Units: `km/h`.
        """
        if speed > 0:
            speed = round(min(max(speed, self._min_speed), self._max_speed), 1)
        else:
            speed = 0.0

        async with self._lock:
            await self._ensure_ready(cli)
            await self._write(speed_frame(speed))

            previous = self._set_speed_state(speed, end_session)

        self._cb(
            SetupEvent(
                event_id="setup",
                event_data={TARGET_SPEED: speed},
                event_source=source,
            )
        )
        self._report(speed, previous, source)

        return ResultCode.SUCCESS

    def _set_speed_state(self, speed: float, end_session: bool = True) -> float:
        """Account for the speed so far, apply the new one. Returns the previous."""
        self._tick()

        previous, self._speed = self._speed, speed

        if speed > 0 and not self._session:
            self._session = True
            self._distance = self._time = 0.0
            self._publish_session()

        elif speed == 0 and end_session:
            self._session = False

        return previous

    def _tick(self, now: float | None = None) -> None:
        """Integrate the commanded speed up to `now`."""
        now = time.monotonic() if now is None else now
        dt, self._last_tick = max(0.0, now - self._last_tick), now

        if self._speed > 0:
            self._distance += self._speed / 3.6 * dt
            self._time += dt
            self._publish_session()

    def _publish_session(self) -> None:
        values = {DISTANCE_TOTAL: int(self._distance), TIME_ELAPSED: int(self._time)}

        if changed := {k: v for k, v in values.items() if self._published.get(k) != v}:
            self._published |= changed

            if self._on_data:
                self._on_data(**changed)

            self._cb(UpdateEvent(event_id="update", event_data=changed))

    def _report(self, speed: float, previous: float, source: ControlSource) -> None:
        """Report the belt speed and start/stop transitions."""
        if self._on_data:
            self._on_data(**{SPEED_INSTANT: speed})

        self._cb(UpdateEvent(event_id="update", event_data={SPEED_INSTANT: speed}))

        if (speed > 0) != (previous > 0):
            self._cb(
                ControlEvent(
                    event_id="start" if speed > 0 else "stop",
                    event_source=source,
                )
            )

    async def start(self, cli: BleakClient) -> ResultCode:
        """Starts the belt at the minimum speed, like the vendor app does."""
        return await self.set_speed(cli, self._min_speed)

    async def stop(self, cli: BleakClient) -> ResultCode:
        """Stops the belt and ends the session."""
        return await self.set_speed(cli, 0)

    async def pause(self, cli: BleakClient) -> ResultCode:
        """Stops the belt, keeps the session counters."""
        return await self.set_speed(cli, 0, end_session=False)

    async def _ensure_ready(self, cli: BleakClient) -> None:
        if self._ready:
            return

        self._cli = cli

        await cli.start_notify(ESLINKER_NOTIFY_UUID, self._on_notify)
        await self._write(HELLO)
        await asyncio.sleep(0.3)

        for frame in HANDSHAKE:
            await self._write(frame)

        await asyncio.sleep(0.3)
        await self._write(CHALLENGE_SEED)

        self._ready = True
        self._last_tick = time.monotonic()
        self._task = asyncio.get_running_loop().create_task(self._keepalive_loop())

        _LOGGER.debug("ESLinker vendor channel ready.")

    async def _write(self, data: bytes) -> None:
        assert self._cli is not None

        _LOGGER.debug("ESLinker write: %s", data.hex(" ").upper())
        await self._cli.write_gatt_char(ESLINKER_WRITE_UUID, data, False)

    async def _keepalive_loop(self) -> None:
        while True:
            await asyncio.sleep(self._keepalive)

            if self._cli is None or not self._cli.is_connected:
                return

            try:
                async with self._lock:
                    self._tick()
                    await self._write(inclination_frame())

            except Exception as exc:  # noqa: BLE001
                _LOGGER.debug("ESLinker keepalive failed: %s", exc)
                return

    def _on_notify(self, c: BleakGATTCharacteristic, data: bytearray) -> None:
        data = bytes(data)

        _LOGGER.debug("ESLinker notify: %s", data.hex(" ").upper())

        for frame in split_frames(data):
            if reply := challenge_reply(frame):
                asyncio.get_running_loop().create_task(self._write(reply))

            elif (button := button_event(frame)) is not None:
                self._on_button(button)

            elif (code := value_event(frame, 0x03)) is not None:
                _LOGGER.warning(
                    "ESLinker fault E%02d: %s.", code, FAULTS.get(code, "unknown")
                )
                previous = self._set_speed_state(0.0)
                self._report(0.0, previous, "safety")

            elif (v := value_event(frame, 0xE0)) is not None:
                previous = self._set_speed_state(v / 10, end_session=False)
                self._report(self._speed, previous, "other")

            elif (bpm := value_event(frame, 0x02)) is not None:
                if bpm:
                    self._cb(UpdateEvent(event_id="update", event_data={HEART_RATE: bpm}))

            elif len(frame) == 4 and frame[:2] == b"\x09\x02":
                self._min_speed, self._max_speed = frame[2] / 10, frame[3] / 10

    def _on_button(self, button: int) -> None:
        """The console only reports its buttons; applying them is our job."""
        match button:
            case 2:  # BUTTON_STOP
                speed = 0.0
            case 1:  # BUTTON_START
                speed = self._min_speed if not self._speed else None
            case 3:  # BUTTON_SPEED_UP
                speed = self._speed + BUTTON_SPEED_STEP if self._speed else None
            case 4:  # BUTTON_SPEED_DOWN
                speed = self._speed - BUTTON_SPEED_STEP if self._speed else None
            case _:
                speed = None

        if speed is None or self._cli is None:
            _LOGGER.debug("ESLinker console button %d ignored.", button)
            return

        _LOGGER.debug("ESLinker console button %d: speed %.1f km/h.", button, speed)

        asyncio.get_running_loop().create_task(
            self.set_speed(self._cli, speed, source="user")
        )
