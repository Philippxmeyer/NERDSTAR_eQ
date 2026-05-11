"""LX200 serial communication with the NERDSTAR eQ mount.

Single asyncio.Queue serialises all outgoing commands; a single worker
coroutine drives the blocking serial I/O in a ThreadPoolExecutor thread.
"""

import asyncio
import logging
import time as _time
from typing import Optional

import serial

import config
import state as state_module

logger = logging.getLogger(__name__)

# Commands the firmware acknowledges silently (no reply frame)
_NO_REPLY_CMDS: frozenset[str] = frozenset({
    ":RG", ":RC", ":RM", ":RS",
    ":TQ", ":TS", ":TL", ":T+", ":T-", ":U",
})

_serial: Optional[serial.Serial] = None
_cmd_queue: Optional[asyncio.Queue] = None
_state_ref: Optional[state_module.ScopeState] = None


# ---------------------------------------------------------------------------
# Format converters
# ---------------------------------------------------------------------------

def ra_to_lx200(ra_hours: float) -> str:
    """Decimal RA hours → LX200 HH:MM:SS."""
    total_sec = int(round(ra_hours * 3600)) % (24 * 3600)
    h = total_sec // 3600
    m = (total_sec % 3600) // 60
    s = total_sec % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def dec_to_lx200(dec_degrees: float) -> str:
    """Decimal Dec degrees → LX200 +/-DD*MM:SS."""
    dec_degrees = max(-90.0, min(90.0, dec_degrees))
    sign = "+" if dec_degrees >= 0 else "-"
    total_arcsec = int(round(abs(dec_degrees) * 3600))
    d = total_arcsec // 3600
    m = (total_arcsec % 3600) // 60
    s = total_arcsec % 60
    return f"{sign}{d:02d}*{m:02d}:{s:02d}"


def lx200_to_ra(s: str) -> float:
    """LX200 HH:MM:SS → decimal RA hours."""
    s = s.strip()
    parts = s.split(":")
    h, m, sec = int(parts[0]), int(parts[1]), int(parts[2])
    return h + m / 60.0 + sec / 3600.0


def lx200_to_dec(s: str) -> float:
    """LX200 +/-DD*MM:SS → decimal Dec degrees."""
    s = s.strip()
    sign = -1.0 if s.startswith("-") else 1.0
    s = s.lstrip("+-")
    sep = "*" if "*" in s else ":"
    d_part, rest = s.split(sep, 1)
    m_part, sec_part = rest.split(":")
    return sign * (int(d_part) + int(m_part) / 60.0 + int(sec_part) / 3600.0)


def _format_lat(lat_deg: float) -> str:
    """North-positive latitude → +/-DD*MM:SS for :Sts."""
    sign = "+" if lat_deg >= 0 else "-"
    total_arcsec = int(round(abs(lat_deg) * 3600))
    d = total_arcsec // 3600
    m = (total_arcsec % 3600) // 60
    s = total_arcsec % 60
    return f"{sign}{d:02d}*{m:02d}:{s:02d}"


def _format_lon_west(east_deg: float) -> str:
    """East-positive longitude → west-positive DDD*MM for :Sg (Meade convention)."""
    west = -east_deg
    if west < 0:
        west += 360.0
    if west >= 360.0:
        west -= 360.0
    total_arcmin = int(round(west * 60))
    d = total_arcmin // 60
    m = total_arcmin % 60
    return f"{d:03d}*{m:02d}"


# ---------------------------------------------------------------------------
# Blocking serial I/O (runs in ThreadPoolExecutor)
# ---------------------------------------------------------------------------

def _try_connect() -> None:
    global _serial, _state_ref
    try:
        _serial = serial.Serial(
            config.SERIAL_PORT,
            config.SERIAL_BAUD,
            timeout=3.0,
            write_timeout=3.0,
        )
        if _state_ref:
            _state_ref.esp32_connected = True
        logger.info("Connected to %s", config.SERIAL_PORT)
    except serial.SerialException as exc:
        logger.debug("Cannot open %s: %s", config.SERIAL_PORT, exc)
        _serial = None
        if _state_ref:
            _state_ref.esp32_connected = False


def _write_and_read(cmd: str, expects_reply: bool) -> Optional[str]:
    """Send command over serial; return response string or None.

    Called from a ThreadPoolExecutor thread — do not await here.
    Retries once on SerialException after attempting to reconnect.
    """
    global _serial, _state_ref

    cmd_bytes = (cmd + "#").encode("ascii")

    for attempt in range(2):
        if _serial is None or not _serial.is_open:
            _try_connect()

        if _serial is None or not _serial.is_open:
            if _state_ref:
                _state_ref.esp32_connected = False
            raise ConnectionError(f"Cannot connect to {config.SERIAL_PORT}")

        try:
            _serial.reset_input_buffer()
            _serial.write(cmd_bytes)
            _serial.flush()

            if not expects_reply:
                if _state_ref:
                    _state_ref.esp32_connected = True
                return None

            response = _serial.read_until(b"#")
            if not response.endswith(b"#"):
                raise serial.SerialTimeoutException(f"Timeout waiting for reply to {cmd!r}")

            if _state_ref:
                _state_ref.esp32_connected = True
            return response[:-1].decode("ascii", errors="replace")

        except serial.SerialException as exc:
            logger.warning("Serial error on attempt %d for %r: %s", attempt + 1, cmd, exc)
            try:
                _serial.close()
            except Exception:
                pass
            _serial = None
            if _state_ref:
                _state_ref.esp32_connected = False
            if attempt == 1:
                raise
    return None


# ---------------------------------------------------------------------------
# Asyncio worker
# ---------------------------------------------------------------------------

async def _worker() -> None:
    assert _cmd_queue is not None
    while True:
        cmd, future = await _cmd_queue.get()
        try:
            loop = asyncio.get_running_loop()
            expects_reply = future is not None
            result = await loop.run_in_executor(None, _write_and_read, cmd, expects_reply)
            if future is not None and not future.done():
                future.set_result(result or "")
        except Exception as exc:
            if future is not None and not future.done():
                future.set_exception(exc)
        finally:
            _cmd_queue.task_done()


async def start_worker(scope_state: state_module.ScopeState) -> None:
    """Initialise queue and start the serial worker task."""
    global _cmd_queue, _state_ref
    _cmd_queue = asyncio.Queue()
    _state_ref = scope_state
    asyncio.create_task(_worker())


async def _send(cmd: str) -> Optional[str]:
    """Enqueue an LX200 command and await its response (or None for no-reply)."""
    if _cmd_queue is None:
        raise RuntimeError("LX200 worker not started")
    expects_reply = cmd not in _NO_REPLY_CMDS
    loop = asyncio.get_running_loop()
    future: Optional[asyncio.Future] = loop.create_future() if expects_reply else None
    await _cmd_queue.put((cmd, future))
    if future is not None:
        return await future
    return None


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

async def get_position(state: state_module.ScopeState) -> None:
    """Poll :GR# and :GD# and update state."""
    try:
        ra_str = await _send(":GR")
        dec_str = await _send(":GD")
        if ra_str:
            state.ra_current = lx200_to_ra(ra_str)
        if dec_str:
            state.dec_current = lx200_to_dec(dec_str)
    except Exception as exc:
        logger.debug("get_position failed: %s", exc)


async def goto(ra_hours: float, dec_degrees: float) -> str:
    """Set target and issue :MS#. Returns firmware reply ('0' = slewing)."""
    ra_str = ra_to_lx200(ra_hours)
    dec_str = dec_to_lx200(dec_degrees)
    await _send(f":Sr{ra_str}")
    await _send(f":Sd{dec_str}")
    result = await _send(":MS")
    return result or ""


async def wait_for_slew(
    state: state_module.ScopeState, timeout: float = 300.0
) -> None:
    """Poll :D# until mount reports idle, then clear state.slewing."""
    state.slewing = True
    deadline = _time.monotonic() + timeout
    while _time.monotonic() < deadline:
        try:
            response = await _send(":D")
            if response is not None and response.strip() == "":
                state.slewing = False
                return
        except Exception as exc:
            logger.debug("wait_for_slew poll error: %s", exc)
        await asyncio.sleep(1.0)
    state.slewing = False


async def sync_time_location(state: state_module.ScopeState) -> None:
    """Push UTC time and observer location to the ESP32's software clock."""
    from state import get_current_utc
    utc = get_current_utc(state)

    # UTC offset = 0; we always feed the ESP32 UTC time as "local"
    await _send(":SG+00.0")
    # Date must be sent before time (:SC buffers; :SL commits both into the
    # firmware's software clock — there is no hardware RTC anymore).
    await _send(f":SC{utc.strftime('%m/%d/%y')}")
    await _send(f":SL{utc.strftime('%H:%M:%S')}")
    await _send(f":Sts{_format_lat(state.latitude_deg)}")
    await _send(f":Sg{_format_lon_west(state.longitude_deg)}")


async def sync_time_only(state: state_module.ScopeState) -> None:
    """Push only UTC date/time (no location) to refresh the ESP32 software clock."""
    from state import get_current_utc
    utc = get_current_utc(state)
    await _send(":SG+00.0")
    await _send(f":SC{utc.strftime('%m/%d/%y')}")
    await _send(f":SL{utc.strftime('%H:%M:%S')}")


async def get_sync_age_seconds() -> Optional[int]:
    """Query :GTS# — seconds since the firmware last received :SC/:SL.

    Returns None if the firmware does not respond or returns garbage.
    """
    try:
        reply = await _send(":GTS")
    except Exception as exc:
        logger.debug("get_sync_age failed: %s", exc)
        return None
    if reply is None:
        return None
    try:
        return int(reply.strip())
    except ValueError:
        return None


async def stop_all() -> None:
    """Emergency stop: :Q#."""
    await _send(":Q")


async def start_slew(direction: str) -> None:
    """:Mn# / :Ms# / :Me# / :Mw#"""
    d = direction.lower()[0]
    await _send(f":M{d}")


async def stop_slew(direction: str) -> None:
    """:Qn# / :Qs# / :Qe# / :Qw#"""
    d = direction.lower()[0]
    await _send(f":Q{d}")


async def set_slew_rate(rate: str) -> None:
    """guide→:RG# center→:RC# find→:RM# max→:RS#"""
    mapping = {"guide": ":RG", "center": ":RC", "find": ":RM", "max": ":RS"}
    cmd = mapping.get(rate.lower(), ":RC")
    await _send(cmd)


async def park() -> None:
    """Optional park move via regular GoTo target from config."""
    ra_hours = getattr(config, "PARK_RA_HOURS", None)
    dec_deg = getattr(config, "PARK_DEC_DEG", None)
    if ra_hours is None or dec_deg is None:
        logger.info("Park skipped (no PARK_RA_HOURS/PARK_DEC_DEG configured)")
        return
    await goto(float(ra_hours), float(dec_deg))
