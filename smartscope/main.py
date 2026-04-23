"""SmartScope – FastAPI application entry point.

Startup tasks:
  • lx200 serial worker
  • position_loop  – polls :GR# / :GD# every 2 s
  • capture_loop   – continuous capture when tracking is enabled

Run:  python3 main.py   (or via systemd unit)
"""

import asyncio
import base64
import logging
import os
import time as _time
from datetime import datetime, timezone
from typing import Optional

import numpy as np
import uvicorn
from fastapi import FastAPI, Request, UploadFile
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel
from sse_starlette.sse import EventSourceResponse

import camera
import catalog
import config
import lx200
import platesolve
import pointing
import stack as stack_module
import state as state_module

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)

app = FastAPI(title="SmartScope")

# ---------------------------------------------------------------------------
# Shared application state
# ---------------------------------------------------------------------------
scope_state = state_module.ScopeState()
stacker = stack_module.ImageStack()
_capture_params: dict = {"exposure": 2.0, "gain": 1.0, "frames": 100}
_capture_enabled: bool = False
_frames_captured: int = 0
_last_solve_fits: Optional[str] = None


# ---------------------------------------------------------------------------
# Background tasks
# ---------------------------------------------------------------------------

async def position_loop() -> None:
    """Poll mount position every 2 s and update scope_state."""
    while True:
        try:
            await lx200.get_position(scope_state)
        except Exception as exc:
            logger.debug("Position poll error: %s", exc)
        await asyncio.sleep(2.0)


async def capture_loop() -> None:
    """Capture frames continuously while tracking is enabled."""
    global _capture_enabled, _frames_captured, _last_solve_fits
    while True:
        if scope_state.tracking and _capture_enabled:
            params = _capture_params.copy()
            try:
                array = await camera.capture_frame(params["exposure"], params["gain"])

                utc = state_module.get_current_utc(scope_state)
                fname = utc.strftime("%Y%m%d_%H%M%S_%f") + ".fits"
                fits_path = os.path.join(config.STORAGE_PATH, fname)
                camera.write_fits(
                    array, fits_path, scope_state, params["exposure"], params["gain"]
                )
                _last_solve_fits = fits_path

                stacker.add_frame(array)
                _frames_captured += 1

                if 0 < params["frames"] <= _frames_captured:
                    _capture_enabled = False
                    logger.info("Capture sequence of %d frames complete", params["frames"])

            except Exception as exc:
                logger.exception("Capture error: %s", exc)
                await asyncio.sleep(1.0)
        else:
            await asyncio.sleep(0.1)


# ---------------------------------------------------------------------------
# Startup
# ---------------------------------------------------------------------------

@app.on_event("startup")
async def startup() -> None:
    await lx200.start_worker(scope_state)
    asyncio.create_task(position_loop())
    asyncio.create_task(capture_loop())

    if os.path.exists(config.CATALOG_PATH):
        try:
            catalog.load_catalog(config.CATALOG_PATH)
        except Exception as exc:
            logger.warning("Catalog load failed: %s", exc)


# ---------------------------------------------------------------------------
# Static / index
# ---------------------------------------------------------------------------

@app.get("/", response_class=HTMLResponse)
async def index() -> HTMLResponse:
    html_path = os.path.join(os.path.dirname(__file__), "static", "index.html")
    with open(html_path, encoding="utf-8") as fh:
        return HTMLResponse(fh.read())


# ---------------------------------------------------------------------------
# SSE preview stream
# ---------------------------------------------------------------------------

@app.get("/preview")
async def preview_stream(request: Request):
    async def generator():
        while True:
            if await request.is_disconnected():
                break
            jpeg = stacker.get_preview_jpeg()
            yield {"data": base64.b64encode(jpeg).decode("ascii")}
            await asyncio.sleep(2.0)

    return EventSourceResponse(generator())


# ---------------------------------------------------------------------------
# Status
# ---------------------------------------------------------------------------

@app.get("/status")
async def status():
    utc = state_module.get_current_utc(scope_state)
    return {
        "ra": scope_state.ra_current,
        "dec": scope_state.dec_current,
        "tracking": scope_state.tracking,
        "slewing": scope_state.slewing,
        "stack_count": stacker.frame_count,
        "utc": utc.isoformat() if scope_state.utc_time_ref is not None else None,
        "location_set": scope_state.location_set,
        "esp32_connected": scope_state.esp32_connected,
        "catalog_loaded": catalog.catalog_size() > 0,
        "capture_active": _capture_enabled,
        "frames_captured": _frames_captured,
    }


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

class InitRequest(BaseModel):
    utc: str        # ISO-8601, e.g. "2025-04-23T21:34:00Z"
    lat: float
    lon: float


@app.post("/init")
async def init(req: InitRequest):
    utc_dt = datetime.fromisoformat(req.utc.replace("Z", "+00:00")).replace(tzinfo=timezone.utc)
    scope_state.utc_time_ref = utc_dt
    scope_state.monotonic_ref = _time.monotonic()
    scope_state.latitude_deg = req.lat
    scope_state.longitude_deg = req.lon
    scope_state.location_set = True

    try:
        await lx200.sync_time_location(scope_state)
    except Exception as exc:
        logger.warning("sync_time_location failed: %s", exc)

    return {"ok": True}


# ---------------------------------------------------------------------------
# GoTo
# ---------------------------------------------------------------------------

class RaDecRequest(BaseModel):
    ra_hours: float
    dec_degrees: float


@app.post("/goto/radec")
async def goto_radec(req: RaDecRequest):
    ra, dec = pointing.apply_offsets(req.ra_hours, req.dec_degrees, scope_state)
    result = await lx200.goto(ra, dec)
    asyncio.create_task(lx200.wait_for_slew(scope_state))
    return {"ok": True, "mount_reply": result}


class ObjectRequest(BaseModel):
    code: str


@app.post("/goto/object")
async def goto_object(req: ObjectRequest):
    obj = catalog.find_by_code(req.code)
    if obj is None:
        return JSONResponse({"error": "Object not found"}, status_code=404)

    utc = state_module.get_current_utc(scope_state)
    ra, dec = catalog.get_position(
        obj, utc, scope_state.latitude_deg, scope_state.longitude_deg
    )
    ra_corr, dec_corr = pointing.apply_offsets(ra, dec, scope_state)
    result = await lx200.goto(ra_corr, dec_corr)
    asyncio.create_task(lx200.wait_for_slew(scope_state))
    return {"ok": True, "mount_reply": result, "ra_hours": ra, "dec_degrees": dec}


# ---------------------------------------------------------------------------
# Search
# ---------------------------------------------------------------------------

@app.get("/search")
async def search(q: str = ""):
    results = catalog.search(q, max_results=20)
    utc = state_module.get_current_utc(scope_state)
    output = []
    for obj in results:
        ra, dec = catalog.get_position(
            obj, utc, scope_state.latitude_deg, scope_state.longitude_deg
        )
        output.append(
            {
                "name": obj.name,
                "code": obj.code,
                "type": obj.type,
                "magnitude": obj.magnitude,
                "ra_hours": ra,
                "dec_degrees": dec,
            }
        )
    return output


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------

class CaptureRequest(BaseModel):
    exposure: float = 2.0
    gain: float = 1.0
    frames: int = 100


@app.post("/capture")
async def capture(req: CaptureRequest):
    global _capture_params, _capture_enabled, _frames_captured
    _capture_params = {"exposure": req.exposure, "gain": req.gain, "frames": req.frames}
    _frames_captured = 0
    _capture_enabled = True
    return {"ok": True}


# ---------------------------------------------------------------------------
# Plate solve
# ---------------------------------------------------------------------------

@app.post("/solve")
async def solve():
    global _last_solve_fits

    # Capture a dedicated solve frame
    params = _capture_params.copy()
    try:
        array = await camera.capture_frame(params["exposure"], params["gain"])
    except Exception as exc:
        return JSONResponse({"error": f"Capture failed: {exc}"}, status_code=500)

    utc = state_module.get_current_utc(scope_state)
    fname = "solve_" + utc.strftime("%Y%m%d_%H%M%S") + ".fits"
    fits_path = os.path.join(config.STORAGE_PATH, fname)
    camera.write_fits(array, fits_path, scope_state, params["exposure"], params["gain"])
    _last_solve_fits = fits_path

    result = await platesolve.solve(
        fits_path,
        scope_state.ra_current,
        scope_state.dec_current,
    )

    if result is None:
        return JSONResponse({"error": "Plate solve failed"}, status_code=500)

    ra_solved, dec_solved = result

    # Update pointing offsets (arcseconds)
    scope_state.ra_offset = (ra_solved - scope_state.ra_current) * 15.0 * 3600.0
    scope_state.dec_offset = (dec_solved - scope_state.dec_current) * 3600.0

    # Keep ESP32 time/location in sync
    try:
        await lx200.sync_time_location(scope_state)
    except Exception as exc:
        logger.warning("sync after solve failed: %s", exc)

    return {
        "ok": True,
        "ra_hours": ra_solved,
        "dec_degrees": dec_solved,
        "ra_offset_arcsec": scope_state.ra_offset,
        "dec_offset_arcsec": scope_state.dec_offset,
    }


# ---------------------------------------------------------------------------
# Tracking
# ---------------------------------------------------------------------------

class TrackRequest(BaseModel):
    enabled: bool


@app.post("/track")
async def track(req: TrackRequest):
    scope_state.tracking = req.enabled
    return {"ok": True, "tracking": scope_state.tracking}


# ---------------------------------------------------------------------------
# Manual slew
# ---------------------------------------------------------------------------

class SlewRequest(BaseModel):
    direction: str   # "n" | "s" | "e" | "w"
    active: bool


@app.post("/slew")
async def slew(req: SlewRequest):
    if req.active:
        await lx200.start_slew(req.direction)
    else:
        await lx200.stop_slew(req.direction)
    return {"ok": True}


class SlewRateRequest(BaseModel):
    rate: str   # "guide" | "center" | "find" | "max"


@app.post("/slew/rate")
async def slew_rate(req: SlewRateRequest):
    await lx200.set_slew_rate(req.rate)
    return {"ok": True}


# ---------------------------------------------------------------------------
# Stack
# ---------------------------------------------------------------------------

@app.post("/stack/reset")
async def stack_reset():
    stacker.reset()
    return {"ok": True}


# ---------------------------------------------------------------------------
# Emergency stop / park
# ---------------------------------------------------------------------------

@app.post("/stop")
async def stop():
    await lx200.stop_all()
    scope_state.slewing = False
    return {"ok": True}


@app.post("/park")
async def park():
    await lx200.park()
    return {"ok": True}


# ---------------------------------------------------------------------------
# Catalog upload
# ---------------------------------------------------------------------------

@app.post("/catalog/upload")
async def catalog_upload(file: UploadFile):
    content = await file.read()
    os.makedirs(os.path.dirname(config.CATALOG_PATH), exist_ok=True)
    with open(config.CATALOG_PATH, "wb") as fh:
        fh.write(content)
    try:
        catalog.load_catalog(config.CATALOG_PATH)
    except Exception as exc:
        return JSONResponse({"error": f"Parse failed: {exc}"}, status_code=400)
    return {"ok": True, "count": catalog.catalog_size()}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        log_level="info",
    )
