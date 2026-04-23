"""Picamera2 wrapper.

All Picamera2 calls run inside a single-threaded ThreadPoolExecutor so they
never block the asyncio event loop and are never called concurrently.
"""

import asyncio
import logging
import os
from concurrent.futures import ThreadPoolExecutor
from datetime import timezone
from typing import Optional

import astropy.io.fits as fits
import numpy as np
from picamera2 import Picamera2

import config
import state as state_module

logger = logging.getLogger(__name__)

_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="camera")
_camera: Optional[Picamera2] = None


def _get_camera() -> Picamera2:
    """Return the singleton Picamera2 instance, starting it if necessary."""
    global _camera
    if _camera is None:
        _camera = Picamera2()
        cfg = _camera.create_still_configuration(
            main={
                "size": (config.SENSOR_WIDTH_PX, config.SENSOR_HEIGHT_PX),
                "format": "RGB888",
            }
        )
        _camera.configure(cfg)
        _camera.start()
        logger.info("Camera started at %dx%d", config.SENSOR_WIDTH_PX, config.SENSOR_HEIGHT_PX)
    return _camera


def _do_capture(exposure_s: float, gain: float) -> np.ndarray:
    """Blocking capture; must be called from the executor thread only."""
    import time as _time
    cam = _get_camera()
    cam.set_controls(
        {
            "ExposureTime": int(exposure_s * 1_000_000),
            "AnalogueGain": float(gain),
            "AeEnable": False,
            "AwbEnable": False,
        }
    )
    # Wait for the control changes to propagate through at least two frames
    _time.sleep(exposure_s + 0.3)
    return cam.capture_array()


async def capture_frame(exposure_s: float, gain: float) -> np.ndarray:
    """Capture a frame asynchronously; returns an HxWx3 uint8 numpy array."""
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(_executor, _do_capture, exposure_s, gain)


def write_fits(
    array: np.ndarray,
    path: str,
    state: state_module.ScopeState,
    exposure_s: float,
    gain: float,
) -> None:
    """Write array as a FITS file with observation metadata."""
    # Convert to grayscale float32 for FITS
    if array.ndim == 3:
        data = np.mean(array, axis=2).astype(np.float32)
    else:
        data = array.astype(np.float32)

    hdu = fits.PrimaryHDU(data)
    hdr = hdu.header
    utc = state_module.get_current_utc(state)
    hdr["DATE-OBS"] = utc.replace(tzinfo=timezone.utc).isoformat()
    hdr["EXPTIME"] = (exposure_s, "Exposure time [s]")
    hdr["GAIN"] = (gain, "Analogue gain")
    hdr["RA"] = (state.ra_current, "RA decimal hours")
    hdr["DEC"] = (state.dec_current, "Dec decimal degrees")
    hdr["FOCALLEN"] = (config.FOCAL_LENGTH_MM, "Focal length [mm]")
    hdr["XPIXSZ"] = (config.PIXEL_SIZE_UM, "Pixel size [um]")

    os.makedirs(os.path.dirname(path), exist_ok=True)
    hdu.writeto(path, overwrite=True)
