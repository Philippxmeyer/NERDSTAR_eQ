"""ASTAP plate-solve wrapper.

Returns the solved (ra_hours, dec_degrees) on success, or None on failure.
"""

import asyncio
import logging
import math
from typing import Optional

import astropy.io.fits as fits

import config

logger = logging.getLogger(__name__)

# Field-of-view derived from sensor / optics geometry
_FOV_DEG = (
    config.SENSOR_WIDTH_PX * config.PIXEL_SIZE_UM * 206_265.0
) / (config.FOCAL_LENGTH_MM * 1_000.0 * 3_600.0)


async def solve(
    fits_path: str,
    hint_ra: float,
    hint_dec: float,
    radius_deg: float = 10.0,
) -> Optional[tuple[float, float]]:
    """Run ASTAP on *fits_path* and return (ra_hours, dec_degrees) or None.

    ASTAP writes WCS keywords back into the FITS file on success.
    Timeout is 30 s.
    """
    spd = hint_dec + 90.0  # south polar distance

    cmd = [
        config.ASTAP_PATH,
        "-f", fits_path,
        "-ra", f"{hint_ra:.6f}",
        "-spd", f"{spd:.6f}",
        "-r", f"{radius_deg:.1f}",
        "-fov", f"{_FOV_DEG:.3f}",
        "-update",
    ]

    logger.info("Plate-solving %s (hint RA=%.4fh Dec=%.2f°)", fits_path, hint_ra, hint_dec)

    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=30.0)
        except asyncio.TimeoutError:
            proc.kill()
            logger.warning("Plate-solve timed out after 30 s")
            return None

        if proc.returncode != 0:
            logger.warning(
                "ASTAP exited %d: %s", proc.returncode, stderr.decode(errors="replace")
            )
            return None

        with fits.open(fits_path) as hdul:
            header = hdul[0].header
            if "CRVAL1" not in header or "CRVAL2" not in header:
                logger.warning("ASTAP succeeded but no WCS in output")
                return None
            ra_deg = float(header["CRVAL1"])
            dec_deg = float(header["CRVAL2"])

        ra_hours = ra_deg / 15.0
        logger.info("Plate-solve result: RA=%.4fh Dec=%.2f°", ra_hours, dec_deg)
        return ra_hours, dec_deg

    except Exception as exc:
        logger.exception("Plate-solve error: %s", exc)
        return None
