"""Pointing model and coordinate corrections.

Phase 1: simple additive offsets derived from the most recent plate solve.

The module is structured so that a Taki / Konradi (or other) model can be
plugged in later by replacing the body of apply_offsets() and adding
calibration-star storage here.
"""

from datetime import datetime

import state as state_module


def sidereal_time(utc: datetime, longitude_deg: float) -> float:
    """Return Local Apparent Sidereal Time (LAST) in decimal hours."""
    from astropy.coordinates import Longitude
    from astropy.time import Time
    import astropy.units as u

    t = Time(utc.replace(tzinfo=None) if utc.tzinfo else utc, format="datetime", scale="utc")
    lst = t.sidereal_time("apparent", longitude=Longitude(longitude_deg * u.deg))
    return float(lst.hour)


def apply_offsets(
    ra_hours: float,
    dec_degrees: float,
    state: state_module.ScopeState,
) -> tuple[float, float]:
    """Apply pointing corrections and return corrected (ra_hours, dec_degrees).

    Phase 1: additive offsets only.
    state.ra_offset  is in arcseconds of RA angle → convert to hours.
    state.dec_offset is in arcseconds → convert to degrees.
    """
    # Future Taki/Konradi model: replace these two lines with a full
    # affine / spherical transform using stored sync points.
    ra_corrected = ra_hours + state.ra_offset / (15.0 * 3600.0)
    dec_corrected = dec_degrees + state.dec_offset / 3600.0
    return ra_corrected, dec_corrected
