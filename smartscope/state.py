import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta


@dataclass
class ScopeState:
    utc_time_ref:    datetime | None = None
    monotonic_ref:   float = 0.0
    latitude_deg:    float = 0.0
    longitude_deg:   float = 0.0
    location_set:    bool  = False
    ra_current:      float = 0.0   # decimal hours
    dec_current:     float = 0.0   # decimal degrees
    ra_offset:       float = 0.0   # arcseconds
    dec_offset:      float = 0.0   # arcseconds
    tracking:        bool  = False
    slewing:         bool  = False
    esp32_connected: bool  = False


def get_current_utc(state: ScopeState) -> datetime:
    if state.utc_time_ref is None:
        return datetime.utcnow()
    elapsed = time.monotonic() - state.monotonic_ref
    return state.utc_time_ref + timedelta(seconds=elapsed)
