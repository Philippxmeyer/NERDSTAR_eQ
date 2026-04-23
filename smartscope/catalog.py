"""Object catalog: XML parsing, full-text search, and position lookup.

Planet and Moon positions are calculated on the Pi using astropy; all other
objects return their stored (fixed) coordinates directly.

Search normalisation handles short forms used by observers:
  M42  → "messier 042"
  NGC253 → "ngc 0253"
"""

import logging
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

logger = logging.getLogger(__name__)

_catalog: list["CatalogObject"] = []

# Types whose coordinates change over time and must be computed live
_DYNAMIC_TYPES = frozenset({"planet", "moon"})


@dataclass
class CatalogObject:
    name: str
    code: str
    type: str
    ra_hours: float
    dec_degrees: float
    magnitude: float


def load_catalog(path: str) -> list[CatalogObject]:
    """Parse XML catalog and replace the module-level catalog list."""
    global _catalog
    tree = ET.parse(path)
    root = tree.getroot()
    objects: list[CatalogObject] = []
    for elem in root.findall("object"):
        try:
            objects.append(
                CatalogObject(
                    name=elem.get("name", ""),
                    code=elem.get("code", ""),
                    type=elem.get("type", ""),
                    ra_hours=float(elem.get("ra_hours", 0)),
                    dec_degrees=float(elem.get("dec_degrees", 0)),
                    magnitude=float(elem.get("magnitude", 99)),
                )
            )
        except (ValueError, TypeError) as exc:
            logger.warning("Skipping malformed catalog entry: %s", exc)
    _catalog = objects
    logger.info("Loaded %d objects from %s", len(_catalog), path)
    return _catalog


def catalog_size() -> int:
    return len(_catalog)


def _search_tokens(q: str) -> list[str]:
    """Expand a raw query into all equivalent search strings.

    Examples:
      "m42"    → ["m42", "messier 042"]
      "ngc253" → ["ngc253", "ngc 0253"]
      "orion"  → ["orion"]
    """
    tokens = [q]
    # M<n> → "messier NNN"
    m = re.match(r"^m\s*(\d+)$", q)
    if m:
        tokens.append(f"messier {int(m.group(1)):03d}")
    # NGC<n> → "ngc NNNN"
    m = re.match(r"^ngc\s*(\d+)$", q)
    if m:
        tokens.append(f"ngc {int(m.group(1)):04d}")
    return tokens


def search(query: str, max_results: int = 20) -> list[CatalogObject]:
    """Full-text search on name and code fields (case-insensitive).

    Recognises short observer notation: M42, NGC253, etc.
    """
    q = query.lower().strip()
    if not q:
        return _catalog[:max_results]

    tokens = _search_tokens(q)
    results: list[CatalogObject] = []
    for obj in _catalog:
        name_l = obj.name.lower()
        code_l = obj.code.lower()
        if any(t in name_l or t in code_l for t in tokens):
            results.append(obj)
            if len(results) >= max_results:
                break
    return results


def find_by_code(code: str) -> Optional[CatalogObject]:
    """Exact (case-insensitive) code lookup."""
    code_lower = code.lower()
    for obj in _catalog:
        if obj.code.lower() == code_lower:
            return obj
    return None


def get_position(
    obj: CatalogObject,
    utc: datetime,
    lat_deg: float,
    lon_deg: float,
) -> tuple[float, float]:
    """Return (ra_hours, dec_degrees) for the object at the given UTC time.

    For Planet and Moon types, compute current geocentric position via astropy.
    For all other types, return stored catalog coordinates.
    """
    if obj.type.lower() in _DYNAMIC_TYPES:
        try:
            from astropy.coordinates import EarthLocation, get_body
            from astropy.time import Time
            import astropy.units as u

            # astropy Time expects a UTC-naive datetime when scale='utc'
            utc_naive = utc.replace(tzinfo=None) if utc.tzinfo else utc
            t = Time(utc_naive, format="datetime", scale="utc")
            location = EarthLocation(lat=lat_deg * u.deg, lon=lon_deg * u.deg)
            # "Earth Moon" → "moon"; "Jupiter" → "jupiter"
            body_name = obj.code.lower().replace("earth ", "")
            body = get_body(body_name, t, location)
            return float(body.ra.hour), float(body.dec.deg)
        except Exception as exc:
            logger.debug("Live position failed for %s: %s", obj.code, exc)

    return obj.ra_hours, obj.dec_degrees
