"""Object catalog: XML parsing, full-text search, and position lookup.

Planet positions are calculated on the Pi using astropy; all other objects
return their stored (fixed) coordinates directly.
"""

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

logger = logging.getLogger(__name__)

_catalog: list["CatalogObject"] = []


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


def search(query: str, max_results: int = 20) -> list[CatalogObject]:
    """Full-text search on name and code fields (case-insensitive)."""
    q = query.lower().strip()
    if not q:
        return _catalog[:max_results]
    results: list[CatalogObject] = []
    for obj in _catalog:
        if q in obj.name.lower() or q in obj.code.lower():
            results.append(obj)
            if len(results) >= max_results:
                break
    return results


def find_by_code(code: str) -> Optional[CatalogObject]:
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

    For planets, compute current geocentric position via astropy.
    For all other types, return stored coordinates.
    """
    if obj.type.lower() == "planet":
        try:
            from astropy.coordinates import EarthLocation, get_body
            from astropy.time import Time
            import astropy.units as u

            # Ensure UTC-naive datetime for astropy
            utc_naive = utc.replace(tzinfo=None) if utc.tzinfo else utc
            t = Time(utc_naive, format="datetime", scale="utc")
            location = EarthLocation(lat=lat_deg * u.deg, lon=lon_deg * u.deg)
            body = get_body(obj.code.lower(), t, location)
            return float(body.ra.hour), float(body.dec.deg)
        except Exception as exc:
            logger.debug("Planet position failed for %s: %s", obj.code, exc)

    return obj.ra_hours, obj.dec_degrees
