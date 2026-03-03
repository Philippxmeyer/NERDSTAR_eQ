#!/usr/bin/env python3
"""Generate catalog_data.inc from data/catalog.xml and expand catalog entries."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List
import xml.etree.ElementTree as ET

TYPE_ORDER = [
    "Planet",
    "Moon",
    "Star",
    "Cluster",
    "Double Star",
    "Galaxy",
    "Nebula",
    "Planetary Nebula",
]


SOUTHMOST_LATITUDE_DEG = 36.0
MIN_VISIBLE_DECLINATION_DEG = SOUTHMOST_LATITUDE_DEG - 90.0

NEW_ENTRY_PREFIX = "NGC "
NEW_ENTRY_START_NUMBER = 3000


@dataclass
class CatalogObject:
    name: str
    code: str
    type: str
    ra_hours: float
    dec_degrees: float
    magnitude: float

    def validate(self) -> None:
        if not self.name or not self.name.strip():
            raise ValueError("catalog object name must not be empty")
        if len(self.name) >= 255:
            raise ValueError(f"catalog object name too long: {self.name}")
        if not self.code or not self.code.strip():
            raise ValueError(f"catalog object {self.name} must include a catalog code")
        if len(self.code) >= 255:
            raise ValueError(f"catalog object code too long: {self.code}")
        if self.type not in TYPE_ORDER:
            raise ValueError(f"unsupported catalog type '{self.type}' for {self.name}")
        if not (0.0 <= self.ra_hours < 24.0):
            raise ValueError(f"RA out of range for {self.name}: {self.ra_hours}")
        if not (-90.0 <= self.dec_degrees <= 90.0):
            raise ValueError(f"Dec out of range for {self.name}: {self.dec_degrees}")
        if not (-30.0 <= self.magnitude <= 30.0):
            raise ValueError(f"magnitude out of range for {self.name}: {self.magnitude}")


COMMON_NAMES_BY_CODE = {
    "Messier 008": "Lagoon Nebula",
    "Messier 012": "Gumball Cluster",
    "Messier 014": "Globular Cluster M14",
    "Messier 016": "Eagle Nebula",
    "Messier 017": "Omega Nebula",
    "Messier 020": "Trifid Nebula",
    "Messier 027": "Dumbbell Nebula",
    "Messier 042": "Orion Nebula",
    "Messier 043": "De Mairan's Nebula",
    "Messier 057": "Ring Nebula",
    "Messier 076": "Little Dumbbell Nebula",
    "Messier 078": "Casper the Friendly Ghost",
    "Messier 097": "Owl Nebula",
    "NGC 0253": "Silver Coin Galaxy",
    "NGC 2403": "Camelopardalis Galaxy",
    "NGC 2903": "Spiral Galaxy NGC 2903",
    "NGC 7000": "North America Nebula",
}


def load_catalog(path: Path) -> List[CatalogObject]:
    tree = ET.parse(path)
    root = tree.getroot()
    if root.tag != "catalog":
        raise ValueError("expected <catalog> as root element")
    objects: List[CatalogObject] = []
    for element in root.findall("object"):
        name = element.attrib["name"].strip()
        code = element.attrib.get("code", name).strip()
        obj = CatalogObject(
            name=name,
            code=code,
            type=element.attrib["type"].strip(),
            ra_hours=float(element.attrib["ra_hours"]),
            dec_degrees=float(element.attrib["dec_degrees"]),
            magnitude=float(element.attrib["magnitude"]),
        )
        obj.validate()
        objects.append(obj)
    return objects


def _parse_ngc_number(name: str) -> int | None:
    if not name.startswith(NEW_ENTRY_PREFIX):
        return None
    suffix = name[len(NEW_ENTRY_PREFIX) :]
    if not suffix.isdigit():
        return None
    return int(suffix)


def normalize_generated_objects(objects: List[CatalogObject]) -> None:
    """Drop synthetic filler entries from previous catalog generations."""

    objects[:] = [
        obj
        for obj in objects
        if (_parse_ngc_number(obj.name) or -1) < NEW_ENTRY_START_NUMBER
    ]


def rename_catalog_only_objects(objects: List[CatalogObject]) -> None:
    for obj in objects:
        if obj.name != obj.code:
            continue
        replacement = COMMON_NAMES_BY_CODE.get(obj.code)
        if replacement:
            obj.name = replacement
            obj.validate()


def filter_visible_objects(objects: List[CatalogObject]) -> List[CatalogObject]:
    return [obj for obj in objects if obj.dec_degrees >= MIN_VISIBLE_DECLINATION_DEG]


def sort_catalog(objects: Iterable[CatalogObject]) -> List[CatalogObject]:
    return sorted(objects, key=lambda o: (TYPE_ORDER.index(o.type), o.name.lower()))


def write_xml(path: Path, objects: Iterable[CatalogObject]) -> None:
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', "<catalog>"]
    for obj in objects:
        lines.append(
            f"  <object name=\"{obj.name}\" code=\"{obj.code}\" type=\"{obj.type}\" ra_hours=\"{obj.ra_hours:.4f}\" "
            f"dec_degrees=\"{obj.dec_degrees:.4f}\" magnitude=\"{obj.magnitude:.1f}\"/>"
        )
    lines.append("</catalog>")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _char_literal(byte: int) -> str:
    ch = chr(byte)
    if ch == "\\":
        return "'\\\\'"
    if ch == "'":
        return "'\\\''"
    if 32 <= byte <= 126:
        return f"'{ch}'"
    return f"'\\x{byte:02x}'"


def write_inc(path: Path, objects: Iterable[CatalogObject]) -> None:
    entries = list(objects)
    entry_lines = ["// Generated from data/catalog.xml", "static constexpr storage::CatalogEntry kCatalogEntries[] = {"]
    strings: List[int] = []

    def append_string(value: str) -> tuple[int, int]:
        data = value.encode("ascii")
        offset = len(strings)
        strings.extend(data)
        return offset, len(data)

    for obj in entries:
        name_offset, name_length = append_string(obj.name)
        code_offset, code_length = append_string(obj.code)
        type_index = TYPE_ORDER.index(obj.type)
        ra_times1000 = int(round(obj.ra_hours * 1000.0))
        dec_times100 = int(round(obj.dec_degrees * 100.0))
        magnitude_times10 = int(round(obj.magnitude * 10.0))
        entry_lines.append(
            f"    {{{name_offset}, {name_length}, {code_offset}, {code_length}, {type_index}, {ra_times1000}, {dec_times100}, {magnitude_times10}}},"
        )
    entry_lines.append("};")
    entry_lines.append("static constexpr char kCatalogStrings[] = {")
    name_line = "    "
    for index, byte in enumerate(strings):
        literal = _char_literal(byte)
        name_line += literal + ", "
        if (index % 12 == 11):
            entry_lines.append(name_line.rstrip())
            name_line = "    "
    if name_line.strip():
        entry_lines.append(name_line.rstrip())
    entry_lines.append("};")
    entry_lines.append("static constexpr size_t kCatalogStringTableSize = sizeof(kCatalogStrings);")
    entry_lines.append("static constexpr size_t kCatalogEntryCount = sizeof(kCatalogEntries) / sizeof(kCatalogEntries[0]);")
    entry_lines.append("")
    path.write_text("\n".join(entry_lines), encoding="utf-8")


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    xml_path = repo_root / "data" / "catalog.xml"
    inc_path = repo_root / "catalog_data.inc"

    objects = load_catalog(xml_path)
    normalize_generated_objects(objects)
    rename_catalog_only_objects(objects)
    visible_objects = filter_visible_objects(objects)
    sorted_objects = sort_catalog(visible_objects)
    write_xml(xml_path, sorted_objects)
    write_inc(inc_path, sorted_objects)


if __name__ == "__main__":
    main()
