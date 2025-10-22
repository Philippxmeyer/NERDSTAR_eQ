#!/usr/bin/env python3
"""Analyze HID debug logs for reset and RPC errors.

This script parses the debug summary emitted by the HID firmware's
RTC-backed instrumentation. It groups related lines into boot summaries and
highlights suspicious reset reasons or failed RPC attempts so regressions can
be spotted quickly when processing serial captures.
"""

from __future__ import annotations

import argparse
import io
import re
import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable, List, Optional


RESET_REASON_SEVERITY = {
    "Unknown": "error",
    "Power-on": "ok",
    "External": "info",
    "Software": "info",
    "Panic": "error",
    "Interrupt WDT": "error",
    "Task WDT": "error",
    "Other WDT": "error",
    "Deep sleep": "info",
    "Brownout": "error",
    "SDIO": "warning",
    "USB": "warning",
    "JTAG": "info",
    "Reserved": "warning",
}


class Severity(Enum):
    OK = "ok"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"

    @classmethod
    def from_label(cls, label: str) -> "Severity":
        try:
            return cls(label)
        except ValueError:
            return cls.WARNING

    def upgrade(self, other: "Severity") -> "Severity":
        order = [self.OK, self.INFO, self.WARNING, self.ERROR]
        return max(self, other, key=lambda level: order.index(level))


@dataclass
class DebugRecord:
    boot_count: Optional[int] = None
    reset_reason: Optional[str] = None
    reset_code: Optional[int] = None
    last_loop_ms: Optional[int] = None
    last_event: Optional[str] = None
    last_event_ms: Optional[int] = None
    last_comm_command: Optional[str] = None
    last_comm_attempt_ms: Optional[int] = None
    last_comm_success_ms: Optional[int] = None
    last_comm_failure_ms: Optional[int] = None
    last_comm_error: Optional[str] = None
    issues: List[str] = field(default_factory=list)
    severity: Severity = Severity.OK

    def finalize(self) -> None:
        if self.reset_reason:
            severity = Severity.from_label(
                RESET_REASON_SEVERITY.get(self.reset_reason, "warning")
            )
            if severity != Severity.OK:
                self.add_issue(
                    f"Reset reason {self.reset_reason} ({self.reset_code})",
                    severity,
                )
        if self.last_comm_error:
            msg = self.last_comm_error.strip()
            if msg and msg != "<none>":
                self.add_issue(
                    f"Last RPC '{self.last_comm_command or '?'}' failed: {msg}",
                    Severity.ERROR,
                )
        if (
            self.last_comm_failure_ms
            and not self.last_comm_success_ms
            and not self.last_comm_error
        ):
            self.add_issue(
                f"Last RPC '{self.last_comm_command or '?'}' failed without error text",
                Severity.WARNING,
            )

    def add_issue(self, message: str, severity: Severity) -> None:
        self.issues.append(message)
        self.severity = self.severity.upgrade(severity)

    def describe(self) -> str:
        lines = []
        header = f"Boot {self.boot_count if self.boot_count is not None else '?'}"
        if self.reset_reason:
            header += f" – reset: {self.reset_reason}"
            if self.reset_code is not None:
                header += f" ({self.reset_code})"
        header += f" – status: {self.severity.value.upper()}"
        lines.append(header)
        if self.last_event:
            lines.append(
                f"  Last event @ {self.last_event_ms} ms: {self.last_event}"
            )
        if self.last_loop_ms is not None:
            lines.append(f"  Last loop heartbeat: {self.last_loop_ms} ms")
        if self.last_comm_command:
            lines.append(f"  Last RPC command: {self.last_comm_command}")
            if self.last_comm_attempt_ms is not None:
                lines.append(
                    f"    Attempt at {self.last_comm_attempt_ms} ms"
                )
            if self.last_comm_success_ms is not None:
                lines.append(
                    f"    Success at {self.last_comm_success_ms} ms"
                )
            if self.last_comm_failure_ms is not None:
                lines.append(
                    f"    Failure at {self.last_comm_failure_ms} ms"
                )
            if self.last_comm_error:
                lines.append(f"    Error: {self.last_comm_error}")
        for issue in self.issues:
            lines.append(f"  !! {issue}")
        return "\n".join(lines)


BOOT_COUNT_RE = re.compile(r"\[DEBUG\] Boot count: (\d+)")
RESET_REASON_RE = re.compile(
    r"\[DEBUG\] Reset reason: (?P<reason>.+?) \((?P<code>\d+)\)"
)
LAST_LOOP_RE = re.compile(r"\[DEBUG\] Last loop heartbeat at t=(\d+)ms")
LAST_EVENT_RE = re.compile(
    r"\[DEBUG\] Last event: (?P<label>.+?) \(t=(?P<ts>\d+)ms\)"
)
LAST_RPC_RE = re.compile(r"\[DEBUG\] Last RPC command: (?P<cmd>.+)")
RPC_ATTEMPT_RE = re.compile(r"\[DEBUG\]\s+Last attempt at t=(\d+)ms")
RPC_SUCCESS_RE = re.compile(r"\[DEBUG\]\s+Last success at t=(\d+)ms")
RPC_FAILURE_RE = re.compile(
    r"\[DEBUG\]\s+Last failure at t=(\d+)ms, error=(?P<error>.*)"
)


def parse_records(lines: Iterable[str]) -> List[DebugRecord]:
    current: Optional[DebugRecord] = None
    records: List[DebugRecord] = []

    for raw_line in lines:
        line = raw_line.strip()
        if not line:
            continue

        boot_match = BOOT_COUNT_RE.search(line)
        if boot_match:
            if current:
                current.finalize()
                records.append(current)
            current = DebugRecord(boot_count=int(boot_match.group(1)))
            continue

        if current is None:
            # Skip lines until we encounter a boot header.
            continue

        if reset := RESET_REASON_RE.search(line):
            current.reset_reason = reset.group("reason")
            current.reset_code = int(reset.group("code"))
            continue

        if loop_match := LAST_LOOP_RE.search(line):
            current.last_loop_ms = int(loop_match.group(1))
            continue

        if event_match := LAST_EVENT_RE.search(line):
            current.last_event = event_match.group("label")
            current.last_event_ms = int(event_match.group("ts"))
            continue

        if rpc_match := LAST_RPC_RE.search(line):
            current.last_comm_command = rpc_match.group("cmd")
            continue

        if attempt_match := RPC_ATTEMPT_RE.search(line):
            current.last_comm_attempt_ms = int(attempt_match.group(1))
            continue

        if success_match := RPC_SUCCESS_RE.search(line):
            current.last_comm_success_ms = int(success_match.group(1))
            continue

        if failure_match := RPC_FAILURE_RE.search(line):
            current.last_comm_failure_ms = int(failure_match.group(1))
            current.last_comm_error = failure_match.group("error").strip()
            continue

    if current:
        current.finalize()
        records.append(current)

    return records


def analyze_stream(stream: io.TextIOBase) -> List[DebugRecord]:
    return parse_records(stream.readlines())


def summarize(records: List[DebugRecord]) -> int:
    highest = Severity.OK
    for record in records:
        print(record.describe())
        print()
        highest = highest.upgrade(record.severity)
    if not records:
        print("No debug records found.")
        return 1
    if highest is Severity.ERROR:
        return 2
    if highest is Severity.WARNING:
        return 1
    return 0


def create_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze HID debug logs for watchdog resets or RPC failures and "
            "summarize potential issues."
        )
    )
    parser.add_argument(
        "input",
        nargs="?",
        default="-",
        help="Log file to parse (default: stdin)",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = create_arg_parser()
    args = parser.parse_args(argv)

    if args.input == "-":
        stream = sys.stdin
    else:
        try:
            stream = open(args.input, "r", encoding="utf-8")
        except OSError as exc:
            parser.error(f"Failed to open {args.input}: {exc}")

    try:
        records = analyze_stream(stream)
    finally:
        if stream is not sys.stdin:
            stream.close()

    return summarize(records)


if __name__ == "__main__":
    raise SystemExit(main())
