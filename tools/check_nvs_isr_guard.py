#!/usr/bin/env python3
"""Lexical flash-guard coverage check, not a proof of runtime safety.

Scans every C/C++ source and header under src, including NVS implementations and
templates. Direct flash sinks and read/write NVS handle construction must have a
visible TccFlashGuard in scope. Session exceptions identify the exact OTA method
and sink and require visible ownership checking/acquisition in that method.

This does NOT prove control flow, successful acquisition, matching task ownership,
indirect calls, callback IRAM placement/quiescence, or hardware output recovery.
Those require host checks, source/ELF inspection and loaded bench qualification.
CI runs this separately from PlatformIO; `pio run` does not imply it ran.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".inc"}
SINK = re.compile(r"\b(?:nvs_set_\w+|nvs_commit|nvs_erase_\w+|nvs_flash_(?:init|erase)\w*|nvs_open(?:_from_partition)?|esp_partition_(?:write|erase_range)|esp_flash_(?:write|erase_region)|esp_ota_(?:begin|write|end|set_boot_partition)|esp_core_dump_image_erase)\s*\(")
TOKEN = re.compile(
    r"(?P<guard>\bTccFlashGuard\s+(?P<name>\w+)\s*;)"
    r"|(?P<release>\b(?P<released>\w+)\s*\.\s*release\s*\()"
    r"|(?P<rw>\bNvsHandle\s+\w+\s*[({]\s*(?!NVS_READONLY\b)[^)}]+[)}])"
    r"|(?P<sink>" + SINK.pattern + r")|(?P<brace>[{}])"
)
SESSION_SINKS = {
    ("Flasher::on_request_download", "esp_flash_erase_region"),
    ("Flasher::on_request_download", "esp_core_dump_image_erase"),
    ("Flasher::on_transfer_data", "esp_flash_write"),
    ("Flasher::on_request_verification", "esp_ota_set_boot_partition"),
}
FUNCTION = re.compile(r"\b(?:void|esp_err_t|bool)\s+(\w+::\w+)\s*\([^;{}]*\)\s*\{")


def strip_comments(text: str) -> str:
    """Keep offsets/newlines while removing comments and quoted literals."""
    return re.sub(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda m: ''.join('\n' if c == '\n' else ' ' for c in m[0]), text)


def inspect_source(source: str, rel: str) -> tuple[list[str], list[str]]:
    text = strip_comments(source)
    functions = list(FUNCTION.finditer(text))
    scopes: list[set[str]] = [set()]
    findings, listing = [], []
    for event in TOKEN.finditer(text):
        kind = event.lastgroup
        if kind == "brace":
            if event[0] == "{":
                scopes.append(set())
            elif len(scopes) > 1:
                scopes.pop()
            continue
        if event.group("guard"):
            scopes[-1].add(event.group("name"))
            continue
        if event.group("release"):
            for scope in scopes:
                scope.discard(event.group("released"))
            continue
        symbol = "NvsHandle(NVS_READWRITE)" if event.group("rw") else event[0].split("(")[0].strip()
        number = text.count('\n', 0, event.start()) + 1
        visible = any(scopes)
        state = "SCOPED" if visible else "UNGUARDED"
        # The one low-level open constructor accepts READONLY and READWRITE.
        # Its exact spelling is pinned; every RW construction is checked above.
        if symbol == "nvs_open" and rel == "nvs/eeprom_config.h":
            before = text[max(0, event.start() - 20):event.start()]
            after = text[event.start():event.end() + 150]
            if re.search(r":\s*error\($", before) and re.match(r"nvs_open\(NVS_PARTITION_USER_CFG, mode, &value\)\)\s*\{\}", after):
                state = "HANDLE_IMPL"
        if state == "UNGUARDED" and rel == "diag/flasher.cpp":
            prior = [f for f in functions if f.start() < event.start()]
            if prior:
                function = prior[-1]
                prefix = text[function.end():event.start()]
                if (function[1], symbol) in SESSION_SINKS and re.search(r"\bflash_guard\.(?:acquire|owns_lock)\s*\(", prefix):
                    state = "SESSION"
        listing.append(f"{state:12} {rel}:{number} {symbol}")
        if state == "UNGUARDED":
            findings.append(f"{rel}:{number}: {symbol} lacks visible scoped/session ownership")
    return findings, listing


def find_violations(list_all: bool = False, src: Path = SRC) -> list[str]:
    findings = []
    for path in sorted(p for p in src.rglob('*') if p.is_file() and p.suffix in SUFFIXES):
        errors, listing = inspect_source(path.read_text(), path.relative_to(src).as_posix())
        findings.extend(errors)
        if list_all:
            print('\n'.join(listing)) if listing else None
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    findings = find_violations(args.list)
    if findings:
        print('\n'.join(findings), file=sys.stderr)
        return 1
    print('OK: direct flash sinks have lexical scoped/session coverage; runtime safety is not proven.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
