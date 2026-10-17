#!/usr/bin/env python3
"""Every flash write must happen with the TCC solenoid's timer ISR disabled.

Writing NVS on an ESP32 disables the flash cache. The inrush-solenoid gptimer
ISR has been observed to crash the chip during that window -- see 687dfbc
("Fix crashing when flashing and saving maps in P") and 2409592 ("Fix TCC ISR
causing crashes in save to NVS code"). `CONFIG_GPTIMER_ISR_IRAM_SAFE=y` was
already set when those crashes were found, so IRAM-safety alone is not
sufficient and stopping the timer is load-bearing.

The guard is `InrushControlSolenoid::isr_disable()` / `isr_enable()`, which is
depth-counted so nesting is safe.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "src"

SINKS = (
    "nvs_set_u8", "nvs_set_i8", "nvs_set_u16", "nvs_set_i16",
    "nvs_set_u32", "nvs_set_i32", "nvs_set_u64", "nvs_set_i64",
    "nvs_set_str", "nvs_set_blob", "nvs_commit", "nvs_erase_key", "nvs_erase_all",
    "esp_partition_write", "esp_partition_erase_range",
    "esp_flash_write", "esp_flash_erase_region",
    "save_to_eeprom",
    "save_core_config",
    "ewm_btn_save_profile",
    "write_nvs_map_data",
    "write_efuse_config",
    "set_device_mode",
    "reset_from_flash",
)

IMPLEMENTATION_FILES = {
    "nvs/eeprom_config.cpp",
    "nvs/module_settings.cpp",
    "stored_map.cpp",
    "stored_table.cpp",
    "adaptation/shift_adaptation.cpp",
    "torque_converter.cpp",
}

ALLOWLIST = {
    ("main.cpp", "save_core_config"):
        "boot path: runs before the solenoid and its timer are started",
    ("nvs/eeprom_config.cpp", "*"):
        "the NVS layer itself; callers and internal helpers hold the guard",
    ("diag/kwp2000.cpp", "write_efuse_config"):
        "efuse burn, not an NVS/flash-cache operation; different hardware path",
    ("diag/flasher.cpp", "esp_flash_write"):
        "session-scoped guard: Flasher disables the ISR for the whole OTA "
        "session and re-enables on teardown, tracked by tcc_isr_disabled",
    ("diag/flasher.cpp", "esp_flash_erase_region"):
        "session-scoped guard, as above",
}

GUARD_OPEN = re.compile(r"\bisr_disable\s*\(|\bdisable_tcc_isr\s*\(")
GUARD_CLOSE = re.compile(r"\bisr_enable\s*\(|\benable_tcc_isr\s*\(")


def strip_comments(text: str) -> str:
    """Blank out comments and string literals, preserving line structure."""
    out, i, n = [], 0, len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(c if c == "\n" else " " for c in text[i:j]))
            i = j
        elif text[i] == '"':
            j = i + 1
            while j < n and not (text[j] == '"' and text[j - 1] != "\\"):
                j += 1
            out.append(" " * (min(j + 1, n) - i))
            i = min(j + 1, n)
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def find_violations(list_all: bool = False) -> list[str]:
    findings, listing = [], []
    for path in sorted(SRC.rglob("*.cpp")):
        rel = str(path.relative_to(SRC))
        if rel in IMPLEMENTATION_FILES:
            continue
        source = strip_comments(path.read_text(errors="replace"))
        depth = 0
        guard = 0
        guard_frame = None

        for number, line in enumerate(source.splitlines(), 1):
            if guard and guard_frame is not None and depth < guard_frame:
                guard, guard_frame = 0, None

            if GUARD_OPEN.search(line):
                if guard == 0:
                    guard_frame = depth
                guard += 1
            if GUARD_CLOSE.search(line):
                guard = max(0, guard - 1)
                if guard == 0:
                    guard_frame = None

            for sink in SINKS:
                if re.search(rf"\b{re.escape(sink)}\s*\(", line):
                    allowed = (rel, sink) in ALLOWLIST or (rel, "*") in ALLOWLIST
                    state = "GUARDED" if guard else ("ALLOWED" if allowed else "UNGUARDED")
                    listing.append(f"  {state:<9} {rel}:{number}  {sink}()")
                    if not guard and not allowed:
                        findings.append(
                            f"{rel}:{number}: {sink}() writes flash with the TCC timer ISR "
                            f"running.\n    {line.strip()[:100]}"
                        )
            depth += line.count("{") - line.count("}")

    if list_all:
        print("\n".join(listing) or "  (no flash-write call sites found)")
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true",
                        help="print every flash-write call site and whether it is guarded")
    args = parser.parse_args()

    findings = find_violations(list_all=args.list)
    if not findings:
        print("OK: every flash write is inside an isr_disable()/isr_enable() region.")
        return 0
    print("\nUnguarded flash writes:\n", file=sys.stderr)
    for finding in findings:
        print(f"  {finding}\n", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
