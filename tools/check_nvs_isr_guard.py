#!/usr/bin/env python3
"""Every flash write must happen with the TCC solenoid's timer ISR disabled.

# The rule, and why it exists

Writing NVS on an ESP32 disables the flash cache. The inrush-solenoid gptimer
ISR has been observed to crash the chip during that window -- see 687dfbc
("Fix crashing when flashing and saving maps in P") and 2409592 ("Fix TCC ISR
causing crashes in save to NVS code"). `CONFIG_GPTIMER_ISR_IRAM_SAFE=y` was
already set when those crashes were found, so IRAM-safety alone is not
sufficient and stopping the timer is load-bearing.

The guard is `InrushControlSolenoid::isr_disable()` / `isr_enable()`, which is
depth-counted so nesting is safe.

# Why a checker rather than care

A real vehicle found the last violation of this rule, three weeks and five
transmission-controller restarts after it was introduced -- and it was
introduced *by the commit that added the guard*:

    sol_tcc->isr_disable();
        shift_adapter->save();      // guarded
        tcc->save();                // guarded
    sol_tcc->isr_enable();          // released one statement too early
    // Save profile
        EEPROM::ewm_btn_save_profile(tag);   // nvs_set_u8, unguarded

Nothing about that is visible in review. It is one line in the wrong place, on
a path that only executes when the driver selects Park, and it only crashes
when the timer happens to fire inside a short write window. The cheapest way to
never repeat it is to make it a build failure.

# What this does and does not check

It is a lexical check, deliberately. A call to a flash-writing symbol must sit
between an `isr_disable()` and its matching `isr_enable()` inside the same
function body. That is exactly the property that was violated, and it needs no
build, no toolchain and no hardware -- so it can run on every push.

It does not follow calls through pointers, does not do inter-procedural
analysis, and will not catch a write reached indirectly through a helper that
is not in SINKS. Extend SINKS when you add a wrapper.

    python3 tools/check_nvs_isr_guard.py          # check
    python3 tools/check_nvs_isr_guard.py --list   # show every call site and its state
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "src"

# Symbols that reach flash. Direct ESP-IDF NVS writes, plus the project's own
# wrappers around them. A wrapper belongs here the moment it can write.
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

# The NVS layer itself. Its internal writes are the implementation of the
# sinks; requiring each to hold the guard would be circular. The obligation
# belongs to its callers, which is what this checker enforces.
IMPLEMENTATION_FILES = {
    # The NVS access layer proper.
    "nvs/eeprom_config.cpp",
    "nvs/module_settings.cpp",
    # Stored map/table plumbing: `save_to_eeprom` here *is* the sink that
    # callers must guard, so requiring the guard within it would be circular.
    "stored_map.cpp",
    "stored_table.cpp",
    # ShiftAdaptationSystem::save() is a fan-out over four StoredMaps. It is a
    # sink from the caller's point of view -- and its one caller, the Park-entry
    # branch in gearbox.cpp, does hold the guard.
    "adaptation/shift_adaptation.cpp",
}

# Call sites that genuinely need no guard. Every entry must say why, and the
# reason must be about *when* the code runs, not about it seeming unlikely.
ALLOWLIST = {
    # path, symbol -> justification
    ("main.cpp", "save_core_config"):
        "boot path: runs before the solenoid and its timer are constructed",
    ("nvs/eeprom_config.cpp", "*"):
        "the NVS layer itself; callers hold the guard",
    ("diag/kwp2000.cpp", "write_efuse_config"):
        "efuse burn, not an NVS/flash-cache operation; different hardware path",
    ("diag/flasher.cpp", "esp_flash_write"):
        "session-scoped guard: Flasher disables the ISR for the whole OTA "
        "session and re-enables on teardown, tracked by tcc_isr_disabled",
    ("diag/flasher.cpp", "esp_flash_erase_region"):
        "session-scoped guard, as above",
}

GUARD_OPEN = re.compile(r"\bisr_disable\s*\(")
GUARD_CLOSE = re.compile(r"\bisr_enable\s*\(")
# A function definition at file scope: a line ending in `{` that is not a
# control-flow keyword. Good enough for this codebase's formatting.
FUNC_START = re.compile(r"^[A-Za-z_][\w:<>,&*\s]*\([^;]*\)\s*(const\s*)?\{?\s*$")


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
        depth = 0          # brace depth
        guard = 0          # isr_disable depth
        guard_frame = None  # brace depth at which the guard was opened

        for number, line in enumerate(source.splitlines(), 1):
            # A guard cannot survive leaving the block it was opened in.
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
    print(
        "Writing NVS disables the flash cache; the TCC solenoid's gptimer ISR has been\n"
        "observed to crash the chip in that window (687dfbc, 2409592). Wrap the write in\n"
        "sol_tcc->isr_disable() / isr_enable(), or add it to ALLOWLIST in this file with a\n"
        "reason about when the code runs.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
