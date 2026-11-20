#!/usr/bin/env python3
"""Verify that no IRAM ISR in firmware.elf can execute .flash.text or read .flash.rodata.

On ESP32, SPI flash erase/write operations (NVS saves, OTA writes) park the other
CPU core in spi_flash_op_block_func with flash cache disabled and mask all
non-IRAM interrupts via esp_intr_noniram_disable(). Any interrupt allocated with
ESP_INTR_FLAG_IRAM continues to fire while flash cache is off. If an IRAM ISR
calls a function in .flash.text (for example, a GCC -fipa-sra / -fpartial-inlining
`$isra$0` or `$part$0` clone that evaded ESP-IDF's exact-symbol linker.lf
noflash placement) or dereferences .flash.rodata, the ESP32 immediately panics
with:
  Guru Meditation Error: Core N panic'ed (Cache error). Cache disabled but cached memory region accessed.

This gate checks:
  1. Source-built objects in firmware.elf emit zero `$isra` or `$part` IPA clones
     in .flash.text (prebuilt newlib/libstdc++ runtime archives are allowlisted).
  2. Every IRAM ISR handler reachable while flash cache is disabled has zero
     calls/jumps into .flash.text (0x400D0000..0x40400000) and zero literal-pool
     references into .flash.rodata (0x3F400000..0x3F800000).
"""
from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
import re
import shutil
import subprocess
import sys

REPO = Path(__file__).resolve().parent.parent
DEFAULT_ELF = REPO / ".pio" / "build" / "unified" / "firmware.elf"

IRAM_START = 0x40080000
IRAM_END = 0x400A0000
FLASH_TEXT_START = 0x400D0000
FLASH_TEXT_END = 0x40400000
FLASH_RODATA_START = 0x3F400000
FLASH_RODATA_END = 0x3F800000

# Prebuilt toolchain archives (libc.a / libstdc++.a) are shipped precompiled in
# toolchain-xtensa-esp-elf; all ESP-IDF and application objects are built from source.
PREBUILT_RUNTIME_CLONE_PREFIXES = (
    "get_arg$isra$",
    "_strtoll_l$isra$",
    "_strtoull_l$isra$",
    "_ZNKSt9type_infoeqERKS_$isra$",
    "med3$constprop$",
)

# Exact ISR roots (or mangled prefixes) that run with ESP_INTR_FLAG_IRAM while
# SPI flash cache is disabled.
ISR_ROOT_PATTERNS = (
    re.compile(r"^_ZL25inrush_solenoid_timer_isr"),
    re.compile(r"^_ZL29inrush_solenoid_timer_isr_new"),
    re.compile(r"^_ZN21InrushControlSolenoid18on_timer_interruptEv$"),
    re.compile(r"^_ZN21InrushControlSolenoid22on_timer_interrupt_newEv$"),
    re.compile(r"^gptimer_default_isr$"),
    re.compile(r"^twai_intr_handler_main$"),
    re.compile(r"^uart_rx_intr_handler_default$"),
    re.compile(r"^adc_dma_intr_handler$"),
    re.compile(r"^adc_dma_intr$"),
    re.compile(r"^shared_intr_isr$"),
    re.compile(r"^esp_ipc_isr_handler$"),
    re.compile(r"^esp_crosscore_isr$"),
    re.compile(r"^gpio_intr_service$"),
    re.compile(r"^timer_isr$"),
    re.compile(r"^ledc_fade_isr$"),
    re.compile(r"^i2c_master_isr_handler_default$"),
    re.compile(r"^pcnt_default_isr$"),
)

# Assert/abort cold paths that intentionally crash the chip and re-enable flash cache
# inside panic_handler before printing.
ABORT_CALLEES = {
    "__assert_func",
    "abort",
    "__stack_chk_fail",
    "esp_ipc_isr_stall_abort",
    "task_wdt_timeout_abort",
    "esp_backtrace_print",
}


def find_tool(binary_name: str) -> str:
    found = shutil.which(binary_name)
    if found:
        return found
    pio_candidate = (
        Path.home()
        / ".platformio"
        / "packages"
        / "toolchain-xtensa-esp-elf"
        / "bin"
        / binary_name
    )
    if pio_candidate.is_file():
        return str(pio_candidate)
    raise FileNotFoundError(f"Required binutils tool not found: {binary_name}")


def check_elf(elf_path: Path, check_ipa_clones: bool = True) -> list[str]:
    if not elf_path.is_file():
        return [f"ELF file not found: {elf_path}"]

    nm_bin = find_tool("xtensa-esp32-elf-nm")
    objdump_bin = find_tool("xtensa-esp32-elf-objdump")

    raw_nm = subprocess.check_output([nm_bin, "-n", "-S", str(elf_path)], text=True).splitlines()
    funcs: list[tuple[int, int, str]] = []
    by_name: dict[str, int] = {}
    violations: list[str] = []

    for line in raw_nm:
        parts = line.split()
        if len(parts) == 4:
            addr, size, stype, name = int(parts[0], 16), int(parts[1], 16), parts[2], parts[3]
        elif len(parts) == 3:
            addr, size, stype, name = int(parts[0], 16), 0, parts[1], parts[2]
        else:
            continue

        if stype in ("t", "T", "w", "W"):
            funcs.append((addr, size, name))
            by_name.setdefault(name, addr)

            if check_ipa_clones and FLASH_TEXT_START <= addr < FLASH_TEXT_END:
                if any(tag in name for tag in ("$isra$", "$part$", ".cold")) and not name.startswith(
                    PREBUILT_RUNTIME_CLONE_PREFIXES
                ):
                    violations.append(
                        f"IPA clone in .flash.text can evade ESP-IDF linker.lf noflash rules: "
                        f"0x{addr:08x} {name}"
                    )

    funcs.sort()
    norm_funcs: list[tuple[int, int, str]] = []
    for i, (addr, size, name) in enumerate(funcs):
        if size == 0 and i + 1 < len(funcs):
            size = max(0, funcs[i + 1][0] - addr)
        norm_funcs.append((addr, addr + size, name))

    def addr_to_func(target: int) -> tuple[int, int, str] | None:
        lo, hi = 0, len(norm_funcs) - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            start, end, name = norm_funcs[mid]
            if start <= target < end:
                return (start, end, name)
            if target < start:
                hi = mid - 1
            else:
                lo = mid + 1
        return None

    # Read raw 32-bit words in .iram0.text so l32r literal loads can be resolved accurately.
    # Strip the ASCII dump column (separated by two spaces after hex columns) so short lines
    # never mistake ASCII characters for hex words.
    sec_dump = subprocess.check_output(
        [objdump_bin, "-s", "-j", ".iram0.text", str(elf_path)], text=True
    )
    mem: dict[int, int] = {}
    for line in sec_dump.splitlines():
        hex_part = line.split("  ", 1)[0]
        m = re.match(r"^\s*([0-9a-f]{8})\s+([0-9a-f ]+)$", hex_part)
        if m:
            base = int(m.group(1), 16)
            for idx, word_hex in enumerate(m.group(2).split()):
                if len(word_hex) == 8:
                    mem[base + idx * 4] = int.from_bytes(bytes.fromhex(word_hex), "little")

    dis = subprocess.check_output(
        [objdump_bin, "-d", "-j", ".iram0.text", str(elf_path)], text=True
    )
    fn_insns: dict[str, list[tuple[int, str]]] = {}
    for line in dis.splitlines():
        m = re.match(r"^\s*([0-9a-f]{8}):\s+[0-9a-f]+\s+(.*)$", line)
        if m:
            pc = int(m.group(1), 16)
            owner = addr_to_func(pc)
            if owner is not None:
                fn_insns.setdefault(owner[2], []).append((pc, m.group(2).strip()))

    adj: dict[str, set[tuple[int, str, int]]] = {}
    rodata_refs: dict[str, set[tuple[int, int]]] = {}
    for start, _end, name in norm_funcs:
        if not (IRAM_START <= start < IRAM_END):
            continue
        callees: set[tuple[int, str, int]] = set()
        ro_loads: set[tuple[int, int]] = set()
        for pc, insn in fn_insns.get(name, []):
            mc = re.match(
                r"^(?:call(?:4|8|12)|j)\s+([0-9a-f]{8})\s+<([^+>]+)(?:\+0x[0-9a-f]+)?>",
                insn,
            )
            if mc:
                tgt = int(mc.group(1), 16)
                tgt_name = mc.group(2)
                # Ignore intra-function branches (`j` within same function).
                if tgt_name != name:
                    callees.add((tgt, tgt_name, pc))
            ml = re.match(r"^l32r\s+a\d+,\s*([0-9a-f]{8})", insn)
            if ml:
                lit_addr = int(ml.group(1), 16)
                val = mem.get(lit_addr)
                if val is not None:
                    if FLASH_RODATA_START <= val < FLASH_RODATA_END:
                        ro_loads.add((pc, val))
                    elif FLASH_TEXT_START <= val < FLASH_TEXT_END:
                        tf = addr_to_func(val)
                        callees.add((val, tf[2] if tf else f"0x{val:08x}", pc))
                    elif IRAM_START <= val < IRAM_END:
                        tf = addr_to_func(val)
                        if tf and tf[0] == val and tf[2] != name:
                            callees.add((val, tf[2], pc))
        adj[name] = callees
        rodata_refs[name] = ro_loads

    isr_roots = sorted(
        {
            name
            for start, _end, name in norm_funcs
            if IRAM_START <= start < IRAM_END
            and any(pat.search(name) for pat in ISR_ROOT_PATTERNS)
        }
    )
    if not isr_roots:
        violations.append("No IRAM ISR roots found in firmware.elf; symbol pattern drift?")
        return violations

    # Mandatory core ISRs that must always exist in IRAM on unified builds:
    for required_prefix in (
        "_ZL25inrush_solenoid_timer_isr",
        "gptimer_default_isr",
        "twai_intr_handler_main",
        "uart_rx_intr_handler_default",
        "adc_dma_intr_handler",
    ):
        if not any(r.startswith(required_prefix) for r in isr_roots):
            violations.append(f"Expected IRAM ISR handler missing from .iram0.text: {required_prefix}")

    visited: set[str] = set()
    queue: deque[tuple[str, list[str]]] = deque((root, [root]) for root in isr_roots)
    while queue:
        cur, path = queue.popleft()
        if cur in visited or cur in ABORT_CALLEES:
            continue
        visited.add(cur)

        for pc, val in sorted(rodata_refs.get(cur, [])):
            violations.append(
                f"IRAM ISR loads .flash.rodata 0x{val:08x} at 0x{pc:08x}: {' -> '.join(path)}"
            )

        for tgt_addr, tgt_name, pc in sorted(adj.get(cur, [])):
            if tgt_name in ABORT_CALLEES:
                continue
            if FLASH_TEXT_START <= tgt_addr < FLASH_TEXT_END:
                violations.append(
                    f"IRAM ISR calls .flash.text {tgt_name} (0x{tgt_addr:08x}) from 0x{pc:08x}: "
                    f"{' -> '.join(path)} -> {tgt_name}"
                )
            elif IRAM_START <= tgt_addr < IRAM_END and tgt_name not in visited:
                queue.append((tgt_name, path + [tgt_name]))

    return violations


def check_sdkconfig(sdkconfig_path: Path) -> list[str]:
    if not sdkconfig_path.is_file():
        return [f"sdkconfig file not found: {sdkconfig_path}"]
    text = sdkconfig_path.read_text(errors="replace")
    violations: list[str] = []
    for forbidden in (
        "CONFIG_I2C_ISR_IRAM_SAFE=y",
        "CONFIG_I2C_MASTER_ISR_HANDLER_IN_IRAM=y",
        "CONFIG_I2S_ISR_IRAM_SAFE=y",
        "CONFIG_PCNT_ISR_IRAM_SAFE=y",
    ):
        if re.search(rf"^{re.escape(forbidden)}$", text, flags=re.MULTILINE):
            violations.append(
                f"{sdkconfig_path.name} enables {forbidden}; polling/unused peripheral ISRs "
                f"must not set ESP_INTR_FLAG_IRAM during SPI flash operations"
            )
    return violations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF, help="Path to firmware.elf")
    parser.add_argument(
        "--sdkconfig",
        type=Path,
        default=REPO / "sdkconfig.unified",
        help="Path to sdkconfig.unified",
    )
    parser.add_argument(
        "--skip-sdkconfig",
        action="store_true",
        help="Check only the ELF file (useful when testing historical ELFs)",
    )
    args = parser.parse_args()

    violations: list[str] = []
    if not args.skip_sdkconfig:
        violations.extend(check_sdkconfig(args.sdkconfig))
    violations.extend(check_elf(args.elf))

    if not violations:
        print(f"OK: {args.elf} has zero .flash.text/.flash.rodata hazards in IRAM ISRs.")
        return 0

    print("IRAM ISR flash-cache safety violations found:", file=sys.stderr)
    for item in violations:
        print(f"  - {item}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
