#!/usr/bin/env python3
"""Host verification suite for the G55 V15 (origin/dev + G55 safety/diag patches) branch."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import tempfile

REPO = Path(__file__).resolve().parents[1]


def test_nvs_isr_guard_checker_clean() -> None:
    res = subprocess.run(
        [sys.executable, str(REPO / "tools" / "check_nvs_isr_guard.py")],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=False,
    )
    assert res.returncode == 0, res.stderr + res.stdout


def test_nvs_isr_guard_checker_catches_unwrapped_park_profile_save(tmp_path: Path) -> None:
    """Verify the static checker fails if ewm_btn_save_profile is outside isr_disable/isr_enable."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "check_nvs_isr_guard", REPO / "tools" / "check_nvs_isr_guard.py"
    )
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    gearbox_cpp = (REPO / "src" / "gearbox.cpp").read_text()
    # Simulate V13's bug: closing isr_enable() before ewm_btn_save_profile
    broken = gearbox_cpp.replace(
        "this->tcc->save();\n                            // Save profile inside the TCC ISR guard as well",
        "this->tcc->save();\n                            sol_tcc->isr_enable();\n                            // Save profile outside guard",
    ).replace(
        "                            sol_tcc->isr_enable();\n                        }\n                        else if (this->shifter_pos == ShifterPosition::N)",
        "                        }\n                        else if (this->shifter_pos == ShifterPosition::N)",
    )
    assert broken != gearbox_cpp
    fake_src = tmp_path / "src"
    fake_src.mkdir()
    (fake_src / "gearbox.cpp").write_text(broken)
    mod.SRC = fake_src
    findings = mod.find_violations()
    assert any("ewm_btn_save_profile" in f for f in findings), findings


def test_v15_cpp_invariants() -> None:
    """Compile and run host C++ checks for TCC_DEFAULT_SETTINGS, BOOT_INFO, and ShiftDiagnosticSnapshot."""
    harness = r"""
#include <cassert>
#include <cstdint>
#include <cstring>

// Stubs for ESP-IDF types needed by headers
typedef int esp_err_t;
#define SOLENOID_H
#define ESP_OK 0
#include "lib/core/tcu_maths.h"
#include "src/nvs/module_settings.h"
#include "src/common_structs.h"
#include "src/diag/boot_info.h"
#include "src/canbus/egs52_torque.h"

int main() {
    // 1. G55 safe TCC_AC defaults: D1-D3 disabled, D4-D5 enabled, PID and adapt enabled
    assert(TCC_DEFAULT_SETTINGS.adapt_enable == true);
    assert(TCC_DEFAULT_SETTINGS.pid_enable == true);
    assert(TCC_DEFAULT_SETTINGS.enable_d1 == false);
    assert(TCC_DEFAULT_SETTINGS.enable_d2 == false);
    assert(TCC_DEFAULT_SETTINGS.enable_d3 == false);
    assert(TCC_DEFAULT_SETTINGS.enable_d4 == true);
    assert(TCC_DEFAULT_SETTINGS.enable_d5 == true);
    assert(sizeof(TCC_MODULE_SETTINGS) == 14);
    assert(sizeof(GAR_MODULE_SETTINGS) == 16);

    // 2. Wire size invariants for RLI 0x2E and RLI 0x2F
    static_assert(sizeof(ShiftDiagnosticSnapshot) == 63, "RLI 0x2E size mismatch");
    static_assert(sizeof(BOOT_INFO) == 8, "RLI 0x2F size mismatch");

    // 3. EGS52 0x1FFF unavailable torque sentinel
    assert(decode_egs52_torque(0x1FFF) == INT16_MAX);
    assert(decode_egs52_torque(UINT16_MAX) == INT16_MAX);
    assert(decode_egs52_torque(2000) == 0);
    assert(decode_egs52_torque(2400) == 100);
    return 0;
}
"""
    with tempfile.TemporaryDirectory() as tmp:
        cpp_path = Path(tmp) / "harness.cpp"
        bin_path = Path(tmp) / "harness"
        (Path(tmp) / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n")
        (Path(tmp) / "esp_attr.h").write_text("#pragma once\n#define DRAM_ATTR\n#define IRAM_ATTR\n")
        cpp_path.write_text(harness)
        subprocess.run(
            [
                "c++",
                "-std=c++20",
                f"-I{tmp}",
                f"-I{REPO}",
                f"-I{REPO / 'src'}",
                f"-I{REPO / 'lib' / 'core'}",
                str(cpp_path),
                "-o",
                str(bin_path),
            ],
            check=True,
        )
        subprocess.run([str(bin_path)], check=True)
