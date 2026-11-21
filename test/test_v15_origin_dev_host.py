#!/usr/bin/env python3
"""Host verification suite for the G55 V15/V16/V17 (origin/dev + G55 safety/diag patches) branch."""

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
        "this->tcc->save();\n                                // Save profile inside the TCC ISR guard as well",
        "this->tcc->save();\n                                sol_tcc->isr_enable();\n                                // Save profile outside guard",
    ).replace(
        "                                sol_tcc->isr_enable();\n                            }\n                        }\n                        else if (this->shifter_pos == ShifterPosition::N)",
        "                            }\n                        }\n                        else if (this->shifter_pos == ShifterPosition::N)",
    )
    assert broken != gearbox_cpp
    fake_src = tmp_path / "src"
    fake_src.mkdir()
    (fake_src / "gearbox.cpp").write_text(broken)
    mod.SRC = fake_src
    findings = mod.find_violations()
    assert any("ewm_btn_save_profile" in f for f in findings), findings


def test_boot_in_park_skips_redundant_nvs_save() -> None:
    """Verify Gearbox::controller_loop only writes Park NVS blobs when entering P from a known position."""
    gearbox_cpp = (REPO / "src" / "gearbox.cpp").read_text()
    assert "const ShifterPosition prev_position = last_position;" in gearbox_cpp
    assert "if (prev_position != ShifterPosition::SignalNotAvailable)" in gearbox_cpp


def test_sdkconfig_and_build_flags_prevent_ipa_iram_escapes() -> None:
    """Verify sdkconfig and CMake/PlatformIO flags guard against GCC IPA IRAM escapes."""
    cmake_txt = (REPO / "CMakeLists.txt").read_text()
    pio_ini = (REPO / "platformio.ini").read_text()
    assert "-fno-ipa-sra" in cmake_txt and "-fno-partial-inlining" in cmake_txt
    assert "-fno-ipa-sra" in pio_ini and "-fno-partial-inlining" in pio_ini

    elf = REPO / ".pio" / "build" / "unified" / "firmware.elf"
    cmd = [sys.executable, str(REPO / "tools" / "check_iram_isr_safety.py")]
    if elf.is_file():
        res = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, check=False)
        assert res.returncode == 0, res.stderr + res.stdout
    else:
        import importlib.util

        spec = importlib.util.spec_from_file_location(
            "check_iram_isr_safety", REPO / "tools" / "check_iram_isr_safety.py"
        )
        assert spec and spec.loader
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        assert mod.check_sdkconfig(REPO / "sdkconfig.unified") == []


def test_v17_rollback_keys_and_start_enable_invariants() -> None:
    """Verify V17 NVS rollback key preservation and PR #272 start-enable sync invariants."""
    eeprom_cpp = (REPO / "src" / "nvs" / "eeprom_config.cpp").read_text()
    assert '"SBS_A7"' in eeprom_cpp, "SBS_A7 must remain in LEGACY_ROLLBACK_KEYS for V16 rollback"

    gearbox_h = (REPO / "src" / "gearbox.h").read_text()
    gearbox_cpp = (REPO / "src" / "gearbox.cpp").read_text()
    main_cpp = (REPO / "src" / "main.cpp").read_text()
    assert "std::atomic<bool> diag_stop_control{false};" in gearbox_h
    assert "std::atomic<bool> is_start_safe{false};" in gearbox_h
    assert "bool get_is_start_safe(void) const { return this->is_start_safe && !this->diag_stop_control; }" in gearbox_h
    assert "this->is_start_safe = lock_state;" in gearbox_cpp
    assert "egs_can_hal->set_safe_start(this->get_is_start_safe());" in gearbox_cpp
    assert "gearbox->get_is_start_safe()" in main_cpp
    assert "this->timer_shifter_n = 10;" not in gearbox_cpp, (
        "timer_shifter_n = 10 can latch Neutral with lever in D after a <200 ms N excursion"
    )

    flasher_cpp = (REPO / "src" / "diag" / "flasher.cpp").read_text()
    diag_data_cpp = (REPO / "src" / "diag" / "diag_data.cpp").read_text()
    kwp_cpp = (REPO / "src" / "diag" / "kwp2000.cpp").read_text()
    assert "if (!this->tcc_isr_disabled && nullptr != sol_tcc)" in flasher_cpp
    assert "if (nullptr != this->gearbox_ref)" in flasher_cpp
    assert "if (nullptr != sol_tcc) { sol_tcc->isr_disable();" in diag_data_cpp
    assert kwp_cpp.count("if (nullptr != sol_tcc)") >= 8


def test_v15_cpp_invariants() -> None:
    """Compile and run host C++ checks for TCC_DEFAULT_SETTINGS, SBS_MODULE_SETTINGS, BOOT_INFO, and ShiftDiagnosticSnapshot."""
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
    // 1. G55 fail-closed safe TCC_AC defaults: all lockup gears, PID, and adapt disabled
    assert(TCC_DEFAULT_SETTINGS.adapt_enable == false);
    assert(TCC_DEFAULT_SETTINGS.pid_enable == false);
    assert(TCC_DEFAULT_SETTINGS.enable_d1 == false);
    assert(TCC_DEFAULT_SETTINGS.enable_d2 == false);
    assert(TCC_DEFAULT_SETTINGS.enable_d3 == false);
    assert(TCC_DEFAULT_SETTINGS.enable_d4 == false);
    assert(TCC_DEFAULT_SETTINGS.enable_d5 == false);
    assert(sizeof(TCC_MODULE_SETTINGS) == 14);
    assert(sizeof(SBS_MODULE_SETTINGS) == 14);
    assert(SBS_DEFAULT_SETTINGS.kickdown_rearm_rpm_offset == 500);
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


if __name__ == "__main__":
    test_nvs_isr_guard_checker_clean()
    with tempfile.TemporaryDirectory() as td:
        test_nvs_isr_guard_checker_catches_unwrapped_park_profile_save(Path(td))
    test_boot_in_park_skips_redundant_nvs_save()
    test_sdkconfig_and_build_flags_prevent_ipa_iram_escapes()
    test_v17_rollback_keys_and_start_enable_invariants()
    test_v15_cpp_invariants()
    print("OK: all host invariants passed")
