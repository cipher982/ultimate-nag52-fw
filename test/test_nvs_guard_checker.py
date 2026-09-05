#!/usr/bin/env python3
"""Regression checks for the lexical coverage boundaries (not runtime proof)."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('guard_checker', Path(__file__).resolve().parents[1] / 'tools/check_nvs_isr_guard.py')
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


class GuardCheckerTest(unittest.TestCase):
    def test_headers_and_previously_excluded_implementations_are_scanned(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            names = ['nvs/eeprom_impl.h', 'nvs/eeprom_config.cpp', 'nvs/module_settings.cpp',
                     'stored_map.cpp', 'stored_table.cpp', 'adaptation/shift_adaptation.cpp', 'new_write.c']
            for name in names:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('void write() { nvs_set_blob\n(handle, key, data, size); }')
            errors = CHECKER.find_violations(src=root)
            self.assertEqual({e.split(':')[0] for e in errors}, set(names))

    def test_scope_exit_and_explicit_release_end_coverage(self):
        errors, _ = CHECKER.inspect_source('''
void persist() {
    { TccFlashGuard guard; nvs_commit(h); }
    nvs_commit(h);
    TccFlashGuard outer;
    nvs_commit(h);
    outer.release();
    nvs_commit(h);
}
''', 'new.cpp')
        self.assertEqual(len(errors), 2)
        self.assertIn('new.cpp:4:', errors[0])
        self.assertIn('new.cpp:8:', errors[1])

    def test_comments_and_strings_do_not_acquire_a_guard(self):
        errors, _ = CHECKER.inspect_source('''
void persist() {
    // TccFlashGuard fake;
    const char* text = "TccFlashGuard fake;";
    nvs_commit(h);
}
''', 'new.hpp')
        self.assertEqual(len(errors), 1)

    def test_session_exception_is_method_and_sink_specific(self):
        errors, _ = CHECKER.inspect_source('''
void Flasher::on_transfer_data() {
    if (!flash_guard.owns_lock()) return;
    esp_flash_write(chip, data, address, size);
    nvs_commit(h);
}
void Flasher::unexpected() {
    esp_flash_write(chip, data, address, size);
}
''', 'diag/flasher.cpp')
        self.assertEqual(len(errors), 2)
        self.assertIn('nvs_commit', errors[0])
        self.assertIn('esp_flash_write', errors[1])

    def test_rw_handle_creation_requires_ownership(self):
        errors, _ = CHECKER.inspect_source('''
void read() { NvsHandle handle; }
void unsafe() { NvsHandle handle(NVS_READWRITE); }
void safe() { TccFlashGuard guard; NvsHandle handle(NVS_READWRITE); }
''', 'nvs/eeprom_config.cpp')
        self.assertEqual(len(errors), 1)


if __name__ == '__main__':
    unittest.main()
