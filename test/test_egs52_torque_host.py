#!/usr/bin/env python3
"""Build and exercise the firmware's EGS52 torque conversion without hardware."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[1]


class Egs52TorqueHostTest(unittest.TestCase):
    def test_valid_and_unavailable_fields(self):
        with tempfile.TemporaryDirectory(prefix="nag52-egs52-torque-", dir="/tmp/agents") as temp:
            binary = Path(temp) / "egs52_torque"
            subprocess.run(
                [
                    os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I" + str(REPO / "src"), str(REPO / "test/host_egs52_torque.cpp"),
                    "-o", str(binary),
                ],
                check=True,
                timeout=60,
            )
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
