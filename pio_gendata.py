import io
import subprocess

from typing import Tuple

if __name__ == "SCons.Script":
    Import("env")
    try:
        import yaml  # noqa: F401
    except ImportError:
        env.Execute("$PYTHONEXE -m pip install pyyaml")
    from scripts import generate_data