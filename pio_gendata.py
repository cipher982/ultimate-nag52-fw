import importlib.util

if __name__ == "SCons.Script":
    if importlib.util.find_spec("yaml") is None:
        raise RuntimeError(
            "PyYAML is required in the PlatformIO Python environment. "
            "Provision platformio with --with pyyaml before building."
        )
    from scripts import generate_data