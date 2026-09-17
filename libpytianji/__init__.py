"""Public Python facade for the native Tianji real-time controller.

The extension is optional: importing this module on a machine without the
Tianji wheel gives a useful installation error only when the native class is
requested.  LeRobot's generic imports therefore remain hardware-free.
"""

from __future__ import annotations

import ctypes
import sys
from contextlib import suppress
from pathlib import Path


def _preload_pinocchio_runtime() -> None:
    """Make the cmeel Pinocchio libraries visible to the native extension.

    ``pin`` installs its C++ libraries below ``cmeel.prefix/lib`` rather than
    a conventional interpreter ``lib`` directory.  Importing Pinocchio alone
    is not sufficient for every ELF loader because its extension may keep the
    dependency local; load the two libraries required by ``_native`` with
    global visibility before importing the pybind module.  This also makes a
    wheel built in one virtual environment work after installation in another
    one, where the absolute build-time RPATH is no longer valid.
    """

    try:
        import pinocchio  # type: ignore[import-not-found]
    except ImportError:
        # Preserve the normal native-import error below, which explains how
        # to install the optional backend when Pinocchio is genuinely absent.
        return

    pinocchio_file = Path(str(pinocchio.__file__)).resolve()
    lib_candidates = [
        pinocchio_file.parents[3],  # cmeel.prefix/lib
        Path(sys.prefix) / "lib",
    ]
    seen: set[Path] = set()
    for lib_dir in lib_candidates:
        if lib_dir in seen or not lib_dir.is_dir():
            continue
        seen.add(lib_dir)
        for name in ("libboost_serialization.so.1.90.0", "libpinocchio_default.so"):
            path = lib_dir / name
            if not path.is_file():
                continue
            with suppress(OSError):
                ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)


_preload_pinocchio_runtime()

try:
    from ._native import (  # type: ignore[import-not-found]
        API_VERSION,
        IK_BACKEND,
        ControllerMode,
        NativeTianjiController,
        PinkIKModel,
        TianjiState,
        TianjiStatus,
    )
except ImportError as _native_error:  # pragma: no cover - depends on install
    _IMPORT_ERROR = _native_error

    def _missing_native(*args, **kwargs):
        raise ImportError(
            "libpytianji native extension is not installed. Run "
            "bash setup_env.sh --install tianji in the lerobot-xensehand "
            "environment."
        ) from _IMPORT_ERROR

    NativeTianjiController = _missing_native  # type: ignore[assignment,misc]
    TianjiState = None  # type: ignore[assignment,misc]
    TianjiStatus = None  # type: ignore[assignment,misc]
    PinkIKModel = _missing_native  # type: ignore[assignment,misc]
    IK_BACKEND = "unavailable"
    ControllerMode = None  # type: ignore[assignment,misc]
    API_VERSION = 0

__version__ = "0.1.4"

__all__ = [
    "API_VERSION",
    "ControllerMode",
    "NativeTianjiController",
    "TianjiState",
    "TianjiStatus",
    "PinkIKModel",
    "IK_BACKEND",
    "__version__",
]
