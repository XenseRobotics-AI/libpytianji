from __future__ import annotations

import os
import pathlib
import shlex
import shutil
import subprocess
import sys
import sysconfig

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext
from setuptools.command.build_py import build_py as _build_py


class CMakeExtension(Extension):
    def __init__(self, name: str) -> None:
        super().__init__(name, sources=[])


def _find_pinocchio_prefix() -> pathlib.Path:
    """Locate the cmeel Pinocchio prefix used by the active interpreter."""

    configured = os.environ.get("LIBPYTIANJI_PINOCCHIO_PREFIX")
    candidates: list[pathlib.Path] = []
    if configured:
        candidates.append(pathlib.Path(configured).expanduser())
    purelib = pathlib.Path(sysconfig.get_path("purelib") or "")
    if purelib:
        candidates.append(purelib / "cmeel.prefix")
    python_prefix = pathlib.Path(sys.prefix)
    candidates.append(python_prefix / "cmeel.prefix")
    candidates.extend((pathlib.Path("/usr/local"), pathlib.Path("/usr")))
    for candidate in candidates:
        if (
            (candidate / "include" / "pinocchio").is_dir()
            and any((candidate / "lib").glob("libpinocchio_default.so*"))
        ):
            return candidate.resolve()
    raise RuntimeError(
        "C++ Pinocchio was not found for libpytianji. Install the pin package "
        "in the active environment or set LIBPYTIANJI_PINOCCHIO_PREFIX."
    )


class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        source_dir = pathlib.Path(__file__).resolve().parent
        output_dir = pathlib.Path(self.get_ext_fullpath(ext.name)).resolve().parent
        output_dir.mkdir(parents=True, exist_ok=True)
        build_dir = pathlib.Path(self.build_temp) / "libpytianji"
        # Never reuse a CMake cache from a different conda environment.  The
        # cache can retain an absolute ``LDFLAGS`` RPATH and make an otherwise
        # self-contained wheel depend on the machine that built it.
        if build_dir.exists():
            shutil.rmtree(build_dir)
        build_dir.mkdir(parents=True, exist_ok=True)

        # Conda exports its own lib directory through LDFLAGS and, for its
        # compiler wrapper, unconditionally appends an environment RPATH.
        # The Tianji wheel deliberately carries libKine.so/libMarvinSDK.so
        # beside the pybind module. Strip linker RPATH options and use the
        # system compiler when it is available; this keeps the artifact
        # relocatable without requiring the external ``patchelf`` executable.
        build_env = os.environ.copy()
        raw_ldflags = build_env.get("LDFLAGS", "")
        if raw_ldflags:
            try:
                ldflag_tokens = shlex.split(raw_ldflags)
            except ValueError:
                # Let CMake report malformed flags rather than silently
                # changing an administrator-provided toolchain string.
                ldflag_tokens = raw_ldflags.split()
            filtered_ldflags: list[str] = []
            skip_rpath_argument = False
            for token in ldflag_tokens:
                if skip_rpath_argument:
                    skip_rpath_argument = False
                    continue
                if token in {"-Wl,-rpath", "-Wl,-rpath-link"}:
                    skip_rpath_argument = True
                    continue
                if token.startswith(("-Wl,-rpath,", "-Wl,-rpath-link,")):
                    continue
                filtered_ldflags.append(token)
            build_env["LDFLAGS"] = " ".join(filtered_ldflags)
        # CMake also honors these variables independently of LDFLAGS.  They
        # are not needed for this module because INSTALL_RPATH is explicit.
        for variable in (
            "CMAKE_RPATH",
            "CMAKE_BUILD_RPATH",
            "CMAKE_INSTALL_RPATH",
            "CMAKE_INSTALL_RPATH_USE_LINK_PATH",
            "CMAKE_SHARED_LINKER_FLAGS",
            "CMAKE_MODULE_LINKER_FLAGS",
        ):
            build_env.pop(variable, None)
        # The conda compiler's ``conda.specs`` adds ``-rpath $CONDA_PREFIX``
        # after all user flags, so filtering LDFLAGS alone cannot remove it.
        # A host GCC/G++ can link the vendor C++17 SDK and does not inject that
        # path.  Respect an explicitly selected compiler, otherwise prefer the
        # system pair for wheel builds; runtime dependencies remain covered by
        # the Python environment's normal loader/preloader.
        # Prefer the distro compiler by absolute path.  ``which g++`` inside
        # an activated conda environment normally resolves to a symlink whose
        # wrapper injects conda.specs (and an absolute RPATH) even when all
        # environment LDFLAGS were removed.
        system_c = "/usr/bin/gcc" if pathlib.Path("/usr/bin/gcc").is_file() else shutil.which("gcc")
        system_cxx = "/usr/bin/g++" if pathlib.Path("/usr/bin/g++").is_file() else shutil.which("g++")
        if system_c and system_cxx and not os.environ.get("TIANJI_USE_CONDA_COMPILER"):
            build_env["CC"] = system_c
            build_env["CXX"] = system_cxx
            # The activated conda environment exports a CMAKE_ARGS toolchain
            # (including its linker and compiler archives).  It would defeat
            # the compiler selection above, so leave that optional override
            # disabled unless a caller explicitly opts into the conda pair.
            build_env.pop("CMAKE_ARGS", None)
        pybind_cmake = subprocess.check_output(
            [sys.executable, "-c", "import pybind11; print(pybind11.get_cmake_dir())"],
            text=True,
        ).strip()
        pinocchio_prefix = _find_pinocchio_prefix()
        # Conda's Python ships the development headers and shared library,
        # but CMake's FindPython3 can select the system interpreter first and
        # then report ``Development.Module`` missing.  Pin both development
        # paths to the interpreter that is building the wheel so a clean
        # environment (and a post-reboot rebuild) behaves like an editable
        # build that already has a warm CMake cache.
        python_include = sysconfig.get_path("platinclude") or sysconfig.get_path("include")
        python_libdir = sysconfig.get_config_var("LIBDIR")
        python_ldlibrary = sysconfig.get_config_var("LDLIBRARY")
        python_library = None
        if python_libdir:
            library_candidates = []
            if python_ldlibrary:
                library_candidates.append(pathlib.Path(python_libdir) / python_ldlibrary)
            # Conda commonly reports the unavailable static archive in
            # ``LDLIBRARY`` even though the shared library is present.  A
            # shared Python library is sufficient for the pybind module and
            # is what FindPython3's Development.Module component needs.
            library_candidates.extend(
                pathlib.Path(python_libdir).glob("libpython*.so")
            )
            library_candidates.extend(
                pathlib.Path(python_libdir).glob("libpython*.so.*")
            )
            python_library = next(
                (candidate for candidate in library_candidates if candidate.is_file()),
                None,
            )
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={output_dir}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE={output_dir}",
            f"-DPython3_EXECUTABLE={sys.executable}",
            f"-DCMAKE_PREFIX_PATH={pybind_cmake}",
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DLIBPYTIANJI_PINOCCHIO_PREFIX={pinocchio_prefix}",
        ]
        if python_include:
            cmake_args.append(f"-DPython3_INCLUDE_DIR={python_include}")
        if python_library and python_library.is_file():
            cmake_args.append(f"-DPython3_LIBRARY={python_library}")
        # CMake is configured from its temporary build directory.  Resolve a
        # caller-provided relative SDK path before handing it to CMake; a
        # relative ``TIANJI_SDK_ROOT=third_party/libpytianji/vendor/tianji_arm`` otherwise gets
        # interpreted relative to ``build/temp...`` and makes wheel builds
        # fail after a reboot or from a different working directory.
        vendor_value = pathlib.Path(
            os.environ.get("TIANJI_SDK_ROOT", str(source_dir / "vendor" / "tianji_arm"))
        ).expanduser()
        if vendor_value.is_absolute():
            vendor_root = vendor_value.resolve()
        else:
            # PEP 517 backends are commonly invoked with the source package as
            # their cwd (pip's in-process runner does exactly that), whereas
            # setup_env.sh passes paths relative to the repository root.  Try
            # both roots and select the first existing SDK so either invocation
            # is deterministic.
            candidates = (
                pathlib.Path.cwd() / vendor_value,
                source_dir.parent.parent / vendor_value,
                source_dir.parent / vendor_value,
            )
            vendor_root = next(
                (candidate.resolve() for candidate in candidates if candidate.is_dir()),
                (pathlib.Path.cwd() / vendor_value).resolve(),
            )
        cmake_args.append(f"-DTIANJI_ROOT={vendor_root}")
        subprocess.check_call(
            ["cmake", str(source_dir), *cmake_args], cwd=build_dir, env=build_env
        )
        subprocess.check_call(
            ["cmake", "--build", ".", "--config", "Release", "-j2"],
            cwd=build_dir,
            env=build_env,
        )
        built = next(output_dir.glob("_native*.so"), None)
        if built is None:
            # Some CMake generators place the artifact in the build directory.
            built = next(build_dir.rglob("_native*.so"), None)
        if built is None:
            raise RuntimeError("CMake did not produce libpytianji/_native.so")
        destination = pathlib.Path(self.get_ext_fullpath(ext.name)).resolve()
        if built.resolve() != destination:
            shutil.copy2(built, destination)
        # Conda's toolchain injects its library directory into the linker's
        # DT_RPATH. Keep only the package-relative vendor lookup in the wheel;
        # Pinocchio is preloaded by the Python facade from the active ``pin``
        # installation.
        # The vendor SDK is copied beside the extension and the Python runtime
        # supplies libstdc++/libgcc. Removing the absolute build prefix is
        # important for relocation to another host or conda environment.
        if sys.platform.startswith("linux"):
            try:
                subprocess.check_call(
                    ["patchelf", "--remove-rpath", str(destination)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                subprocess.check_call(
                    [
                        "patchelf",
                        "--set-rpath",
                        "$ORIGIN",
                        str(destination),
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except FileNotFoundError:
                # The linker flags above already produce an $ORIGIN-only
                # RPATH.  patchelf is merely an optional post-build repair for
                # unusual toolchains and is not a package installation
                # prerequisite.
                pass
            except subprocess.CalledProcessError as error:
                raise RuntimeError(
                    "patchelf could not make the libpytianji extension relocatable"
                ) from error
        lib_dir = vendor_root / "SDK_PYTHON"
        package_dir = destination.parent
        # ``build_ext --inplace`` asks setuptools to compile under
        # ``build/lib...`` and then copies only the extension into the source
        # package.  The extension uses an ``$ORIGIN`` RUNPATH, so an editable
        # install would otherwise import successfully during the build but
        # fail later with ``libKine.so: cannot open shared object``.  Keep the
        # source package self-contained as well as the wheel staging package;
        # this is also convenient for the repository's hardware smoke tests.
        package_destinations = {package_dir, source_dir / "libpytianji"}
        for name in ("libKine.so", "libMarvinSDK.so"):
            source = lib_dir / name
            if not source.is_file():
                raise RuntimeError(f"Tianji runtime library not found: {source}")
            for destination_dir in package_destinations:
                destination_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination_dir / name)

        # The native planner accepts an absolute path at runtime, but a wheel
        # must remain self-contained.  Ship the default model beside the
        # extension and let the LeRobot adapter fall back to it when the
        # repository checkout is not the current working directory.
        config_candidates = (
            vendor_root / "CommonConfig" / "config" / "ccs_m6_40.MvKDCfg",
            vendor_root / "CommonConfig" / "ccs_m6_40.MvKDCfg",
        )
        config_source = next((candidate for candidate in config_candidates if candidate.is_file()), None)
        if config_source is None:
            raise RuntimeError("Tianji default ccs_m6_40.MvKDCfg was not found")
        for destination_dir in package_destinations:
            destination_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(config_source, destination_dir / config_source.name)


class build_py(_build_py):
    """Remove artifacts from an older in-tree native package before copying.

    ``libpytianji`` used to be developed under the temporary name
    ``tianji_arm_rt``. Setuptools reuses ``build/lib`` when a wheel is built,
    so a stale build directory could otherwise leak that old package (and its
    extension) into a new wheel.
    """

    def run(self) -> None:
        build_root = pathlib.Path(self.build_lib)
        legacy_package = build_root / "tianji_arm_rt"
        if legacy_package.is_dir():
            shutil.rmtree(legacy_package)
        package_root = build_root / "libpytianji"
        for legacy_extension in package_root.glob("_tianji_arm_rt*.so"):
            legacy_extension.unlink()
        super().run()


setup(
    name="libpytianji",
    version="0.1.3",
    packages=find_packages(include=["libpytianji"]),
    ext_modules=[CMakeExtension("libpytianji._native")],
    cmdclass={"build_ext": CMakeBuild, "build_py": build_py},
    zip_safe=False,
    python_requires=">=3.12,<3.14",
    package_data={
        "libpytianji": [
            "_native*.so",
            "libKine.so",
            "libMarvinSDK.so",
            "*.MvKDCfg",
            "urdf/*.urdf",
            "meshes/*.STL",
        ]
    },
)
