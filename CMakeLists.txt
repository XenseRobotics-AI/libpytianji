cmake_minimum_required(VERSION 3.18)

project(libpytianji LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)

# ``Development.Module`` is the only Python development component pybind11
# needs.  Some conda distributions expose a tiny ``libpython3.so`` shim and
# omit the conventional ``libpython3.12.so`` SONAME; asking CMake to resolve a
# full ``Development`` bundle then incorrectly reports the component missing.
# The wheel builder pins the interpreter/include/library explicitly, while a
# standalone CMake invocation can still use the normal discovery path.
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
if(NOT pybind11_DIR)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
            "import pybind11; print(pybind11.get_cmake_dir())"
    RESULT_VARIABLE PYBIND11_LOOKUP_RESULT
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(PYBIND11_LOOKUP_RESULT EQUAL 0 AND EXISTS "${PYBIND11_CMAKE_DIR}")
    set(pybind11_DIR "${PYBIND11_CMAKE_DIR}" CACHE PATH
        "Directory containing pybind11Config.cmake" FORCE)
  endif()
endif()
find_package(pybind11 CONFIG REQUIRED)

set(TIANJI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../tianji_arm" CACHE PATH
    "Tianji vendor SDK root")
set(TIANJI_LIB_DIR "${TIANJI_ROOT}/SDK_PYTHON" CACHE PATH
    "Directory containing libKine.so and libMarvinSDK.so")
set(TIANJI_CONFIG "${TIANJI_ROOT}/CommonConfig/config/ccs_m6_40.MvKDCfg" CACHE FILEPATH
    "Default Tianji kinematics configuration")

if(NOT EXISTS "${TIANJI_CONFIG}")
  set(TIANJI_CONFIG "${TIANJI_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg")
endif()

if(NOT EXISTS "${TIANJI_ROOT}/kinematicsSDK/FxRobot.h")
  message(FATAL_ERROR "Tianji kinematicsSDK sources are missing: ${TIANJI_ROOT}")
endif()
if(NOT EXISTS "${TIANJI_LIB_DIR}/libKine.so" OR
   NOT EXISTS "${TIANJI_LIB_DIR}/libMarvinSDK.so")
  message(FATAL_ERROR "Tianji shared libraries are missing in ${TIANJI_LIB_DIR}")
endif()
if(NOT EXISTS "${TIANJI_CONFIG}")
  message(FATAL_ERROR "Tianji kinematics configuration is missing: ${TIANJI_CONFIG}")
endif()

# The real-time IK backend is an independent native Pinocchio implementation.
# Do not depend on the Python Pink/qpsolvers packages in this C++ worker: a
# Python callback would acquire the GIL and introduce unbounded latency.  Find
# the cmeel Pinocchio prefix (or accept an explicit package-specific override).
set(LIBPYTIANJI_PINOCCHIO_PREFIX "" CACHE PATH
    "Pinocchio installation prefix used by libpytianji's native Pink IK")
if(NOT LIBPYTIANJI_PINOCCHIO_PREFIX)
  if(DEFINED ENV{LIBPYTIANJI_PINOCCHIO_PREFIX} AND
     NOT "$ENV{LIBPYTIANJI_PINOCCHIO_PREFIX}" STREQUAL "")
    set(LIBPYTIANJI_PINOCCHIO_PREFIX "$ENV{LIBPYTIANJI_PINOCCHIO_PREFIX}")
  endif()
endif()
if(NOT LIBPYTIANJI_PINOCCHIO_PREFIX)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
            "import sysconfig; print(sysconfig.get_paths().get('purelib', ''))"
    OUTPUT_VARIABLE _python_purelib
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(_python_purelib)
    set(_pinocchio_candidates "${_python_purelib}/cmeel.prefix")
  endif()
  list(APPEND _pinocchio_candidates
       "${Python3_PREFIX}/cmeel.prefix" "/usr/local" "/usr")
  foreach(_candidate IN LISTS _pinocchio_candidates)
    if(EXISTS "${_candidate}/include/pinocchio")
      file(GLOB _pinocchio_libs
           "${_candidate}/lib/libpinocchio_default.so*"
           "${_candidate}/lib/libpinocchio_default.a")
      if(_pinocchio_libs)
        set(LIBPYTIANJI_PINOCCHIO_PREFIX "${_candidate}")
        break()
      endif()
    endif()
  endforeach()
endif()
if(NOT LIBPYTIANJI_PINOCCHIO_PREFIX)
  message(FATAL_ERROR
          "C++ Pinocchio was not found. Install pin in the active environment "
          "or set LIBPYTIANJI_PINOCCHIO_PREFIX to its prefix.")
endif()
if(NOT EXISTS "${LIBPYTIANJI_PINOCCHIO_PREFIX}/include/pinocchio" OR
   NOT EXISTS "${LIBPYTIANJI_PINOCCHIO_PREFIX}/lib/libpinocchio_default.so")
message(FATAL_ERROR "Invalid Pinocchio prefix: ${LIBPYTIANJI_PINOCCHIO_PREFIX}")
endif()

set(TIANJI_EIGEN_INCLUDE_DIR "/usr/include/eigen3")
if(NOT EXISTS "${TIANJI_EIGEN_INCLUDE_DIR}/Eigen/Core")
  set(TIANJI_EIGEN_INCLUDE_DIR "/home/olorin/rdk_install/include/eigen3")
endif()
if(NOT EXISTS "${TIANJI_EIGEN_INCLUDE_DIR}/Eigen/Core")
  set(TIANJI_EIGEN_INCLUDE_DIR "${LIBPYTIANJI_PINOCCHIO_PREFIX}/include")
endif()
if(NOT EXISTS "${TIANJI_EIGEN_INCLUDE_DIR}/Eigen/Core")
  message(FATAL_ERROR "Eigen headers were not found; set TIANJI_EIGEN_INCLUDE_DIR")
endif()

pybind11_add_module(_native MODULE
  src/native_controller.cpp
  src/pink_ik.cpp
  src/pybind_module.cpp
)

target_include_directories(_native PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${TIANJI_ROOT}/kinematicsSDK"
  "${TIANJI_ROOT}/DEMO_C++"
  "${LIBPYTIANJI_PINOCCHIO_PREFIX}/include"
  "${TIANJI_EIGEN_INCLUDE_DIR}"
)

target_link_directories(_native PRIVATE "${TIANJI_LIB_DIR}")
target_link_directories(_native PRIVATE "${LIBPYTIANJI_PINOCCHIO_PREFIX}/lib")
target_link_libraries(_native PRIVATE
  Kine MarvinSDK
  pinocchio_default
  boost_serialization
  Threads::Threads
)
target_compile_options(_native PRIVATE -Wall -Wextra -Wpedantic)
# Pinocchio's default joint collection contains more than Boost.MPL's legacy
# 20-type limit.  Raise it explicitly for this standalone extension so GCC
# does not instantiate a truncated boost::variant.
target_compile_definitions(_native PRIVATE
  BOOST_MPL_LIMIT_LIST_SIZE=30
  BOOST_MPL_LIMIT_MAP_SIZE=30
)

# The vendor libraries are shipped beside the extension in the wheel.  During
# an editable/source build this also keeps the checked-in SDK self-contained.
set_target_properties(_native PROPERTIES
  # The setup.py build copies both vendor libraries beside the extension, so
  # source and wheel installs use the same self-contained lookup path. The
  # Python facade preloads Pinocchio from the active ``pin`` installation.
  BUILD_RPATH "\$ORIGIN"
  BUILD_RPATH_USE_ORIGIN TRUE
  # setup.py copies the build artifact straight into the Python package. Use
  # only the package-relative install RPATH so the wheel remains relocatable.
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "\$ORIGIN"
  # Do not append every -L directory (notably the active conda prefix) to the
  # install RPATH.  The vendor libraries are copied beside the extension and
  # an environment-specific path would make the wheel non-relocatable.
  INSTALL_RPATH_USE_LINK_PATH FALSE
  OUTPUT_NAME "_native"
)

# Conda toolchains may inject the active environment into CMAKE_*_RPATH even
# when INSTALL_RPATH is set. The Python facade preloads the Pinocchio runtime
# from the active ``pin`` installation; only the package-relative vendor path
# belongs in the installed artifact.
target_link_options(_native PRIVATE
  "-Wl,--disable-new-dtags,-rpath,\$ORIGIN"
)

install(TARGETS _native LIBRARY DESTINATION libpytianji)
install(FILES
  "${TIANJI_LIB_DIR}/libKine.so"
  "${TIANJI_LIB_DIR}/libMarvinSDK.so"
  "${TIANJI_CONFIG}"
  DESTINATION libpytianji
)
