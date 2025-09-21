cmake_minimum_required(VERSION 3.22)
project(iNVNt VERSION 0.1.0 LANGUAGES CXX)

# Global basics
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(INVNT_GEN_INC ${PROJECT_BINARY_DIR}/generated/include)
set(INVNT_SHARE_DIR "${PROJECT_BINARY_DIR}/share")

# Create share directory
file(MAKE_DIRECTORY "${INVNT_SHARE_DIR}")

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(INVNT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(INVNT_BUILD_TESTS "Build tests" ON)
option(INVNT_WITH_OSQP "Enable OSQP-based solvers where available" OFF)
option(INVNT_WITH_MUJOCO "Enable MuJoCo-based simulator module" ON)
option(INVNT_BUILD_UNITREEA1 "Build Unitree A1 robot model" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)  # Eigen3::Eigen

# Subprojects
add_subdirectory(lib)

if (INVNT_BUILD_UNITREEA1)
  if (NOT INVNT_WITH_MUJOCO)
    message(FATAL_ERROR "INVNT_BUILD_UNITREEA1 requires INVNT_WITH_MUJOCO to be ON")
  endif()
  add_subdirectory(apps/a1)
endif()

if (INVNT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# Install public headers from top-level include
include(GNUInstallDirs)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export/Config to support find_package(iNVNt)
install(EXPORT iNVNtTargets
  NAMESPACE iNVNt::
  FILE iNVNtTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iNVNt)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/iNVNtConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/iNVNtConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iNVNt)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/iNVNtConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/iNVNtConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/iNVNtConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iNVNt)