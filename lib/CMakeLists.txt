add_library(iNVNt INTERFACE)

# Generate config.hpp
configure_file(
    ${CMAKE_SOURCE_DIR}/include/iNVNt/core/config.hpp.in
    ${INVNT_GEN_INC}/iNVNt/core/config.hpp @ONLY
)

target_include_directories(iNVNt
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${INVNT_GEN_INC}>
    $<INSTALL_INTERFACE:include>
)

find_package(Eigen3 REQUIRED)

target_link_libraries(iNVNt INTERFACE Eigen3::Eigen)
target_compile_features(iNVNt INTERFACE cxx_std_20)

if (INVNT_WITH_OSQP)
  find_package(osqp REQUIRED)
  target_link_libraries(iNVNt INTERFACE osqp::osqp)
endif()

if (INVNT_WITH_MUJOCO)
  find_package(glfw3 REQUIRED)
  find_package(mujoco REQUIRED)
  target_link_libraries(iNVNt INTERFACE glfw mujoco::mujoco)
endif()

install(TARGETS iNVNt EXPORT iNVNtTargets)