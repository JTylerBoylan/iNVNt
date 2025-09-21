project(a1)

include_directories(include)

add_executable(unitreea1_mujoco main.cpp)
target_link_libraries(unitreea1_mujoco PRIVATE iNVNt)
install(TARGETS unitreea1_mujoco RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(test_pid_control tests/test_pid_control.cpp)
target_link_libraries(test_pid_control PRIVATE iNVNt)

add_executable(test_ik_control tests/test_ik_control.cpp)
target_link_libraries(test_ik_control PRIVATE iNVNt)

add_executable(test_osc_control tests/test_osc_control.cpp)
target_link_libraries(test_osc_control PRIVATE iNVNt)

# Copy mujoco model into share directory
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/model/"
     DESTINATION "${INVNT_SHARE_DIR}/${PROJECT_NAME}/model/")
