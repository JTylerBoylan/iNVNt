add_executable(test_clock test_clock.cpp)
target_link_libraries(test_clock PUBLIC iNVNt)

add_executable(test_param test_param.cpp)
target_link_libraries(test_param PUBLIC iNVNt)

add_executable(test_dag test_dag.cpp)
target_link_libraries(test_dag PUBLIC iNVNt)

if (INVNT_BUILD_UNITREEA1)
    if (INVNT_WITH_MUJOCO)
        add_executable(test_mujoco test_mujoco.cpp)
        target_link_libraries(test_mujoco PUBLIC iNVNt)
    endif()
endif()