FetchContent_Declare(doctest GIT_REPOSITORY https://github.com/doctest/doctest.git GIT_TAG v2.4.11)
FetchContent_MakeAvailable(doctest)
factory_sim_mark_system(doctest)

add_executable(factory_sim_tests
  test_main.cpp
  sim/world_test.cpp
  sim/sensors_test.cpp
  sim/scene_test.cpp
  sim/actuators_test.cpp
  sim/control_test.cpp
  render/scene_test.cpp
  render/orbit_camera_test.cpp
  edit/scene_history_test.cpp
)
target_link_libraries(factory_sim_tests PRIVATE factory_sim_view doctest::doctest fmt::fmt)
add_test(NAME factory_sim_tests COMMAND factory_sim_tests)

# The PLC link: soft_plc in lockstep, its monitoring protocol, and S7.
add_executable(factory_sim_plc_tests
  test_main.cpp
  plc/plc_test.cpp
)
target_link_libraries(factory_sim_plc_tests PRIVATE factory_sim_plc soft_plc_core doctest::doctest)
target_compile_definitions(factory_sim_plc_tests PRIVATE FACTORY_SIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_test(NAME factory_sim_plc_tests COMMAND factory_sim_plc_tests)
