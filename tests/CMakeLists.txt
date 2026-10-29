add_executable(real_gps_core_test Core_TEST.cc)
target_link_libraries(real_gps_core_test PRIVATE real_gps_core)
target_compile_options(real_gps_core_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME receiver_core COMMAND real_gps_core_test)

add_executable(real_gps_scenario_test Scenario_TEST.cc)
target_link_libraries(real_gps_scenario_test PRIVATE real_gps_core)
target_compile_options(real_gps_scenario_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME receiver_scenarios COMMAND real_gps_scenario_test)

add_test(NAME backend_validation COMMAND ${CMAKE_COMMAND}
  -DSOURCE_ROOT=${PROJECT_SOURCE_DIR}
  -DTEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/backend
  -P ${CMAKE_CURRENT_SOURCE_DIR}/Backend_TEST.cmake)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME python_helpers COMMAND ${Python3_EXECUTABLE} -m unittest discover
  -s ${CMAKE_CURRENT_SOURCE_DIR}/python -v)
set_tests_properties(python_helpers PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

if(BUILD_SIMULATOR)
  add_executable(real_gps_px4_metadata_test Px4Metadata_TEST.cc)
  target_link_libraries(real_gps_px4_metadata_test PRIVATE real_gps_core)
  target_compile_options(real_gps_px4_metadata_test PRIVATE -Wall -Wextra -Wpedantic)
  real_gps_configure_backend(real_gps_px4_metadata_test)
  add_test(NAME px4_metadata COMMAND real_gps_px4_metadata_test)

  add_executable(real_gps_transport_test Transport_TEST.cc)
  target_link_libraries(real_gps_transport_test PRIVATE real_gps_core)
  target_compile_options(real_gps_transport_test PRIVATE -Wall -Wextra -Wpedantic)
  real_gps_configure_backend(real_gps_transport_test)
  target_compile_definitions(real_gps_transport_test PRIVATE
    REAL_GPS_LIBRARY="$<TARGET_FILE:RealGpsSystem>")
  add_dependencies(real_gps_transport_test RealGpsSystem)
  add_test(NAME plugin_transport COMMAND real_gps_transport_test)
  set_tests_properties(plugin_transport PROPERTIES TIMEOUT 30)
endif()
