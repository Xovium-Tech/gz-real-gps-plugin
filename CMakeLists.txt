cmake_minimum_required(VERSION 3.16)
project(gz_real_gps_plugin VERSION 0.2.0 LANGUAGES CXX)

include(GNUInstallDirs)
option(BUILD_SIMULATOR "Build the Gazebo system plugin" ON)

add_library(real_gps_core STATIC
  src/core/Receiver.cc
  src/core/Scenario.cc
  src/core/Almanac.cc
  src/core/Geometry.cc
)
target_compile_features(real_gps_core PUBLIC cxx_std_17)
target_include_directories(real_gps_core PUBLIC include)
target_compile_options(real_gps_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(real_gps_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(BUILD_SIMULATOR)
  include(cmake/GazeboBackend.cmake)
  real_gps_validate_backend()
  add_library(RealGpsSystem SHARED src/adapters/gzsim/RealGpsSystem.cc)
  target_link_libraries(RealGpsSystem PRIVATE real_gps_core)
  target_compile_options(RealGpsSystem PRIVATE -Wall -Wextra -Wpedantic)
  real_gps_configure_backend(RealGpsSystem)
  install(TARGETS RealGpsSystem LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

# Minimal source releases exclude development tests and must build by default.
set(real_gps_tests_available OFF)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
  set(real_gps_tests_available ON)
endif()
option(BUILD_TESTING "Build regression and integration tests" ${real_gps_tests_available})
include(CTest)
if(BUILD_TESTING)
  if(NOT real_gps_tests_available)
    message(FATAL_ERROR "BUILD_TESTING requires the tests directory from a repository checkout")
  endif()
  add_subdirectory(tests)
endif()
