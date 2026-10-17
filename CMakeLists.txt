cmake_minimum_required(VERSION 3.16)
project(gz_real_gps_plugin VERSION 0.1.1 LANGUAGES CXX)

include(GNUInstallDirs)
include(cmake/GazeboBackend.cmake)
real_gps_validate_backend()

add_library(RealGpsSystem SHARED
  src/adapters/gzsim/RealGpsSystem.cc
  src/core/Receiver.cc
  src/core/Almanac.cc
  src/core/Geometry.cc
)
target_compile_features(RealGpsSystem PRIVATE cxx_std_17)
target_include_directories(RealGpsSystem PRIVATE include)
target_compile_options(RealGpsSystem PRIVATE -Wall -Wextra -Wpedantic)
real_gps_configure_backend(RealGpsSystem)

install(TARGETS RealGpsSystem LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
