file(MAKE_DIRECTORY "${TEST_ROOT}/source")
file(WRITE "${TEST_ROOT}/source/CMakeLists.txt"
  "cmake_minimum_required(VERSION 3.16)\nproject(BackendCheck NONE)\n"
  "include(\"${SOURCE_ROOT}/cmake/GazeboBackend.cmake\")\nreal_gps_validate_backend()\n")

function(configure_backend directory distro expected_message)
  execute_process(COMMAND "${CMAKE_COMMAND}" -S "${TEST_ROOT}/source"
    -B "${TEST_ROOT}/${directory}" "-DGZ_DISTRO=${distro}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
  if(expected_message STREQUAL "")
    if(NOT result EQUAL 0)
      message(FATAL_ERROR "Valid backend ${distro} failed: ${output}${error}")
    endif()
  elseif(result EQUAL 0 OR NOT "${output}${error}" MATCHES "${expected_message}")
    message(FATAL_ERROR "Expected rejection '${expected_message}', got ${result}: ${output}${error}")
  endif()
endfunction()

# Start clean so the cache-reuse assertions remain valid on repeated CTest runs.
file(REMOVE_RECURSE "${TEST_ROOT}/harmonic" "${TEST_ROOT}/jetty" "${TEST_ROOT}/invalid")
configure_backend(harmonic harmonic "")
configure_backend(harmonic harmonic "")
configure_backend(jetty jetty "")
configure_backend(invalid unsupported "Unknown GZ_DISTRO")
configure_backend(harmonic jetty "Use separate build directories")
