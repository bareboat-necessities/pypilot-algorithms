cmake_minimum_required(VERSION 3.16)
project(pypilot_algorithms LANGUAGES CXX)

option(PYPILOT_ALGORITHMS_BUILD_TESTS "Build tests" ON)
option(PYPILOT_ALGORITHMS_BUILD_EXAMPLES "Build examples" ON)

add_library(pypilot_algorithms INTERFACE)
target_include_directories(pypilot_algorithms INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(pypilot_algorithms INTERFACE cxx_std_11)

if(PYPILOT_ALGORITHMS_BUILD_EXAMPLES)
  add_executable(compute_basic_pilot examples/linux/compute_basic_pilot.cpp)
  target_link_libraries(compute_basic_pilot PRIVATE pypilot_algorithms)
endif()

if(PYPILOT_ALGORITHMS_BUILD_TESTS)
  enable_testing()
  foreach(test_name test_angles test_filters test_basic_pilot test_wind_pilot
      test_more_pilots)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE pypilot_algorithms)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
