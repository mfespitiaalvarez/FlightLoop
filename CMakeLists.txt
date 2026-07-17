cmake_minimum_required(VERSION 3.24)
project(FlightLoop VERSION 0.1.0 LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 5.0.0)
FetchContent_MakeAvailable(eigen)

add_library(flightloop
    src/integrator/integrator.cc
    src/integrator/rk4_integrator.cc)
target_compile_features(flightloop PUBLIC cxx_std_20)
target_include_directories(flightloop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(flightloop PUBLIC Eigen3::Eigen)

include(CTest)
if(BUILD_TESTING)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0)
  FetchContent_MakeAvailable(googletest)

  add_executable(flightloop_tests
      tests/unit/frames/rotation_test.cc
      tests/unit/integrator/rk4_integrator_test.cc)
  target_link_libraries(flightloop_tests PRIVATE flightloop GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(flightloop_tests)
endif()
