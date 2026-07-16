cmake_minimum_required(VERSION 3.24)
project(FlightLoop VERSION 0.1.0 LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 5.0.0)
FetchContent_MakeAvailable(eigen)

add_library(flightloop src/integrator/integrator.cc)
target_compile_features(flightloop PUBLIC cxx_std_20)
target_include_directories(flightloop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(flightloop PUBLIC Eigen3::Eigen)
