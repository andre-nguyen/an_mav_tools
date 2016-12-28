cmake_minimum_required (VERSION 2.8.11)
project (rrtstar)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/rrtstar)
add_library(
  rrtstar
  src/kdtree.c
  src/system_single_integrator.cpp
)
