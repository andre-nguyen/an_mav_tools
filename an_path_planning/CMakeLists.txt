cmake_minimum_required(VERSION 2.8.3)
project(an_path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
add_subdirectory(rrtstar)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES an_path_planning
  CATKIN_DEPENDS roscpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  rrtstar/include
)

add_executable(rrt_node src/rrt_node.cpp)
add_dependencies(rrt_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_node
  ${catkin_LIBRARIES}
  rrtstar
)
