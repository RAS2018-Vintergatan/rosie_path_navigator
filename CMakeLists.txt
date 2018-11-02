cmake_minimum_required(VERSION 2.8.3)
project(rosie_path_navigator)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs tf)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_path_navigator src/path_navigator_node.cpp)
target_link_libraries(rosie_path_navigator ${catkin_LIBRARIES})
add_dependencies(rosie_path_navigator geometry_msgs nav_msgs tf)
