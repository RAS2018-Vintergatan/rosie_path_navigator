cmake_minimum_required(VERSION 2.8.3)
project(rosie_path_navigator)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp geometry_msgs nav_msgs tf)

add_service_files(
  FILES
  PathNavigationService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp message_runtime geometry_msgs nav_msgs tf
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_path_navigator_node src/path_navigator_node.cpp)
target_link_libraries(rosie_path_navigator_node ${catkin_LIBRARIES})
add_dependencies(rosie_path_navigator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
