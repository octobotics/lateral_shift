cmake_minimum_required(VERSION 2.8.3)
project(lateral_shift)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  hector_mapping
  geometry_msgs
  behaviortree_cpp_v3  # Add the appropriate package if not found
)

catkin_package(
  CATKIN_DEPENDS roscpp std_srvs zed_interfaces geometry_msgs behaviortree_cpp_v3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lateral_shift_controller src/main.cpp)

target_link_libraries(lateral_shift_controller
  ${catkin_LIBRARIES}
)

install(TARGETS lateral_shift_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)