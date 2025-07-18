cmake_minimum_required(VERSION 2.8.3)
project(rox_argo_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  neo_msgs
  neo_srvs
  tf
  tf2_ros
  nav_msgs
  angles
  neo_common
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rox_argo_kinematics_node src/rox_argo_kinematics.cpp)

target_link_libraries(rox_argo_kinematics_node
  ${catkin_LIBRARIES}
)

install(TARGETS rox_argo_kinematics_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES LICENSE README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 