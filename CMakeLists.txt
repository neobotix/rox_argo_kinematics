cmake_minimum_required(VERSION 3.5)
project(rox_argo_kinematics)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(neo_srvs2 REQUIRED)
find_package(neo_msgs2 REQUIRED)
find_package(neo_common2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(rox_argo_kinematics_node src/rox_argo_kinematics.cpp)

ament_target_dependencies(rox_argo_kinematics_node
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  neo_msgs2
  neo_srvs2
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs
  neo_common2
  angles)

ament_export_include_directories(include)

install(DIRECTORY include/
   DESTINATION include/
)

install(TARGETS 
rox_argo_kinematics_node
DESTINATION lib/${PROJECT_NAME}
	)

install(DIRECTORY launch
  	
  DESTINATION share/${PROJECT_NAME})

ament_package()
