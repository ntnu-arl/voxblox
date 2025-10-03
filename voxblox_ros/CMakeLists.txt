cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

message(STATUS "Building with ROS2")

find_package(ament_cmake REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(minkindr_conversions REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra)

add_executable(tsdf_server
  src/tsdf_server_node.cc
  src/tsdf_server.cc
  src/transformer.cc
)

target_include_directories(tsdf_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(tsdf_server
  voxblox::voxblox
  minkindr_conversions::minkindr_conversions
  gflags
)

ament_target_dependencies(tsdf_server
  rclcpp
  voxblox_msgs
  tf2_ros
  tf2_eigen
  pcl_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  image_transport
  std_srvs
  minkindr_conversions
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS tsdf_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()