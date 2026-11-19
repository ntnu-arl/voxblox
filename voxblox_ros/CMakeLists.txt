cmake_minimum_required(VERSION 3.8)
project(voxblox_ros)
# ROS 2 port of the NTNU voxblox_ros fork: only the servers GBPlanner links against
# (TSDF/ESDF server, transformer). Demo executables are not ported.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros1_compat REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(minkindr REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/esdf_server.cc
  src/transformer.cc
  src/tsdf_server.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} glog gflags)
ament_target_dependencies(${PROJECT_NAME} rclcpp ros1_compat voxblox voxblox_msgs minkindr sensor_msgs
  std_msgs std_srvs geometry_msgs visualization_msgs tf2 tf2_ros tf2_eigen pcl_conversions Eigen3)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp ros1_compat voxblox voxblox_msgs minkindr sensor_msgs std_msgs std_srvs
  geometry_msgs visualization_msgs tf2 tf2_ros tf2_eigen pcl_conversions PCL Eigen3)
ament_package()
