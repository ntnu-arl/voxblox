// #include "voxblox_ros/tsdf_server.h"
// #include "voxblox_ros/ros_interface.hpp"

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   google::InitGoogleLogging(argv[0]);
//   google::ParseCommandLineFlags(&argc, &argv, false);
//   google::InstallFailureSignalHandler();
//   auto nh = std::make_shared<rclcpp::Node>("voxblox");
//   auto nh_private = std::make_shared<rclcpp::Node>("voxblox_private");

//   voxblox::TsdfServer node(nh, nh_private);

//   rclcpp::spin(nh);
//   return 0;
// }


#include "voxblox_ros/tsdf_server.h"
#include "voxblox_ros/ros_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  
  // Use single node with private namespace
  auto nh = std::make_shared<rclcpp::Node>("voxblox_node");

  voxblox::TsdfServer node(nh, nh);  // Pass same node for both

  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}