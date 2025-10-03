cmake_minimum_required(VERSION 3.5)
project(voxblox_msgs)

# Default to ROS 2
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Block.msg"
  "msg/Layer.msg"
  "msg/Mesh.msg"
  "msg/MeshBlock.msg"
  "msg/VoxelEvaluationDetails.msg"
  "srv/FilePath.srv"
  DEPENDENCIES std_msgs
)

ament_package()