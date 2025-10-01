cmake_minimum_required(VERSION 3.5)
project(voxblox_msgs)

if(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 2)
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
else()
  cmake_minimum_required(VERSION 3.0.2)

  find_package(catkin_simple REQUIRED)
  find_package(std_msgs REQUIRED)

  catkin_simple(ALL_DEPS_REQUIRED)

  add_message_files(
    DIRECTORY msg
    FILES
    Block.msg
    Layer.msg
    Mesh.msg
    MeshBlock.msg
    VoxelEvaluationDetails.msg
  )

  add_service_files(
    DIRECTORY srv
    FILES
    FilePath.srv
  )

  generate_messages(DEPENDENCIES std_msgs)

  ##########
  # EXPORT #
  ##########

  cs_install()
  cs_export()
endif()
