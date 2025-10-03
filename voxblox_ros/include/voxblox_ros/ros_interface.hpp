#pragma once

#if __has_include(<rclcpp/rclcpp.hpp>)
#include <rclcpp/rclcpp.hpp>

// ROS Messages
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <voxblox_msgs/msg/mesh.hpp>
#include <voxblox_msgs/msg/layer.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h>

// Services
// #include <std_srvs/srv/empty.hpp>
#include <voxblox_msgs/srv/file_path.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <minkindr_conversions/kindr_msg.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <voxblox_msgs/msg/layer.hpp>
#include <voxblox_msgs/msg/mesh.hpp>

// ROS2 type aliases
using NodeHandle = std::shared_ptr<rclcpp::Node>;
using Time = rclcpp::Time;
using Duration = rclcpp::Duration;

// Publisher type aliases
template<typename MessageType>
using Publisher = std::shared_ptr<rclcpp::Publisher<MessageType>>;

template<typename MessageType>
using Subscriber = std::shared_ptr<rclcpp::Subscription<MessageType>>;

template<typename ServiceType>
using Service = std::shared_ptr<rclcpp::Service<ServiceType>>;

using TimerBase = std::shared_ptr<rclcpp::TimerBase>;

// Message type aliases
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
using MarkerMsg = visualization_msgs::msg::Marker;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
using TransformMsg = geometry_msgs::msg::Transform;
using GeometryPointMsg = geometry_msgs::msg::Point;
using MeshMsg = voxblox_msgs::msg::Mesh;
using LayerMsg = voxblox_msgs::msg::Layer;
using ColorRGBAMsg = std_msgs::msg::ColorRGBA;
using MeshBlockMsg = voxblox_msgs::msg::MeshBlock;
using BlockMsg = voxblox_msgs::msg::Block;

using PointCloudPclMsg = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudRgbPclMsg = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudIPclMsg = pcl::PointCloud<pcl::PointXYZI>;

// Service type aliases
using EmptySrv = std_srvs::srv::Empty;
using FilePathSrv = voxblox_msgs::srv::FilePath;

// Ptr and ConstPtr were deprecated in ROS2. The naming is kept wrt to ROS2 but the
// using declarations here allow the same code to compile in both ROS1 and ROS2.
template<typename MessageType>
using SharedPtr = typename MessageType::SharedPtr;
template<typename MessageType>
using ConstSharedPtr = typename MessageType::ConstSharedPtr;

// Transform broadcasters
using TransformBroadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>;

// Logging macros
// #define LOG_INFO(...) RCLCPP_INFO(rclcpp::get_logger("voxblox"), __VA_ARGS__)
// #define LOG_WARN(...) RCLCPP_WARN(rclcpp::get_logger("voxblox"), __VA_ARGS__)
// #define LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("voxblox"), __VA_ARGS__)
// #define LOG_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("voxblox"), __VA_ARGS__)
// #define LOG_INFO_STREAM(stream) RCLCPP_INFO_STREAM(rclcpp::get_logger("voxblox"), stream)
// #define LOG_WARN_STREAM(stream) RCLCPP_WARN_STREAM(rclcpp::get_logger("voxblox"), stream)
// #define LOG_ERROR_STREAM(stream) RCLCPP_ERROR_STREAM(rclcpp::get_logger("voxblox"), stream)
// #define LOG_FATAL_STREAM(stream) RCLCPP_FATAL_STREAM(rclcpp::get_logger("voxblox"), stream)
// #define LOG_INFO_ONCE(...) RCLCPP_INFO_ONCE(rclcpp::get_logger("voxblox"), __VA_ARGS__)
// #define LOG_WARN_ONCE(...) RCLCPP_WARN_ONCE(rclcpp::get_logger("voxblox"), __VA_ARGS__)
// #define LOG_ERROR_ONCE(...) RCLCPP_ERROR_ONCE(rclcpp::get_logger("voxblox"), __VA_ARGS__)
// #define LOG_FATAL_ONCE(...) RCLCPP_FATAL_ONCE(rclcpp::get_logger("voxblox"), __VA_ARGS__)
// #define LOG_INFO_THROTTLE(rate, ...) RCLCPP_INFO_THROTTLE(rclcpp::get_logger("voxblox"), *rclcpp::Clock::make_shared(), rate, __VA_ARGS__)
// #define LOG_WARN_THROTTLE(rate, ...) RCLCPP_WARN_THROTTLE(rclcpp::get_logger("voxblox"), *rclcpp::Clock::make_shared(), rate, __VA_ARGS__)
// #define LOG_ERROR_THROTTLE(rate, ...) RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("voxblox"), *rclcpp::Clock::make_shared(), rate, __VA_ARGS__)
// #define LOG_FATAL_THROTTLE(rate, ...) RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("voxblox"), *rclcpp::Clock::make_shared(), rate, __VA_ARGS__)

// Helper functions
// template<typename MessageType>
// Publisher<MessageType> create_publisher(NodeHandle& nh, const std::string& topic, int queue_size, bool latch = false) {
//     rclcpp::QoS qos(queue_size);
//     if (latch) {
//         qos.transient_local();
//     }
//     return nh->create_publisher<MessageType>(topic, qos);
// }

template<typename MessageType>
Publisher<MessageType> create_publisher(NodeHandle& node, const std::string& topic, int queue_size, bool latch = false)
{
  rclcpp::QoS qos(queue_size);
  if (latch) {
    qos = qos.transient_local();
  }
  return node->template create_publisher<MessageType>(topic, qos);
}

// template<typename MessageType, typename... Args>
// Subscriber<MessageType> create_subscriber(NodeHandle& nh, const std::string& topic, int queue_size, Args&&... args) {
//     return nh->create_subscription<MessageType>(topic, queue_size, std::forward<Args>(args)...);
// }
template<typename MessageType>
Subscriber<MessageType> create_subscriber(NodeHandle& node, const std::string& topic, int queue_size,
                                        std::function<void(const std::shared_ptr<MessageType>)> callback)
{
  rclcpp::QoS qos(queue_size);
  return node->template create_subscription<MessageType>(topic, qos, callback);
}

template<typename PublisherType>
size_t get_subscription_count(const PublisherType& pub) {
    return pub->get_subscription_count();
}

inline Time timeNow(NodeHandle& node) { return node->now(); }

#else
#include <ros/ros.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/Layer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Services
#include <std_srvs/Empty.h>
#include <voxblox_msgs/FilePath.h>

// TF
#include <tf/transform_broadcaster.h>

#include <voxblox_msgs/Layer.h>
#include <voxblox_msgs/Mesh.h>

// ROS1 type aliases
using NodeHandle = ros::NodeHandle;
using Time = ros::Time;
using Duration = ros::Duration;

// Publisher type aliases
template<typename MessageType>
using Publisher = ros::Publisher;

template<typename MessageType>
using Subscriber = ros::Subscriber;

template<typename ServiceType>
using Service = ros::ServiceServer;

using TimerBase = ros::Timer;

// Message type aliases
using PointCloud2Msg = sensor_msgs::PointCloud2;
using MarkerArrayMsg = visualization_msgs::MarkerArray;
using TransformStampedMsg = geometry_msgs::TransformStamped;
using MeshMsg = voxblox_msgs::Mesh;
using LayerMsg = voxblox_msgs::Layer;
using MeshBlockMsg = voxblox_msgs::MeshBlock;
using BlockMsg = voxblox_msgs::Block;

using PointCloudPclMsg = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudRgbPclMsg = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudIPclMsg = pcl::PointCloud<pcl::PointXYZI>;

// Service type aliases
using EmptySrv = std_srvs::Empty;
using FilePathSrv = voxblox_msgs::FilePath;

// Ptr and ConstPtr are part of ROS1 messages.
template<typename MessageType>
using SharedPtr = typename MessageType::Ptr;
template<typename MessageType>
using ConstSharedPtr = typename MessageType::ConstPtr;

// Transform broadcasters
using TransformBroadcaster = tf::TransformBroadcaster;

// Logging macros
#define LOG_INFO(...) ROS_INFO(__VA_ARGS__)
#define LOG_WARN(...) ROS_WARN(__VA_ARGS__)
#define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define LOG_FATAL(...) ROS_FATAL(__VA_ARGS__)
#define LOG_INFO_STREAM(stream) ROS_INFO_STREAM(stream)
#define LOG_WARN_STREAM(stream) ROS_WARN_STREAM(stream)
#define LOG_ERROR_STREAM(stream) ROS_ERROR_STREAM(stream)
#define LOG_FATAL_STREAM(stream) ROS_FATAL_STREAM(stream)
#define LOG_INFO_ONCE(...) ROS_INFO_ONCE(__VA_ARGS__)
#define LOG_WARN_ONCE(...) ROS_WARN_ONCE(__VA_ARGS__)
#define LOG_ERROR_ONCE(...) ROS_ERROR_ONCE(__VA_ARGS__)
#define LOG_FATAL_ONCE(...) ROS_FATAL_ONCE(__VA_ARGS__)
#define LOG_INFO_THROTTLE(rate, ...) ROS_INFO_THROTTLE(rate, __VA_ARGS__)
#define LOG_WARN_THROTTLE(rate, ...) ROS_WARN_THROTTLE(rate, __VA_ARGS__)
#define LOG_ERROR_THROTTLE(rate, ...) ROS_ERROR_THROTTLE(rate, __VA_ARGS__)
#define LOG_FATAL_THROTTLE(rate, ...) ROS_FATAL_THROTTLE(rate, __VA_ARGS__)

// Helper functions
template<typename MessageType>
Publisher<MessageType> create_publisher(NodeHandle& nh, const std::string& topic, int queue_size, bool latch = false) {
    return nh.advertise<MessageType>(topic, queue_size, latch);
}

template<typename MessageType, typename... Args>
Subscriber<MessageType> create_subscriber(NodeHandle& nh, const std::string& topic, int queue_size, Args&&... args) {
    return nh.subscribe(topic, queue_size, std::forward<Args>(args)...);
}

template<typename PublisherType>
size_t get_subscription_count(const PublisherType& pub) {
    return pub.getNumSubscribers();
}

inline Time timeNow(NodeHandle& nh) { return ros::Time::now(); }

#endif
