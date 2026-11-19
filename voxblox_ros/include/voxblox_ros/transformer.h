#ifndef VOXBLOX_ROS_TRANSFORMER_H_
#define VOXBLOX_ROS_TRANSFORMER_H_

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros1_compat/ros1_compat.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <voxblox/core/common.h>

namespace voxblox {

/// geometry_msgs Transform -> kindr (replaces minkindr_conversions on ROS 2).
inline void transformMsgToKindr(const geometry_msgs::msg::Transform& msg,
                                Transformation* kindr) {
  const Rotation rotation(Eigen::Quaternion<FloatingPoint>(
      msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z));
  const Point position(msg.translation.x, msg.translation.y, msg.translation.z);
  *kindr = Transformation(rotation, position);
}

/// kindr -> geometry_msgs Transform.
inline void transformKindrToMsg(const kindr::minimal::QuatTransformationTemplate<double>& kindr,
                                geometry_msgs::msg::Transform* msg) {
  const Eigen::Vector3d p = kindr.getPosition();
  const Eigen::Quaterniond q = kindr.getEigenQuaternion();
  msg->translation.x = p.x(); msg->translation.y = p.y(); msg->translation.z = p.z();
  msg->rotation.x = q.x(); msg->rotation.y = q.y(); msg->rotation.z = q.z(); msg->rotation.w = q.w();
}

/**
 * Class that binds to either the TF tree or resolves transformations from a
 * transform topic, depending on settings loaded from ROS params.
 * (ROS 2 port: the static T_B_D / T_B_C XmlRpc parameters are not supported.)
 */
class Transformer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit Transformer(const rclcpp::Node::SharedPtr& node);

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);

  void transformCallback(
      const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg);

 private:
  bool lookupTransformTf(const std::string& from_frame,
                         const std::string& to_frame,
                         const ros::Time& timestamp, Transformation* transform);

  bool lookupTransformQueue(const ros::Time& timestamp,
                            Transformation* transform);

  rclcpp::Node::SharedPtr node_;
  std::string world_frame_;
  std::string sensor_frame_;
  bool use_tf_transforms_;
  int64_t timestamp_tolerance_ns_;
  Transformation T_B_C_;
  Transformation T_B_D_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      transform_sub_;
  AlignedDeque<geometry_msgs::msg::TransformStamped> transform_queue_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TRANSFORMER_H_
