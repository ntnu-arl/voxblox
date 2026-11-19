#include "voxblox_ros/transformer.h"

namespace voxblox {

Transformer::Transformer(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      world_frame_("world"),
      sensor_frame_(""),
      use_tf_transforms_(true),
      timestamp_tolerance_ns_(1000000) {
  world_frame_ = ros1_compat::param<std::string>("world_frame", world_frame_);
  sensor_frame_ = ros1_compat::param<std::string>("sensor_frame", sensor_frame_);

  const double kNanoSecondsInSecond = 1.0e9;
  double timestamp_tolerance_sec =
      timestamp_tolerance_ns_ / kNanoSecondsInSecond;
  timestamp_tolerance_sec =
      ros1_compat::param<double>("timestamp_tolerance_sec", timestamp_tolerance_sec);
  timestamp_tolerance_ns_ =
      static_cast<int64_t>(timestamp_tolerance_sec * kNanoSecondsInSecond);

  use_tf_transforms_ = ros1_compat::param<bool>("use_tf_transforms", use_tf_transforms_);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  if (!use_tf_transforms_) {
    transform_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
        "transform", 40,
        std::bind(&Transformer::transformCallback, this, std::placeholders::_1));
  }
}

void Transformer::transformCallback(
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg) {
  transform_queue_.push_back(*transform_msg);
}

bool Transformer::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const ros::Time& timestamp,
                                  Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(timestamp, transform);
  }
}

// Stolen from octomap_manager
bool Transformer::lookupTransformTf(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const ros::Time& timestamp,
                                    Transformation* transform) {
  CHECK_NOTNULL(transform);
  geometry_msgs::msg::TransformStamped tf_transform;
  const rclcpp::Time time_to_lookup = timestamp;

  // Allow overwriting the TF frame for the sensor.
  std::string from_frame_modified = from_frame;
  if (!sensor_frame_.empty()) {
    from_frame_modified = sensor_frame_;
  }

  // Previous behavior was just to use the latest transform if the time is in
  // the future. Now we will just wait.
  if (!tf_buffer_->canTransform(to_frame, from_frame_modified,
                                 time_to_lookup)) {
    return false;
  }

  try {
    tf_transform = tf_buffer_->lookupTransform(to_frame, from_frame_modified,
                                               time_to_lookup);
  } catch (tf2::TransformException& ex) {  // NOLINT
    ROS_ERROR_STREAM(
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  transformMsgToKindr(tf_transform.transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(const ros::Time& timestamp,
                                       Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (transform_queue_.empty()) {
    ROS_WARN_THROTTLE(30, "No match found for transform timestamp %.3f as transform queue is empty.",
                      timestamp.toSec());
    return false;
  }
  // Try to match the transforms in the queue.
  bool match_found = false;
  auto it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (ros::Time(it->header.stamp) > timestamp) {
      if ((ros::Time(it->header.stamp) - timestamp).toNSec() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - ros::Time(it->header.stamp)).toNSec() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  // Match found basically means an exact match.
  Transformation T_G_D;
  if (match_found) {
    transformMsgToKindr(it->transform, &T_G_D);
  } else {
    // If we think we have an inexact match, have to check that we're still
    // within bounds and interpolate.
    if (it == transform_queue_.begin() || it == transform_queue_.end()) {
      ROS_WARN_THROTTLE(30, "No match found for transform timestamp %.3f", timestamp.toSec());
      return false;
    }
    // Newest should be 1 past the requested timestamp, oldest should be one
    // before the requested timestamp.
    Transformation T_G_D_newest;
    transformMsgToKindr(it->transform, &T_G_D_newest);
    int64_t offset_newest_ns = (ros::Time(it->header.stamp) - timestamp).toNSec();
    // We already checked that this is not the beginning.
    it--;
    Transformation T_G_D_oldest;
    transformMsgToKindr(it->transform, &T_G_D_oldest);
    int64_t offset_oldest_ns = (timestamp - ros::Time(it->header.stamp)).toNSec();

    // Interpolate between the two transformations using the exponential map.
    FloatingPoint t_diff_ratio =
        static_cast<FloatingPoint>(offset_oldest_ns) /
        static_cast<FloatingPoint>(offset_newest_ns + offset_oldest_ns);

    Transformation::Vector6 diff_vector =
        (T_G_D_oldest.inverse() * T_G_D_newest).log();
    T_G_D = T_G_D_oldest * Transformation::exp(t_diff_ratio * diff_vector);
  }

  // If we have a static transform, apply it too.
  // Transform should actually be T_G_C. So need to take it through the full
  // chain.
  *transform = T_G_D * T_B_D_.inverse() * T_B_C_;

  // And also clear the queue up to this point. This leaves the current
  // message in place.
  transform_queue_.erase(transform_queue_.begin(), it);
  return true;
}

}  // namespace voxblox
