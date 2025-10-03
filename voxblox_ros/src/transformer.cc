#include "voxblox_ros/transformer.h"

// #include <tf2_eigen/tf2_eigen.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace voxblox {

Transformer::Transformer(NodeHandle& nh, NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      sensor_frame_(""),
      use_tf_transforms_(true),
      timestamp_tolerance_ns_(1000000),
      tf_buffer_(std::make_unique<tf2_ros::Buffer>(nh->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
  nh_private_->get_parameter_or("world_frame", world_frame_, world_frame_);
  nh_private_->get_parameter_or("sensor_frame", sensor_frame_, sensor_frame_);

  const double kNanoSecondsInSecond = 1.0e9;
  double timestamp_tolerance_sec =
      timestamp_tolerance_ns_ / kNanoSecondsInSecond;
  nh_private_->get_parameter_or("timestamp_tolerance_sec", timestamp_tolerance_sec,
                    timestamp_tolerance_sec);
  timestamp_tolerance_ns_ =
      static_cast<int64_t>(timestamp_tolerance_sec * kNanoSecondsInSecond);

  // Transform settings.
  nh_private_->get_parameter_or("use_tf_transforms", use_tf_transforms_,
                    use_tf_transforms_);
  // If we use topic transforms, we have 2 parts: a dynamic transform from a
  // topic and a static transform from parameters.
  // Static transform should be T_G_D (where D is whatever sensor the
  // dynamic coordinate frame is in) and the static should be T_D_C (where
  // C is the sensor frame that produces the depth data). It is possible to
  // specify T_C_D and set invert_static_tranform to true.
  if (!use_tf_transforms_) {
    transform_sub_ = create_subscriber<TransformStampedMsg>(
      nh_, "transform", 40, 
      std::bind(&Transformer::transformCallback, this, std::placeholders::_1));
    // Retrieve T_D_C from params.
    // TODO(helenol): split out into a function to avoid duplication.
    TransformMsg T_B_D_msg;
    // if (nh_private_->get_parameter("T_B_D", T_B_D_msg)) {
    //   tf2::fromMsg(T_B_D_msg, T_B_D_);

    //   // See if we need to invert it.
    //   bool invert_static_tranform = false;
    //   nh_private_->get_parameter_or("invert_T_B_D", invert_static_tranform,
    //                     invert_static_tranform);
    //   if (invert_static_tranform) {
    //     T_B_D_ = T_B_D_.inverse();
    //   }
    // }
    std::vector<double> T_B_D_vector;
    if (nh_private_->get_parameter("T_B_D", T_B_D_vector)) {
      if (T_B_D_vector.size() == 7) {  // [x, y, z, qx, qy, qz, qw]
        geometry_msgs::msg::Transform T_B_D_msg;
        T_B_D_msg.translation.x = T_B_D_vector[0];
        T_B_D_msg.translation.y = T_B_D_vector[1];
        T_B_D_msg.translation.z = T_B_D_vector[2];
        T_B_D_msg.rotation.x = T_B_D_vector[3];
        T_B_D_msg.rotation.y = T_B_D_vector[4];
        T_B_D_msg.rotation.z = T_B_D_vector[5];
        T_B_D_msg.rotation.w = T_B_D_vector[6];
        
        // tf2::fromMsg(T_B_D_msg, T_B_D_);
        tf2::transformMsgToKindr(T_B_D_msg, &T_B_D_);
        
        bool invert_static_tranform = false;
        nh_private_->get_parameter_or("invert_T_B_D", invert_static_tranform,
                          invert_static_tranform);
        if (invert_static_tranform) {
          T_B_D_ = T_B_D_.inverse();
        }
      }
    }
    // TransformMsg T_B_C_msg;
    // if (nh_private_->get_parameter("T_B_C", T_B_C_msg)) {
    //   tf2::fromMsg(T_B_C_msg, T_B_C_);

    //   // See if we need to invert it.
    //   bool invert_static_tranform = false;
    //   nh_private_->get_parameter_or("invert_T_B_C", invert_static_tranform,
    //                     invert_static_tranform);
    //   if (invert_static_tranform) {
    //     T_B_C_ = T_B_C_.inverse();
    //   }
    // }
    std::vector<double> T_B_C_vector;
    if (nh_private_->get_parameter("T_B_C", T_B_C_vector)) {
      if (T_B_C_vector.size() == 7) {  // [x, y, z, qx, qy, qz, qw]
        geometry_msgs::msg::Transform T_B_C_msg;
        T_B_C_msg.translation.x = T_B_C_vector[0];
        T_B_C_msg.translation.y = T_B_C_vector[1];
        T_B_C_msg.translation.z = T_B_C_vector[2];
        T_B_C_msg.rotation.x = T_B_C_vector[3];
        T_B_C_msg.rotation.y = T_B_C_vector[4];
        T_B_C_msg.rotation.z = T_B_C_vector[5];
        T_B_C_msg.rotation.w = T_B_C_vector[6];
        
        // tf2::fromMsg(T_B_C_msg, T_B_C_);
        tf2::transformMsgToKindr(T_B_C_msg, &T_B_C_);
        
        bool invert_static_tranform = false;
        nh_private_->get_parameter_or("invert_T_B_C", invert_static_tranform,
                          invert_static_tranform);
        if (invert_static_tranform) {
          T_B_C_ = T_B_C_.inverse();
        }
      }
    }
  }
}

void Transformer::transformCallback(
    TransformStampedMsg::ConstSharedPtr transform_msg) {
  transform_queue_.push_back(*transform_msg);
}

bool Transformer::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const Time& timestamp,
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
                                    const Time& timestamp,
                                    Transformation* transform) {
  CHECK_NOTNULL(transform);
  geometry_msgs::msg::TransformStamped tf_transform;
  Time time_to_lookup = timestamp;

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
    tf_transform = tf_buffer_->lookupTransform(to_frame, from_frame_modified, time_to_lookup);
  } catch (const tf2::TransformException& ex) {  // NOLINT
    RCLCPP_ERROR_STREAM(nh_->get_logger(),
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  // tf2::fromMsg(tf_transform.transform, *transform);
  tf2::transformMsgToKindr(tf_transform.transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(const Time& timestamp,
                                       Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (transform_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 30000,
        "No match found for transform timestamp: " << timestamp.seconds() 
        << " as transform queue is empty.");
    return false;
  }
  // Try to match the transforms in the queue.
  bool match_found = false;
  std::deque<TransformStampedMsg>::iterator it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    // Convert ROS message timestamp to rclcpp::Time for comparison
    rclcpp::Time msg_time(it->header.stamp.sec, it->header.stamp.nanosec);
    
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (msg_time > timestamp) {
      if ((msg_time - timestamp).nanoseconds() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - msg_time).nanoseconds() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  // Match found basically means an exact match.
  Transformation T_G_D;
  if (match_found) {
    tf2::transformMsgToKindr(it->transform, &T_G_D);
  } else {
    // If we think we have an inexact match, have to check that we're still
    // within bounds and interpolate.
    if (it == transform_queue_.begin() || it == transform_queue_.end()) {
      rclcpp::Time front_time(transform_queue_.front().header.stamp.sec, 
                              transform_queue_.front().header.stamp.nanosec);
      rclcpp::Time back_time(transform_queue_.back().header.stamp.sec,
                             transform_queue_.back().header.stamp.nanosec);
      RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 30000,
          "No match found for transform timestamp: " << timestamp.seconds()
          << " Queue front: " << front_time.seconds()
          << " back: " << back_time.seconds());
      return false;
    }
    // Newest should be 1 past the requested timestamp, oldest should be one
    // before the requested timestamp.
    Transformation T_G_D_newest;
    tf2::transformMsgToKindr(it->transform, &T_G_D_newest);
    rclcpp::Time newest_time(it->header.stamp.sec, it->header.stamp.nanosec);
    int64_t offset_newest_ns = (newest_time - timestamp).nanoseconds();
    // We already checked that this is not the beginning.
    it--;
    Transformation T_G_D_oldest;
    tf2::transformMsgToKindr(it->transform, &T_G_D_oldest);
    rclcpp::Time oldest_time(it->header.stamp.sec, it->header.stamp.nanosec);
    int64_t offset_oldest_ns = (timestamp - oldest_time).nanoseconds();

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