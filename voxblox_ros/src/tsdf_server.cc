#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

TsdfServer::TsdfServer(const rclcpp::Node::SharedPtr& node)
    : TsdfServer(node, getTsdfMapConfigFromRosParam(),
                 getTsdfIntegratorConfigFromRosParam(),
                 getMeshIntegratorConfigFromRosParam()) {}

TsdfServer::TsdfServer(const rclcpp::Node::SharedPtr& node,
                       const TsdfMap::Config& config,
                       const TsdfIntegratorBase::Config& integrator_config,
                       const MeshIntegratorConfig& mesh_config)
    : node_(node),
      verbose_(true),
      world_frame_("world"),
      icp_corrected_frame_("icp_corrected"),
      pose_corrected_frame_("pose_corrected"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      use_freespace_pointcloud_(false),
      color_map_(new RainbowColorMap()),
      publish_pointclouds_on_update_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      cache_mesh_(false),
      enable_icp_(false),
      accumulate_icp_corrections_(true),
      occupancy_min_distance_voxel_size_factor_(1.0),
      pointcloud_queue_size_(1),
      num_subscribers_tsdf_map_(0),
      transformer_(node) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
  getServerConfigFromRosParam();

  // Advertise topics.
  surface_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/surface_pointcloud", rclcpp::QoS(1).transient_local());
  tsdf_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/tsdf_pointcloud", rclcpp::QoS(1).transient_local());
  occupancy_marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/occupied_nodes", rclcpp::QoS(1).transient_local());
  tsdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/tsdf_slice", rclcpp::QoS(1).transient_local());

  pointcloud_queue_size_ = ros1_compat::param<std::decay_t<decltype(pointcloud_queue_size_)>>("pointcloud_queue_size", pointcloud_queue_size_);
  // Sensor-data QoS (best effort) accepts both reliable and best-effort publishers.
  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", rclcpp::SensorDataQoS().keep_last(pointcloud_queue_size_),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { insertPointcloud(m); });

  mesh_pub_ = node_->create_publisher<voxblox_msgs::msg::Mesh>("~/mesh", rclcpp::QoS(1).transient_local());

  // Publishing/subscribing to a layer from another node (when using this as
  // a library, for example within a planner).
  tsdf_map_pub_ =
      node_->create_publisher<voxblox_msgs::msg::Layer>("~/tsdf_map_out", rclcpp::QoS(1));
  tsdf_map_sub_ = node_->create_subscription<voxblox_msgs::msg::Layer>(
      "~/tsdf_map_in", 1,
      std::bind(&TsdfServer::tsdfMapCallback, this, std::placeholders::_1));
  publish_tsdf_map_ = ros1_compat::param<std::decay_t<decltype(publish_tsdf_map_)>>("publish_tsdf_map", publish_tsdf_map_);

  if (use_freespace_pointcloud_) {
    // points that are not inside an object, but may also not be on a surface.
    // These will only be used to mark freespace beyond the truncation distance.
    freespace_pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "freespace_pointcloud", rclcpp::SensorDataQoS().keep_last(pointcloud_queue_size_),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { insertFreespacePointcloud(m); });
  }

  if (enable_icp_) {
    icp_transform_pub_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>("~/icp_transform", rclcpp::QoS(1).transient_local());
    icp_corrected_frame_ = ros1_compat::param<std::decay_t<decltype(icp_corrected_frame_)>>("icp_corrected_frame", icp_corrected_frame_);
    pose_corrected_frame_ = ros1_compat::param<std::decay_t<decltype(pose_corrected_frame_)>>("pose_corrected_frame", pose_corrected_frame_);
  }

  // Initialize TSDF Map and integrator.
  tsdf_map_.reset(new TsdfMap(config));

  std::string method("merged");
  method = ros1_compat::param<std::decay_t<decltype(method)>>("method", method);
  if (method.compare("simple") == 0) {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("merged") == 0) {
    tsdf_integrator_.reset(new MergedTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("fast") == 0) {
    tsdf_integrator_.reset(new FastTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else {
    tsdf_integrator_.reset(new SimpleTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  }

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));

  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  icp_.reset(new ICP(getICPConfigFromRosParam()));

  // Advertise services.
  generate_mesh_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/generate_mesh", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { generateMeshCallback(req, res); });
  clear_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/clear_map", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { clearMapCallback(req, res); });
  save_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "~/save_map", [this](const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> req, std::shared_ptr<voxblox_msgs::srv::FilePath::Response> res) { saveMapCallback(req, res); });
  load_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "~/load_map", [this](const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> req, std::shared_ptr<voxblox_msgs::srv::FilePath::Response> res) { loadMapCallback(req, res); });
  publish_pointclouds_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/publish_pointclouds", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { publishPointcloudsCallback(req, res); });
  publish_tsdf_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/publish_map", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) { publishTsdfMapCallback(req, res); });

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0;
  update_mesh_every_n_sec = ros1_compat::param<std::decay_t<decltype(update_mesh_every_n_sec)>>("update_mesh_every_n_sec", update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ =
        rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(update_mesh_every_n_sec), std::bind(&TsdfServer::updateMeshEvent, this));
  }

  double publish_map_every_n_sec = 1.0;
  publish_map_every_n_sec = ros1_compat::param<std::decay_t<decltype(publish_map_every_n_sec)>>("publish_map_every_n_sec", publish_map_every_n_sec);

  if (publish_map_every_n_sec > 0.0) {
    publish_map_timer_ =
        rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(publish_map_every_n_sec), std::bind(&TsdfServer::publishMapEvent, this));
  }
}

void TsdfServer::getServerConfigFromRosParam() {
  // Before subscribing, determine minimum time between messages.
  // 0 by default.
  double min_time_between_msgs_sec = 0.0;
  min_time_between_msgs_sec = ros1_compat::param<std::decay_t<decltype(min_time_between_msgs_sec)>>("min_time_between_msgs_sec", min_time_between_msgs_sec);
  min_time_between_msgs_.fromSec(min_time_between_msgs_sec);

  max_block_distance_from_body_ = ros1_compat::param<std::decay_t<decltype(max_block_distance_from_body_)>>("max_block_distance_from_body", max_block_distance_from_body_);
  slice_level_ = ros1_compat::param<std::decay_t<decltype(slice_level_)>>("slice_level", slice_level_);
  world_frame_ = ros1_compat::param<std::decay_t<decltype(world_frame_)>>("world_frame", world_frame_);
  publish_pointclouds_on_update_ = ros1_compat::param<std::decay_t<decltype(publish_pointclouds_on_update_)>>("publish_pointclouds_on_update", publish_pointclouds_on_update_);
  publish_slices_ = ros1_compat::param<std::decay_t<decltype(publish_slices_)>>("publish_slices", publish_slices_);
  publish_pointclouds_ = ros1_compat::param<std::decay_t<decltype(publish_pointclouds_)>>("publish_pointclouds", publish_pointclouds_);

  use_freespace_pointcloud_ = ros1_compat::param<std::decay_t<decltype(use_freespace_pointcloud_)>>("use_freespace_pointcloud", use_freespace_pointcloud_);
  pointcloud_queue_size_ = ros1_compat::param<std::decay_t<decltype(pointcloud_queue_size_)>>("pointcloud_queue_size", pointcloud_queue_size_);
  enable_icp_ = ros1_compat::param<std::decay_t<decltype(enable_icp_)>>("enable_icp", enable_icp_);
  accumulate_icp_corrections_ = ros1_compat::param<std::decay_t<decltype(accumulate_icp_corrections_)>>("accumulate_icp_corrections", accumulate_icp_corrections_);

  occupancy_min_distance_voxel_size_factor_ = ros1_compat::param<std::decay_t<decltype(occupancy_min_distance_voxel_size_factor_)>>("occupancy_min_distance_voxel_size_factor", occupancy_min_distance_voxel_size_factor_);

  verbose_ = ros1_compat::param<std::decay_t<decltype(verbose_)>>("verbose", verbose_);

  // Mesh settings.
  mesh_filename_ = ros1_compat::param<std::decay_t<decltype(mesh_filename_)>>("mesh_filename", mesh_filename_);
  std::string color_mode("");
  color_mode = ros1_compat::param<std::decay_t<decltype(color_mode)>>("color_mode", color_mode);
  color_mode_ = getColorModeFromString(color_mode);

  // Color map for intensity pointclouds.
  std::string intensity_colormap("rainbow");
  float intensity_max_value = kDefaultMaxIntensity;
  intensity_colormap = ros1_compat::param<std::decay_t<decltype(intensity_colormap)>>("intensity_colormap", intensity_colormap);
  intensity_max_value = ros1_compat::param<std::decay_t<decltype(intensity_max_value)>>("intensity_max_value", intensity_max_value);

  // Default set in constructor.
  if (intensity_colormap == "rainbow") {
    color_map_.reset(new RainbowColorMap());
  } else if (intensity_colormap == "inverse_rainbow") {
    color_map_.reset(new InverseRainbowColorMap());
  } else if (intensity_colormap == "grayscale") {
    color_map_.reset(new GrayscaleColorMap());
  } else if (intensity_colormap == "inverse_grayscale") {
    color_map_.reset(new InverseGrayscaleColorMap());
  } else if (intensity_colormap == "ironbow") {
    color_map_.reset(new IronbowColorMap());
  } else {
    ROS_ERROR_STREAM("Invalid color map: " << intensity_colormap);
  }
  color_map_->setMaxValue(intensity_max_value);
}

void TsdfServer::processPointCloudMessageAndInsert(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  // Convert the PCL pointcloud into our awesome format.

  // Horrible hack fix to fix color parsing colors in PCL.
  bool color_pointcloud = false;
  bool has_intensity = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    }
  }

  Pointcloud points_C;
  Colors colors;
  timing::Timer ptcloud_timer("ptcloud_preprocess");

  // Convert differently depending on RGB or I type.
  if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  }
  ptcloud_timer.Stop();

  Transformation T_G_C_refined = T_G_C;
  if (enable_icp_) {
    timing::Timer icp_timer("icp");
    if (!accumulate_icp_corrections_) {
      icp_corrected_transform_.setIdentity();
    }
    static Transformation T_offset;
    const size_t num_icp_updates =
        icp_->runICP(tsdf_map_->getTsdfLayer(), points_C,
                     icp_corrected_transform_ * T_G_C, &T_G_C_refined);
    if (verbose_) {
      ROS_INFO("ICP refinement performed %zu successful update steps",
               num_icp_updates);
    }
    icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

    if (!icp_->refiningRollPitch()) {
      // its already removed internally but small floating point errors can
      // build up if accumulating transforms
      Transformation::Vector6 T_vec = icp_corrected_transform_.log();
      T_vec[3] = 0.0;
      T_vec[4] = 0.0;
      icp_corrected_transform_ = Transformation::exp(T_vec);
    }

    // Publish transforms as both TF and message.
    geometry_msgs::msg::TransformStamped icp_tf_msg, pose_tf_msg, transform_msg;
    transformKindrToMsg(icp_corrected_transform_.cast<double>(), &icp_tf_msg.transform);
    transformKindrToMsg(T_G_C.cast<double>(), &pose_tf_msg.transform);
    transform_msg.transform = icp_tf_msg.transform;
    icp_tf_msg.header.stamp = pointcloud_msg->header.stamp;
    icp_tf_msg.header.frame_id = world_frame_;
    icp_tf_msg.child_frame_id = icp_corrected_frame_;
    pose_tf_msg.header.stamp = pointcloud_msg->header.stamp;
    pose_tf_msg.header.frame_id = icp_corrected_frame_;
    pose_tf_msg.child_frame_id = pose_corrected_frame_;
    tf_broadcaster_->sendTransform(icp_tf_msg);
    tf_broadcaster_->sendTransform(pose_tf_msg);

    transform_msg.header.frame_id = world_frame_;
    transform_msg.child_frame_id = icp_corrected_frame_;
    icp_transform_pub_->publish(transform_msg);

    icp_timer.Stop();
  }

  if (verbose_) {
    ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
  }

  const auto start = std::chrono::steady_clock::now();
  integratePointcloud(T_G_C_refined, points_C, colors, is_freespace_pointcloud);
  const auto end = std::chrono::steady_clock::now();
  if (verbose_) {
    ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
             std::chrono::duration<double>(end - start).count(),
             tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(T_G_C.getPosition(),
                                max_block_distance_from_body_);
  block_remove_timer.Stop();

  // Callback for inheriting classes.
  newPoseCallback(T_G_C);
}

// Checks if we can get the next message from queue.
bool TsdfServer::getNextPointcloudFromQueue(
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
    sensor_msgs::msg::PointCloud2::SharedPtr* pointcloud_msg, Transformation* T_G_C) {
  const size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();
  if (transformer_.lookupTransform((*pointcloud_msg)->header.frame_id,
                                   world_frame_,
                                   (*pointcloud_msg)->header.stamp, T_G_C)) {
    queue->pop();
    return true;
  } else {
    if (queue->size() >= kMaxQueueSize) {
      ROS_ERROR_THROTTLE(60,
                         "Input pointcloud queue getting too long! Dropping "
                         "some pointclouds. Either unable to look up transform "
                         "timestamps or the processing is taking too long.");
      while (queue->size() >= kMaxQueueSize) {
        queue->pop();
      }
    }
  }
  return false;
}

void TsdfServer::insertPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg_in) {
  if (ros::Time(pointcloud_msg_in->header.stamp) - last_msg_time_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  bool processed_any = false;
  while (
      getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = false;
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);
    processed_any = true;
  }

  if (!processed_any) {
    return;
  }

  if (publish_pointclouds_on_update_) {
    publishPointclouds();
  }

  if (verbose_) {
    ROS_INFO_STREAM("Timings: " << std::endl << timing::Timing::Print());
    ROS_INFO_STREAM(
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  }
}

void TsdfServer::insertFreespacePointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg_in) {
  if (ros::Time(pointcloud_msg_in->header.stamp) - last_msg_time_freespace_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_freespace_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    freespace_pointcloud_queue_.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  while (getNextPointcloudFromQueue(&freespace_pointcloud_queue_,
                                    &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = true;
    processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                      is_freespace_pointcloud);
  }
}

void TsdfServer::integratePointcloud(const Transformation& T_G_C,
                                     const Pointcloud& ptcloud_C,
                                     const Colors& colors,
                                     const bool is_freespace_pointcloud) {
  CHECK_EQ(ptcloud_C.size(), colors.size());
  tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors,
                                        is_freespace_pointcloud);
}

void TsdfServer::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  { sensor_msgs::msg::PointCloud2 out_msg; pcl::toROSMsg(pointcloud, out_msg); tsdf_pointcloud_pub_->publish(out_msg); }
}

void TsdfServer::publishTsdfSurfacePoints() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(),
                                       surface_distance_thresh, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  { sensor_msgs::msg::PointCloud2 out_msg; pcl::toROSMsg(pointcloud, out_msg); surface_pointcloud_pub_->publish(out_msg); }
}

void TsdfServer::publishTsdfOccupiedNodes() {
  // Create a pointcloud with distance = intensity.
  visualization_msgs::msg::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), world_frame_,
      tsdf_map_->getTsdfLayer().voxel_size() *
          occupancy_min_distance_voxel_size_factor_,
      &marker_array);
  occupancy_marker_pub_->publish(marker_array);
}

void TsdfServer::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayerSlice(tsdf_map_->getTsdfLayer(), 2,
                                             slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  { sensor_msgs::msg::PointCloud2 out_msg; pcl::toROSMsg(pointcloud, out_msg); tsdf_slice_pub_->publish(out_msg); }
}

void TsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_tsdf_map_) {
    return;
  }
  int subscribers = this->tsdf_map_pub_->get_subscription_count();
  if (subscribers > 0) {
    if (num_subscribers_tsdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
      ROS_WARN("New subscriber to tsdf_map_, resetting remote map.");
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxblox_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(this->tsdf_map_->getTsdfLayer(),
                                   only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->tsdf_map_pub_->publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_tsdf_map_ = subscribers;
}

void TsdfServer::publishPointclouds() {
  // Combined function to publish all possible pointcloud messages -- surface
  // pointclouds, updated points, and occupied points.
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
  if (publish_slices_) {
    publishSlices();
  }
}

void TsdfServer::updateMesh() {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_->publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }

  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
  }
}

bool TsdfServer::generateMesh() {
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
  } else {
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_mesh_updated_blocks,
                                   clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_->publish(mesh_msg);

  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  }

  ROS_INFO_STREAM("Mesh Timings: " << std::endl << timing::Timing::Print());
  return true;
}

bool TsdfServer::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool TsdfServer::loadMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load
  // the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = io::LoadBlocksFromFile(
      file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport, tsdf_map_->getTsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  }
  return success;
}

bool TsdfServer::clearMapCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {  // NOLINT
  clear();
  return true;
}

bool TsdfServer::generateMeshCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                      std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {  // NOLINT
  return generateMesh();
}

bool TsdfServer::saveMapCallback(const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
                                 std::shared_ptr<voxblox_msgs::srv::FilePath::Response> /*response*/) {  // NOLINT
  return saveMap(request->file_path);
}

bool TsdfServer::loadMapCallback(const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
                                 std::shared_ptr<voxblox_msgs::srv::FilePath::Response> /*response*/) {  // NOLINT
  bool success = loadMap(request->file_path);
  return success;
}

bool TsdfServer::publishPointcloudsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/, std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {  // NOLINT
  publishPointclouds();
  return true;
}

bool TsdfServer::publishTsdfMapCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {  // NOLINT
  publishMap();
  return true;
}

void TsdfServer::updateMeshEvent() {
  updateMesh();
}

void TsdfServer::publishMapEvent() {
  publishMap();
}

void TsdfServer::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();

  // Publish a message to reset the map to all subscribers.
  if (publish_tsdf_map_) {
    constexpr bool kResetRemoteMap = true;
    publishMap(kResetRemoteMap);
  }
}

void TsdfServer::tsdfMapCallback(const voxblox_msgs::msg::Layer::SharedPtr layer_msg) {
  timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      deserializeMsgToLayer<TsdfVoxel>(*layer_msg, tsdf_map_->getTsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid TSDF map message!");
  } else {
    ROS_INFO_ONCE("Got an TSDF map from ROS topic!");
    if (publish_pointclouds_on_update_) {
      publishPointclouds();
    }
  }
}

}  // namespace voxblox
