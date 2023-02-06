#ifndef VOXBLOX_ROS_TSDF_SERVER_H_
#define VOXBLOX_ROS_TSDF_SERVER_H_

#include <memory>
#include <queue>
#include <string>

#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_msgs/InfoGain.h>
#include <voxblox_msgs/InfoGainBaseline.h>
#include <voxblox_msgs/Mesh.h>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/transformer.h"



namespace voxblox {

constexpr float kDefaultMaxIntensity = 100.0;

constexpr double kHorizontalFov = 87.0 * M_PI/180.0;
constexpr double kHorizontalFov_div2 = 0.5 * kHorizontalFov;
constexpr double kVerticalFov = 58.0 * M_PI/180.0;
constexpr double kVerticalFov_div2 = 0.5 * kVerticalFov;
constexpr double kMaxDetectionRange = 5.0;
constexpr double kMinDetectionRange = 0.1; // IGNORED
constexpr double kRaycastingHorizontalRes = 5.0 * M_PI/180.0;
constexpr double kRaycastingVerticalRes = 5.0 * M_PI/180.0;
constexpr double F_X = 239.35153198242188;
constexpr double F_Y = 239.05279541015625;

// constexpr int kNumYawStep1_ = 16;
// constexpr int kNumYawStep2_ = 3; //32;
constexpr int kNumYaw_ = 32; // kNumYawStep1_ * kNumYawStep2_;
constexpr int kNumVelZ_ = 8;
constexpr int kNumVelX_ = 1; 
constexpr double kForwardVel_ = 0.75;
constexpr int kSkipStepGenerate = 5;
constexpr int kNumTimestep = 15;
constexpr double alpha_v = 0.92; // Ts=0.4, T_sampling=5/(10*15)
constexpr double alpha_psi = 0.9293; // Ts=1/K_yaw=1/2.2, T_sampling=5/(10*15)

typedef Eigen::Matrix<double, 6, 1> StateVec;

enum VoxelStatus { kUnknown = 0, kOccupied, kFree };

struct VolumetricGain {
  VolumetricGain()
      : gain(0),
        accumulative_gain(0),
        num_unknown_voxels(0),
        num_free_voxels(0),
        num_occupied_voxels(0),
        num_unknown_surf_voxels(0) {}

  void reset() {
    gain = 0;
    accumulative_gain = 0;
    num_unknown_voxels = 0;
    num_free_voxels = 0;
    num_occupied_voxels = 0;
    num_unknown_surf_voxels = 0;
    is_frontier = false;
    unseen_voxel_hash_keys.clear();
  }

  double gain;
  double accumulative_gain;
  int num_unknown_voxels;
  int num_free_voxels;
  int num_occupied_voxels;
  int num_unknown_surf_voxels;
  std::vector<std::size_t> unseen_voxel_hash_keys;

  bool is_frontier;

  void printGain() {
    std::cout << "Gains: " << gain << ", " << num_unknown_voxels << ", "
              << num_occupied_voxels << ", " << num_free_voxels << std::endl;
  }
};

struct SensorParamsBase {
  double min_range;  // Minimum range for map annotation (used for zoom camera
                    // only).
  double max_range;  // Maximum range for volumetric gain.
  Eigen::Vector2d fov;            // [Horizontal, Vertical] angles (rad).
  Eigen::Vector2d resolution;  // Resolution in rad [H x V] for volumetric gain.
  int height;                      // Number of rays in vertical direction
  int width;                       // Number of rays in horizontal direction
  int heightRemoval;  // Number of pixel which are not seen on each side in
                      // vertical direction.
  int widthRemoval;   // Number of pixel which are not seen on each side in
                      // horizontal direction.
  void initialize();
  // Check if this state is inside sensor's FOV.
  // pos in (W).
  // bool isInsideFOV(StateVec& state, Eigen::Vector3d& pos);
  // Get all endpoints from ray casting models in (W).
  void getFrustumEndpoints(StateVec& state, std::vector<Eigen::Vector3d>& ep);
  // void getFrustumEndpoints(StateVec& state, std::vector<Eigen::Vector3d>& ep,
  //                         float darkness_range);

  // void updateFrustumEndpoints();
  // Get all edges in (W).
  // void getFrustumEdges(StateVec& state, std::vector<Eigen::Vector3d>& edges);
  // Check if this is potential frontier given sensor FOV.

private:

  // These are to support camera model, approximate as a pyramid.
  // TopLeft, TopRight, BottomRight, BottomLeft.
  Eigen::Matrix<double, 3, 4> edge_points;    // Sensor coordinate, normalized.
  Eigen::Matrix<double, 3, 4> edge_points_B;  // Body coordinate, normalized.
  // These are to support camera model.
  // Place all 4 {top, right, bottom, left} vector into a matrix.
  Eigen::Matrix<double, 3, 4> normal_vectors;

  std::vector<Eigen::Vector3d> frustum_endpoints;    // Sensor coordinate.
  std::vector<Eigen::Vector3d> frustum_endpoints_B;  // Body coordinate.

  double num_voxels_full_fov;
};

class TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
             const TsdfMap::Config& config,
             const TsdfIntegratorBase::Config& integrator_config,
             const MeshIntegratorConfig& mesh_config);
  virtual ~TsdfServer() {}

  void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

  void insertPointcloud(const sensor_msgs::PointCloud2::Ptr& pointcloud);
  void insertPointcloudWithInterestingness(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in);

  void insertFreespacePointcloud(
      const sensor_msgs::PointCloud2::Ptr& pointcloud);

  virtual void processPointCloudMessageAndInsert(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
      const Transformation& T_G_C, const bool is_freespace_pointcloud);
  virtual void processPointCloudMessageAndInsertWithInterestingness(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg, std::shared_ptr<AlignedQueue<GlobalIndex>> interesting_voxel_idx,
    const Transformation& T_G_C, const bool is_freespace_pointcloud);

  void integratePointcloud(const Transformation& T_G_C,
                           const Pointcloud& ptcloud_C, const Colors& colors,
                           const bool is_freespace_pointcloud = false);
  virtual void newPoseCallback(const Transformation& /*new_pose*/) {
    // Do nothing.
  }

  void publishAllUpdatedTsdfVoxels();
  void publishTsdfSurfacePoints();
  void publishTsdfOccupiedNodes();

  virtual void publishSlices();
  /// Incremental update.
  virtual void updateMesh();
  /// Batch update.
  virtual bool generateMesh();
  // Publishes all available pointclouds.
  virtual void publishPointclouds();
  // Publishes the complete map
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);
  bool clearMapCallback(std_srvs::Empty::Request& request,           // NOLINT
                        std_srvs::Empty::Response& response);        // NOLINT
  bool saveMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool generateMeshCallback(std_srvs::Empty::Request& request,       // NOLINT
                            std_srvs::Empty::Response& response);    // NOLINT
  bool publishPointcloudsCallback(
      std_srvs::Empty::Request& request,                             // NOLINT
      std_srvs::Empty::Response& response);                          // NOLINT
  bool publishTsdfMapCallback(std_srvs::Empty::Request& request,     // NOLINT
                              std_srvs::Empty::Response& response);  // NOLINT

  void updateMeshEvent(const ros::TimerEvent& event);
  void publishMapEvent(const ros::TimerEvent& event);

  std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }
  std::shared_ptr<const TsdfMap> getTsdfMapPtr() const { return tsdf_map_; }

  /// Accessors for setting and getting parameters.
  double getSliceLevel() const { return slice_level_; }
  void setSliceLevel(double slice_level) { slice_level_ = slice_level; }

  bool setPublishSlices() const { return publish_slices_; }
  void setPublishSlices(const bool publish_slices) {
    publish_slices_ = publish_slices;
  }

  void setWorldFrame(const std::string& world_frame) {
    world_frame_ = world_frame;
  }
  std::string getWorldFrame() const { return world_frame_; }

  /// CLEARS THE ENTIRE MAP!
  virtual void clear();

  /// Overwrites the layer with what's coming from the topic!
  void tsdfMapCallback(const voxblox_msgs::Layer& layer_msg);

  // custom info gain calculation
  void integratePointcloud(const Transformation& T_G_C,
                            const Pointcloud& ptcloud_C,
                            const Colors& colors,
                            const Interestingness& interestingness,
                            const bool is_freespace_pointcloud);  
  void lightInsertPointcloud(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
    std::shared_ptr<AlignedQueue<GlobalIndex>> interesting_voxel_idx);

  virtual void lightProcessPointCloudMessageAndInsert(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg, 
    std::shared_ptr<AlignedQueue<GlobalIndex>> interesting_voxel_idx,
    const Transformation& T_G_C, const bool is_freespace_pointcloud);

  bool calcInfoGainCallback(voxblox_msgs::InfoGain::Request& request,
                            voxblox_msgs::InfoGain::Response& response);

  bool baselineInfoGainCallback(voxblox_msgs::InfoGainBaseline::Request& request,
                                voxblox_msgs::InfoGainBaseline::Response& response);

  bool saveViewingDistanceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void computeVolumetricGainRayModelNoBound(StateVec& state,
                                            VolumetricGain& vgain);

  float getScanStatus(
      Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
      std::tuple<int, int, int>& gain_log,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      SensorParamsBase& sensor_params);

  void spreadInterestingness(std::shared_ptr<AlignedQueue<GlobalIndex>> interesting_voxel_idx);    

  bool checkUnknownStatus(const voxblox::TsdfVoxel* voxel) const;

  voxblox::Layer<voxblox::TsdfVoxel>* sdf_layer_;
  std::shared_ptr<AlignedQueue<GlobalIndex>> interesting_voxels = std::make_shared<AlignedQueue<GlobalIndex>>();
  std::shared_ptr<AlignedQueue<GlobalIndex>> observed_voxels = std::make_shared<AlignedQueue<GlobalIndex>>();

 protected:
  /**
   * Gets the next pointcloud that has an available transform to process from
   * the queue.
   */
  bool getNextPointcloudFromQueue(
      std::queue<sensor_msgs::PointCloud2::Ptr>* queue,
      sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  /// Data subscribers.
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber freespace_pointcloud_sub_;

  /// Publish markers for visualization.
  ros::Publisher mesh_pub_;
  ros::Publisher tsdf_pointcloud_pub_;
  ros::Publisher surface_pointcloud_pub_;
  ros::Publisher tsdf_slice_pub_;
  ros::Publisher occupancy_marker_pub_;
  ros::Publisher icp_transform_pub_;

  /// Publish the complete map for other nodes to consume.
  ros::Publisher tsdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  ros::Subscriber tsdf_map_sub_;

  // Services.
  ros::ServiceServer calc_info_gain_srv_;
  ros::ServiceServer baseline_info_gain_srv_;
  ros::ServiceServer save_viewing_dist_srv_;
  ros::ServiceServer generate_mesh_srv_;
  ros::ServiceServer clear_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer publish_pointclouds_srv_;
  ros::ServiceServer publish_tsdf_map_srv_;

  /// Tools for broadcasting TFs.
  tf::TransformBroadcaster tf_broadcaster_;

  // Timers.
  ros::Timer update_mesh_timer_;
  ros::Timer publish_map_timer_;

  bool verbose_;

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string world_frame_;
  /**
   * Name of the ICP corrected frame. Publishes TF and transform topic to this
   * if ICP on.
   */
  std::string icp_corrected_frame_;
  /// Name of the pose in the ICP correct Frame.
  std::string pose_corrected_frame_;

  /// Delete blocks that are far from the system to help manage memory
  double max_block_distance_from_body_;

  /// Pointcloud visualization settings.
  double slice_level_;

  /// If the system should subscribe to a pointcloud giving points in freespace
  bool use_freespace_pointcloud_;

  /**
   * Mesh output settings. Mesh is only written to file if mesh_filename_ is
   * not empty.
   */
  std::string mesh_filename_;
  /// How to color the mesh.
  ColorMode color_mode_;

  /// Colormap to use for intensity pointclouds.
  std::shared_ptr<ColorMap> color_map_;

  /// Will throttle to this message rate.
  ros::Duration min_time_between_msgs_;

  /// What output information to publish
  bool publish_pointclouds_on_update_;
  bool publish_slices_;
  bool publish_pointclouds_;
  bool publish_tsdf_map_;

  /// Whether to save the latest mesh message sent (for inheriting classes).
  bool cache_mesh_;

  /**
   *Whether to enable ICP corrections. Every pointcloud coming in will attempt
   * to be matched up to the existing structure using ICP. Requires the initial
   * guess from odometry to already be very good.
   */
  bool enable_icp_;
  /**
   * If using ICP corrections, whether to store accumulate the corrected
   * transform. If this is set to false, the transform will reset every
   * iteration.
   */
  bool accumulate_icp_corrections_;

  /// Subscriber settings.
  int pointcloud_queue_size_;
  int num_subscribers_tsdf_map_;

  // Maps and integrators.
  std::shared_ptr<TsdfMap> tsdf_map_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;

  /// ICP matcher
  std::shared_ptr<ICP> icp_;

  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  /// Optionally cached mesh message.
  voxblox_msgs::Mesh cached_mesh_msg_;

  /**
   * Transformer object to keep track of either TF transforms or messages from
   * a transform topic.
   */
  Transformer transformer_;
  /**
   * Queue of incoming pointclouds, in case the transforms can't be immediately
   * resolved.
   */
  std::queue<sensor_msgs::PointCloud2::Ptr> pointcloud_queue_;
  std::queue<sensor_msgs::PointCloud2::Ptr> freespace_pointcloud_queue_;

  // Last message times for throttling input.
  ros::Time last_msg_time_ptcloud_;
  ros::Time last_msg_time_freespace_ptcloud_;

  /// Current transform corrections from ICP.
  Transformation icp_corrected_transform_;

  // Info gain calculation
  ros::Publisher gain_vis_pub_;
  SensorParamsBase camera_param;
  VolumetricGain vgain;
  double decay_lambda_ = 1.0; // 0.0-1.0
  double decay_distance_ = 5.0;
  double area_factor_ = 1e5;

  // For evaluating viewing quality
  double interestingness_threshold_ = 0.0;
  int interesting_weight_threshold_ = 0;

  // action sequence lib 
  // Eigen::Matrix<double, kNumYaw_ * kNumVelZ_ * kNumVelX_, 6 * kNumTimestep> camera_states_;
  // Eigen::Matrix<double, kNumYaw_ * kNumVelZ_ * kNumVelX_, 3 * kNumTimestep> action_sequences_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> camera_states_ =
      Eigen::MatrixXd::Zero(kNumYaw_ * kNumVelZ_ * kNumVelX_, 6 * kNumTimestep);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> camera_states_rotated_ =
      Eigen::MatrixXd::Zero(kNumYaw_ * kNumVelZ_ * kNumVelX_, 3 * kNumTimestep); // rotate camera_states_ to align with the current robot's yaw
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> action_sequences_ = 
      Eigen::MatrixXd::Zero(kNumYaw_ * kNumVelZ_ * kNumVelX_, 3 * kNumTimestep);
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_
