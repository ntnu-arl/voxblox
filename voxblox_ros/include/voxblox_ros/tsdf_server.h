#ifndef VOXBLOX_ROS_TSDF_SERVER_H_
#define VOXBLOX_ROS_TSDF_SERVER_H_

#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <queue>
#include <string>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/transformer.h"
#include "voxblox_ros/ros_interface.hpp"

namespace voxblox {

constexpr float kDefaultMaxIntensity = 100.0;

class TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TsdfServer(NodeHandle& nh, NodeHandle& nh_private);
  TsdfServer(NodeHandle& nh, NodeHandle& nh_private,
             const TsdfMap::Config& config,
             const TsdfIntegratorBase::Config& integrator_config,
             const MeshIntegratorConfig& mesh_config);
  virtual ~TsdfServer() {}

  void getServerConfigFromRosParam(NodeHandle& nh_private);

  void insertPointcloud(const PointCloud2Msg::ConstSharedPtr& pointcloud);

  void insertFreespacePointcloud(
      PointCloud2Msg::ConstSharedPtr pointcloud);

  virtual void processPointCloudMessageAndInsert(
      const PointCloud2Msg::ConstSharedPtr& pointcloud_msg,
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

  bool clearMapCallback(EmptySrv::Request::SharedPtr request,           // NOLINT
                        EmptySrv::Response::SharedPtr response);        // NOLINT
  bool saveMapCallback(FilePathSrv::Request::SharedPtr request,     // NOLINT
                       FilePathSrv::Response::SharedPtr response);  // NOLINT
  bool loadMapCallback(FilePathSrv::Request::SharedPtr request,     // NOLINT
                       FilePathSrv::Response::SharedPtr response);  // NOLINT
  bool generateMeshCallback(EmptySrv::Request::SharedPtr request,       // NOLINT
                            EmptySrv::Response::SharedPtr response);    // NOLINT
  bool publishPointcloudsCallback(
      EmptySrv::Request::SharedPtr request,                             // NOLINT
      EmptySrv::Response::SharedPtr response);                          // NOLINT
  bool publishTsdfMapCallback(EmptySrv::Request::SharedPtr request,     // NOLINT
                              EmptySrv::Response::SharedPtr response);  // NOLINT

  void updateMeshEvent();
  void publishMapEvent();

  std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }

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
  void tsdfMapCallback(LayerMsg::ConstSharedPtr layer_msg);

 protected:
  /**
   * Gets the next pointcloud that has an available transform to process from
   * the queue.
   */
  bool getNextPointcloudFromQueue(
      std::queue<PointCloud2Msg::ConstSharedPtr>* queue,
      PointCloud2Msg::ConstSharedPtr* pointcloud_msg, Transformation* T_G_C);

  NodeHandle& nh_;
  NodeHandle& nh_private_;

  /// Data subscribers.
  Subscriber<PointCloud2Msg> pointcloud_sub_;
  Subscriber<PointCloud2Msg> freespace_pointcloud_sub_;

  /// Publish markers for visualization.
  Publisher<MeshMsg> mesh_pub_;
  Publisher<PointCloud2Msg> tsdf_pointcloud_pub_;
  Publisher<PointCloud2Msg> surface_pointcloud_pub_;
  Publisher<PointCloud2Msg> tsdf_slice_pub_;
  Publisher<MarkerArrayMsg> occupancy_marker_pub_;
  Publisher<TransformStampedMsg> icp_transform_pub_;

  /// Publish the complete map for other nodes to consume.
  Publisher<LayerMsg> tsdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  Subscriber<LayerMsg> tsdf_map_sub_;

  // Services.
  Service<EmptySrv> generate_mesh_srv_;
  Service<EmptySrv> clear_map_srv_;
  Service<FilePathSrv> save_map_srv_;
  Service<FilePathSrv> load_map_srv_;
  Service<EmptySrv> publish_pointclouds_srv_;
  Service<EmptySrv> publish_tsdf_map_srv_;

  /// Tools for broadcasting TFs.
  TransformBroadcaster tf_broadcaster_;

  // Timers.
  TimerBase update_mesh_timer_;
  TimerBase publish_map_timer_;

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
  Duration min_time_between_msgs_;

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

  FloatingPoint occupancy_min_distance_voxel_size_factor_;

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
  MeshMsg cached_mesh_msg_;

  /**
   * Transformer object to keep track of either TF transforms or messages from
   * a transform topic.
   */
  Transformer transformer_;
  /**
   * Queue of incoming pointclouds, in case the transforms can't be immediately
   * resolved.
   */
  std::queue<PointCloud2Msg::ConstSharedPtr> pointcloud_queue_;
  std::queue<PointCloud2Msg::ConstSharedPtr> freespace_pointcloud_queue_;

  // Last message times for throttling input.
  Time last_msg_time_ptcloud_;
  Time last_msg_time_freespace_ptcloud_;

  /// Current transform corrections from ICP.
  Transformation icp_corrected_transform_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_
