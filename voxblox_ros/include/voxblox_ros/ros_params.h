#ifndef VOXBLOX_ROS_ROS_PARAMS_H_
#define VOXBLOX_ROS_ROS_PARAMS_H_

#include <rclcpp/rclcpp.hpp>
#include <ros1_compat/ros1_compat.h>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxblox {

inline TsdfMap::Config getTsdfMapConfigFromRosParam() {
  TsdfMap::Config tsdf_config;

  /**
   * Workaround for OS X on mac mini not having specializations for float
   * for some reason.
   */
  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  voxel_size = ros1_compat::param<std::decay_t<decltype(voxel_size)>>("tsdf_voxel_size", voxel_size);
  voxels_per_side = ros1_compat::param<std::decay_t<decltype(voxels_per_side)>>("tsdf_voxels_per_side", voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    ROS_ERROR("voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;

  return tsdf_config;
}

inline ICP::Config getICPConfigFromRosParam() {
  ICP::Config icp_config;

  icp_config.min_match_ratio = ros1_compat::param<std::decay_t<decltype(icp_config.min_match_ratio)>>("icp_min_match_ratio", icp_config.min_match_ratio);
  icp_config.subsample_keep_ratio = ros1_compat::param<std::decay_t<decltype(icp_config.subsample_keep_ratio)>>("icp_subsample_keep_ratio", icp_config.subsample_keep_ratio);
  icp_config.mini_batch_size = ros1_compat::param<std::decay_t<decltype(icp_config.mini_batch_size)>>("icp_mini_batch_size", icp_config.mini_batch_size);
  icp_config.refine_roll_pitch = ros1_compat::param<std::decay_t<decltype(icp_config.refine_roll_pitch)>>("icp_refine_roll_pitch", icp_config.refine_roll_pitch);
  icp_config.inital_translation_weighting = ros1_compat::param<std::decay_t<decltype(icp_config.inital_translation_weighting)>>("icp_inital_translation_weighting", icp_config.inital_translation_weighting);
  icp_config.inital_rotation_weighting = ros1_compat::param<std::decay_t<decltype(icp_config.inital_rotation_weighting)>>("icp_inital_rotation_weighting", icp_config.inital_rotation_weighting);

  return icp_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam() {
  TsdfIntegratorBase::Config integrator_config;

  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam();
  integrator_config.default_truncation_distance =
      tsdf_config.tsdf_voxel_size * 4;

  double truncation_distance = integrator_config.default_truncation_distance;
  double max_weight = integrator_config.max_weight;
  integrator_config.voxel_carving_enabled = ros1_compat::param<std::decay_t<decltype(integrator_config.voxel_carving_enabled)>>("voxel_carving_enabled", integrator_config.voxel_carving_enabled);
  truncation_distance = ros1_compat::param<std::decay_t<decltype(truncation_distance)>>("truncation_distance", truncation_distance);
  integrator_config.max_ray_length_m = ros1_compat::param<std::decay_t<decltype(integrator_config.max_ray_length_m)>>("max_ray_length_m", integrator_config.max_ray_length_m);
  integrator_config.min_ray_length_m = ros1_compat::param<std::decay_t<decltype(integrator_config.min_ray_length_m)>>("min_ray_length_m", integrator_config.min_ray_length_m);
  max_weight = ros1_compat::param<std::decay_t<decltype(max_weight)>>("max_weight", max_weight);

  integrator_config.use_const_weight = ros1_compat::param<std::decay_t<decltype(integrator_config.use_const_weight)>>("use_const_weight", integrator_config.use_const_weight);
  integrator_config.use_symmetric_weight_dropoff = ros1_compat::param<std::decay_t<decltype(integrator_config.use_symmetric_weight_dropoff)>>("use_symmetric_weight_dropoff", integrator_config.use_symmetric_weight_dropoff);
  integrator_config.use_weight_dropoff = ros1_compat::param<std::decay_t<decltype(integrator_config.use_weight_dropoff)>>("use_weight_dropoff", integrator_config.use_weight_dropoff);

  integrator_config.allow_clear = ros1_compat::param<std::decay_t<decltype(integrator_config.allow_clear)>>("allow_clear", integrator_config.allow_clear);
  integrator_config.start_voxel_subsampling_factor = ros1_compat::param<std::decay_t<decltype(integrator_config.start_voxel_subsampling_factor)>>("start_voxel_subsampling_factor", integrator_config.start_voxel_subsampling_factor);
  integrator_config.max_consecutive_ray_collisions = ros1_compat::param<std::decay_t<decltype(integrator_config.max_consecutive_ray_collisions)>>("max_consecutive_ray_collisions", integrator_config.max_consecutive_ray_collisions);
  integrator_config.clear_checks_every_n_frames = ros1_compat::param<std::decay_t<decltype(integrator_config.clear_checks_every_n_frames)>>("clear_checks_every_n_frames", integrator_config.clear_checks_every_n_frames);
  integrator_config.max_integration_time_s = ros1_compat::param<std::decay_t<decltype(integrator_config.max_integration_time_s)>>("max_integration_time_s", integrator_config.max_integration_time_s);
  integrator_config.enable_anti_grazing = ros1_compat::param<std::decay_t<decltype(integrator_config.enable_anti_grazing)>>("anti_grazing", integrator_config.enable_anti_grazing);
  integrator_config.clearing_ray_weight_factor = ros1_compat::param<std::decay_t<decltype(integrator_config.clearing_ray_weight_factor)>>("clearing_ray_weight_factor", integrator_config.clearing_ray_weight_factor);
  integrator_config.weight_ray_by_range = ros1_compat::param<std::decay_t<decltype(integrator_config.weight_ray_by_range)>>("weight_ray_by_range", integrator_config.weight_ray_by_range);
  integrator_config.integration_order_mode = ros1_compat::param<std::decay_t<decltype(integrator_config.integration_order_mode)>>("integration_order_mode", integrator_config.integration_order_mode);

  integrator_config.default_truncation_distance =
      static_cast<float>(truncation_distance);
  integrator_config.max_weight = static_cast<float>(max_weight);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromRosParam() {
  EsdfMap::Config esdf_config;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam();
  esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
  esdf_config.esdf_voxels_per_side = tsdf_config.tsdf_voxels_per_side;

  return esdf_config;
}

inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam() {
  EsdfIntegrator::Config esdf_integrator_config;

  TsdfIntegratorBase::Config tsdf_integrator_config =
      getTsdfIntegratorConfigFromRosParam();

  esdf_integrator_config.min_distance_m =
      tsdf_integrator_config.default_truncation_distance / 2.0;

  esdf_integrator_config.full_euclidean_distance = ros1_compat::param<std::decay_t<decltype(esdf_integrator_config.full_euclidean_distance)>>("esdf_euclidean_distance", esdf_integrator_config.full_euclidean_distance);
  esdf_integrator_config.max_distance_m = ros1_compat::param<std::decay_t<decltype(esdf_integrator_config.max_distance_m)>>("esdf_max_distance_m", esdf_integrator_config.max_distance_m);
  esdf_integrator_config.min_distance_m = ros1_compat::param<std::decay_t<decltype(esdf_integrator_config.min_distance_m)>>("esdf_min_distance_m", esdf_integrator_config.min_distance_m);
  esdf_integrator_config.default_distance_m = ros1_compat::param<std::decay_t<decltype(esdf_integrator_config.default_distance_m)>>("esdf_default_distance_m", esdf_integrator_config.default_distance_m);
  esdf_integrator_config.min_diff_m = ros1_compat::param<std::decay_t<decltype(esdf_integrator_config.min_diff_m)>>("esdf_min_diff_m", esdf_integrator_config.min_diff_m);
  esdf_integrator_config.clear_sphere_radius = ros1_compat::param<std::decay_t<decltype(esdf_integrator_config.clear_sphere_radius)>>("clear_sphere_radius", esdf_integrator_config.clear_sphere_radius);
  esdf_integrator_config.occupied_sphere_radius = ros1_compat::param<std::decay_t<decltype(esdf_integrator_config.occupied_sphere_radius)>>("occupied_sphere_radius", esdf_integrator_config.occupied_sphere_radius);
  esdf_integrator_config.add_occupied_crust = ros1_compat::param<std::decay_t<decltype(esdf_integrator_config.add_occupied_crust)>>("esdf_add_occupied_crust", esdf_integrator_config.add_occupied_crust);
  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  return esdf_integrator_config;
}

inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam() {
  MeshIntegratorConfig mesh_integrator_config;

  mesh_integrator_config.min_weight = ros1_compat::param<std::decay_t<decltype(mesh_integrator_config.min_weight)>>("mesh_min_weight", mesh_integrator_config.min_weight);
  mesh_integrator_config.use_color = ros1_compat::param<std::decay_t<decltype(mesh_integrator_config.use_color)>>("mesh_use_color", mesh_integrator_config.use_color);

  return mesh_integrator_config;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ROS_PARAMS_H_
