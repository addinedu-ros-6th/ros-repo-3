-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link", --"base_link", "imu_link"
  published_frame = "map", -- "map", "base_link"
  odom_frame = "odom", -- "odom",
  provide_odom_frame = false, -- Set to false since odom is already provided by the robot
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- options.tracking_frame = "base_link"

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15  -- Minimize if you need more nearby details
TRAJECTORY_BUILDER_2D.max_range = 5.0  -- Maximize to capture distant landmarks
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0 -- must be written in float type
--TRAJECTORY_BUILDER_2D.use_laser_scan = true -- coment this out when duplicate error occurs
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Increasing this (below) can improve scan quality by averaging over multiple frames but may reduce real-time performance
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 5 -- controls how many scans are combined before inserting into the submap.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3  -- Increase for more robust scan matching
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.0)

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1  -- Lower to retain more scans at close distances
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.05)  -- Lower to improve rotational accuracy

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100  -- Increase this to make submaps accumulate more data (originally set to about 50)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- Reduce this (e.g., to 0.05) for higher resolution but may cause higher computational cost

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5e2  -- Increase if map drifts
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4e2  -- Increase for more stable rotation correction

-- for loop closure
-- POSE_GRAPH.constraint_builder.logging_enabled = true
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5  -- Adjust to reduce computation load; 0.3 is a good starting point
POSE_GRAPH.constraint_builder.min_score = 0.55  -- Reduce slightly if features are being forgotten
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- Make global localization more permissive if about 0.5
POSE_GRAPH.optimize_every_n_nodes = 1  -- Reduce to run optimizations more frequently for dynamic environments


-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e3
-- TRAJECTORY_BUILDER.pure_localization = true 

return options
