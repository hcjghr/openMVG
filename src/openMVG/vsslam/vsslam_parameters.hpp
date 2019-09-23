// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>

namespace openMVG {
namespace vsslam {

class Frame;
class MapLandmark;

/// Define struct for all parameters

struct VSSLAM_Parameters : public std::enable_shared_from_this<VSSLAM_Parameters>
{

  std::shared_ptr<VSSLAM_Parameters> share_ptr()
  {
    return shared_from_this();
  }


  // ---------------
  // Tracking initialization parameters
  // ---------------
  size_t init_track_min_matches = 30; // Min number of points needed for initialization of tracking
  float init_track_max_model_thresh_px = 2.0f;
  float init_track_min_cos_parallax_pt = 0.99998 ; // Min angle between rays for the point to be triangulated (0.99998 ~ 0.36deg; 0.99995 ~ 0.5deg;  0.9998 ~ 1.15deg)


  // ---------------
  // Tracking parameters
  // ---------------
  size_t track_min_matches = 10;  // Min matches needed for the tracking to be successful

  // Motion model
  float track_match_mm_desc_ratio = 0.8; // Matching ratio for matching using motion model
  float track_match_mm_max_scale_ratio = 1.1; // Matching can be done only between points with max twice the scale difference
  float track_match_mm_win_size = 3;

  // Reference frame
  float track_match_rf_desc_ratio = 0.9; // Matching ratio for matching using motion model
  float track_match_rf_max_scale_ratio = 1.2; // Matching can be done only between points with max twice the scale difference
  float track_match_rf_win_size = 10;

  // Relocalization
  float relocalization_max_model_thresh_px = 2.0f;
  float track_match_reloc_desc_ratio = 0.9; // Matching ratio for matching using motion model
  float track_match_reloc_max_scale_ratio = 1.2; // Matching can be done only between points with max twice the scale difference


  // Local map
  size_t track_local_map_n_frames = 10; // Number of frames used for construction local map for tracking
  float track_match_lm_desc_ratio = 0.6; // Matching ratio for matching using motion model
  float track_match_lm_max_scale_ratio = 1.2; // Matching can be done only between points with max twice the scale difference
  float track_match_lm_win_size = 1;


  // ---------------
  // Matching parameters
  // ---------------
  // Initialization
  float match_init_desc_ratio = 0.8f; // Matching ration of descriptors
  float match_init_max_scale_ratio = 1.2f; // Matching can be done only between points with max twice the scale difference


  // ---------------
  // New Triangulations
  // ---------------
  size_t triangulation_local_map_n_frames = 5; // Number of frames used for constructing local map of frames used for finding new landmarks
  float triang_match_desc_ratio = 0.8; // Matching ratio for matching using motion model
  float triang_match_max_scale_ratio = 1.1; // Matching can be done only between points with max twice the scale difference
  float triang_match_max_epipolar_distance = 6.0; // Max distance to epipolar line
  // ---------------
  // Mapping parameters
  // ---------------
  size_t map_min_frame_init = 3; // Minimum frames needed for initialization of map
  float map_min_quality_landmark = 3;  // Minimum of observations needed for a point to be added to global map
  size_t map_max_inactive_f_local_landmark = 10; // Maximum frames of inactivity that local landmark survives
  size_t map_min_init_pts = 30;
  size_t map_min_obs_per_frame = 10;

// ---------------
  // New Keyframe
  // ---------------
  size_t n_max_frames_not_tracked = 100;  // Max number of frames between two tracked frames
  size_t n_min_global_landmarks_tracked = 50; // Min number of global landmarks needed to be tracked (otherwise new frame)
  size_t n_min_global_landmarks_to_check_prev_stats = 100;  // Min number of global landmarks to be confident (otherwise check stats of prev frame)
  float f_min_ratio_of_landmarks_tracked_reference_frame = 0.6; // Min number of landmark tracked compared to prev frame (otherwise new frame)
  float f_max_ratio_of_landmarks_tracked_prev_frame = 1.4;  // If more than ratio of landmarks tracked add frame



  // ---------------
  // Output parameters
  // ---------------
  std::string s_output_folder = "";
  bool b_export_intermediate_scene_ply = true;
  bool b_export_stats_file = true;
  bool b_export_graph_file = true;

  std::string s_graph_file_path = "SlamPP_graph_file.txt";
  std::string s_stats_file_path = "VSSLAM_stats.csv";


  int verbose_level = 0;
};


}
}
