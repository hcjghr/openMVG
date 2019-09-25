// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>
#include <iostream>
#include <fstream>

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
  // Verbose level
  // ---------------
  int verbose_level = 0;

  // ---------------
  // Tracking general parameters
  // ---------------
  size_t max_lost_frames_before_reinit = 5; // Number of frames the system can be lost before restating

  // ---------------
  // Feature extraction
  // ---------------
  float feat_extract_sift_thresh = 0.04f;
  float feat_extract_akaze_thresh = 1.0f;

  // ---------------
  // Tracking initialization parameters
  // ---------------
  size_t init_track_min_matches = 30; // Min number of points needed for initialization of tracking
  float init_track_max_model_thresh_im_ratio = 0.004; //
  float init_track_max_model_thresh_px = 2.0f;
  float init_track_min_cos_parallax_pt = 0.99995; // Min angle between rays for the point to be triangulated (0.99998 ~ 0.36deg; 0.99995 ~ 0.5deg;  0.9998 ~ 1.15deg)

  // ---------------
  // Tracking parameters
  // ---------------
  size_t track_min_matches = 10;  // Min matches needed for the tracking to be successful

  // Motion model
  float track_match_mm_desc_ratio = 0.8; // Matching ratio for matching using motion model
  float track_match_mm_max_scale_ratio = 1.1; // Matching can be done only between points with max twice the scale difference
  float track_match_mm_win_size_im_ratio = 0.006;
  float track_match_mm_win_size = 3;

  // Reference frame
  float track_match_rf_desc_ratio = 0.8; // Matching ratio for matching using motion model
  float track_match_rf_max_scale_ratio = 1.2; // Matching can be done only between points with max twice the scale difference
  float track_match_rf_win_size_im_ratio = 0.006;
  float track_match_rf_win_size = 10;

  // Relocalization
  float relocalization_max_model_thresh_px_im_ratio = 0.015f;
  float relocalization_max_model_thresh_px = 2.0f;
  float track_match_reloc_desc_ratio = 0.8; // Matching ratio for matching using motion model
  float track_match_reloc_max_scale_ratio = 1.4; // Matching can be done only between points with max twice the scale difference

  // Local map
  size_t track_local_map_n_frames = 20; // Number of frames used for construction local map for tracking
  float track_match_lm_desc_ratio = 0.6; // Matching ratio for matching using motion model
  float track_match_lm_max_scale_ratio = 1.4; // Matching can be done only between points with max twice the scale difference
  float track_match_lm_win_size_im_ratio = 0.008;
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
  float triang_match_max_epipolar_distance_im_ratio = 0.008; // Max distance to epipolar line
  float triang_match_max_epipolar_distance = 6.0; // Max distance to epipolar line
  // ---------------
  // Mapping parameters
  // ---------------
  size_t map_min_frame_init = 3; // Minimum frames needed for initialization of map
  float map_min_quality_landmark = 3;  // Minimum of observations needed for a point to be added to global map
  size_t map_max_inactive_f_local_landmark = 30; // Maximum frames of inactivity that local landmark survives
  size_t map_min_init_pts = 30;
  size_t map_min_obs_per_frame = 10;

  // ---------------
  // New Keyframe
  // ---------------
  size_t n_max_frames_not_tracked = 100;  // Max number of frames between two tracked frames
  size_t n_min_global_landmarks_tracked = 20; // Min number of global landmarks needed to be tracked (otherwise new frame)
  size_t n_min_global_landmarks_to_check_prev_stats = 40;  // Min number of global landmarks to be confident (otherwise check stats of prev frame)

  float f_min_ratio_of_landmarks_tracked_reference_frame = 0.4; // Min number of landmark tracked compared to prev frame (otherwise new frame)
  float f_max_ratio_of_landmarks_tracked_prev_frame = 1.4;//1.4;  // If more than ratio of landmarks tracked add frame

  // ---------------
  // SlamPP Settings
  // ---------------
  size_t slampp_n_max_inc_iters = 2;

  // ---------------
  // Output parameters
  // ---------------
  std::string s_output_folder = "";
  bool b_export_intermediate_scene_ply = true;
  bool b_export_stats_file = true;
  bool b_export_graph_file = true;

  std::string s_graph_file_path = "SlamPP_graph_file.txt";
  std::string s_stats_file_path = "VSSLAM_stats.csv";


  // Process
  std::string trim(std::string const& source, char const* delims = " \t\r\n") {
    std::string result(source);
    std::string::size_type index = result.find_last_not_of(delims);
    if(index != std::string::npos)
      result.erase(++index);

    index = result.find_first_not_of(delims);
    if(index != std::string::npos)
      result.erase(0, index);
    else
      result.erase();
    return result;
  }
  void LoadConfigFile(std::string configFile)
  {
    std::ifstream file(configFile.c_str());

    std::string line;
    std::string name;
    std::string value;
    int posEqual;
    while (std::getline(file,line)) {

      if (! line.length()) continue;

      if (line[0] == '#') continue;
      if (line[0] == ';') continue;

      posEqual=line.find('=');
      name  = trim(line.substr(0,posEqual));
      value = trim(line.substr(posEqual+1));

        if (name=="verbose_level")
        {
          verbose_level = std::stoi(value);
          
        }
        else if (name== "max_lost_frames_before_reinit")
        {
          max_lost_frames_before_reinit = std::stof(value);
          
        }
        else if (name== "feat_extract_sift_thresh")
        {
          feat_extract_sift_thresh = std::stof(value);
          
        }
        else if (name== "feat_extract_akaze_thresh")
        {
          feat_extract_akaze_thresh = std::stof(value);
          
        }
        else if (name== "init_track_min_matches")
        {
          init_track_min_matches = std::stoi(value);
          
        }
        else if (name== "init_track_max_model_thresh_im_ratio")
        {
          init_track_max_model_thresh_im_ratio = std::stof(value);
          
        }
        else if (name== "init_track_min_cos_parallax_pt")
        {
          init_track_min_cos_parallax_pt = std::stof(value);
          
        }
        else if (name== "track_min_matches")
        {
          track_min_matches = std::stoi(value);
          
        }
        else if (name== "track_match_mm_desc_ratio")
        {
          track_match_mm_desc_ratio = std::stof(value);
          
        }
        else if (name== "track_match_mm_max_scale_ratio")
        {
          track_match_mm_max_scale_ratio = std::stof(value);
          
        }
        else if (name== "track_match_mm_win_size_im_ratio")
        {
          track_match_mm_win_size_im_ratio = std::stof(value);
          
        }
        else if (name== "track_match_rf_desc_ratio")
        {
          track_match_rf_desc_ratio = std::stof(value);
          
        }
        else if (name== "track_match_rf_max_scale_ratio")
        {
          track_match_rf_max_scale_ratio = std::stof(value);
          
        }
        else if (name== "track_match_rf_win_size_im_ratio")
        {
          track_match_rf_win_size_im_ratio = std::stof(value);
          
        }
        else if (name== "relocalization_max_model_thresh_px_im_ratio")
        {
          relocalization_max_model_thresh_px_im_ratio = std::stof(value);
          
        }
        else if (name== "track_match_reloc_desc_ratio")
        {
          track_match_reloc_desc_ratio = std::stof(value);
          
        }
        else if (name== "track_match_reloc_max_scale_ratio")
        {
          track_match_reloc_max_scale_ratio = std::stof(value);
          
        }
        else if (name== "track_local_map_n_frames")
        {
          track_local_map_n_frames = std::stoi(value);
          
        }
        else if (name== "track_match_lm_desc_ratio")
        {
          track_match_lm_desc_ratio = std::stof(value);
          
        }
        else if (name== "track_match_lm_max_scale_ratio")
        {
          track_match_lm_max_scale_ratio = std::stof(value);
          
        }
        else if (name== "track_match_lm_win_size_im_ratio")
        {
          track_match_lm_win_size_im_ratio = std::stof(value);
          
        }
        else if (name== "match_init_desc_ratio")
        {
          match_init_desc_ratio = std::stof(value);
          
        }
        else if (name== "match_init_max_scale_ratio")
        {
          match_init_max_scale_ratio = std::stof(value);
          
        }
        else if (name== "triangulation_local_map_n_frames")
        {
          triangulation_local_map_n_frames = std::stoi(value);
          
        }
        else if (name== "triang_match_desc_ratio")
        {
          triang_match_desc_ratio = std::stof(value);
          
        }
        else if (name== "triang_match_max_scale_ratio")
        {
          triang_match_max_scale_ratio = std::stof(value);
          
        }
        else if (name== "triang_match_max_epipolar_distance_im_ratio")
        {
          triang_match_max_epipolar_distance_im_ratio = std::stof(value);
          
        }
        else if (name== "map_min_frame_init")
        {
          map_min_frame_init = std::stoi(value);
          
        }
        else if (name== "map_min_quality_landmark")
        {
          map_min_quality_landmark = std::stoi(value);
          
        }
        else if (name== "map_max_inactive_f_local_landmark")
        {
          map_max_inactive_f_local_landmark = std::stoi(value);
          
        }
        else if (name== "map_min_init_pts")
        {
          map_min_init_pts = std::stoi(value);
          
        }
        else if (name== "map_min_obs_per_frame")
        {
          map_min_obs_per_frame = std::stoi(value);
          
        }
        else if (name== "n_max_frames_not_tracked")
        {
          n_max_frames_not_tracked = std::stoi(value);
          
        }
        else if (name== "n_min_global_landmarks_tracked")
        {
          n_min_global_landmarks_tracked = std::stoi(value);
          
        }
        else if (name== "n_min_global_landmarks_to_check_prev_stats")
        {
          n_min_global_landmarks_to_check_prev_stats = std::stoi(value);
          
        }
        else if (name== "f_min_ratio_of_landmarks_tracked_reference_frame")
        {
          f_min_ratio_of_landmarks_tracked_reference_frame = std::stof(value);
          
        }
        else if (name== "f_max_ratio_of_landmarks_tracked_prev_frame")
        {
          f_max_ratio_of_landmarks_tracked_prev_frame = std::stof(value);
          
        }
        else if (name== "slampp_n_max_inc_iters")
        {
          slampp_n_max_inc_iters = std::stoi(value);
          
        }
        else if (name== "s_graph_file_path")
        {
          s_graph_file_path = value;
          
        }
        else if (name== "s_stats_file_path")
        {
          s_stats_file_path = value;
          
        }
    }
  }

  void updateThresholdsWithImageSize(int width, int height)
  {
    int max_dim = std::max<int>(width,height);

    init_track_max_model_thresh_px = init_track_max_model_thresh_im_ratio * max_dim;
    track_match_mm_win_size = track_match_mm_win_size_im_ratio * max_dim;
    track_match_rf_win_size = track_match_rf_win_size_im_ratio * max_dim;
    relocalization_max_model_thresh_px = relocalization_max_model_thresh_px_im_ratio * max_dim;
    track_match_lm_win_size = track_match_lm_win_size_im_ratio * max_dim;
    triang_match_max_epipolar_distance = triang_match_max_epipolar_distance_im_ratio * max_dim;

    std::cout << "Params: updateThresholdsWithImageSize" << std::endl
              << "init_track_max_model_thresh_px: " << init_track_max_model_thresh_px << std::endl
              << "track_match_mm_win_size: " << track_match_mm_win_size << std::endl
              << "track_match_rf_win_size: " << track_match_rf_win_size << std::endl
              << "relocalization_max_model_thresh_px: " << relocalization_max_model_thresh_px << std::endl
              << "track_match_lm_win_size: " << track_match_lm_win_size << std::endl
              << "triang_match_max_epipolar_distance: " << triang_match_max_epipolar_distance << std::endl;
  }



};


}
}
