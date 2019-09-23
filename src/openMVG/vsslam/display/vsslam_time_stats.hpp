// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>
#include <stdlib.h>
#include <unistd.h>
#include "openMVG/types.hpp"
#include <openMVG/vsslam/system/Frame.hpp>
#include "openMVG/matching/indMatchDecoratorXY.hpp"

#ifndef SWINE_NOGL
#include "software/VSSLAM/CGlWindow.hpp" // swine
#endif // !SWINE_NOGL

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

namespace openMVG {
namespace vsslam {

/// Define struct for all parameters

struct VSSLAM_Time_Stats
{
  bool b_enable_time_stats = true;
  bool b_enable_features_stats = true;

  //graph file
  std::ofstream stats_file;


  bool b_keyframe = false;
  size_t keyframe_reason = 0; 
  IndexT frame_id = UndefinedIndexT;

  // ------------------------
  // Time Statistics
  // ------------------------
  double d_track_track = 0.0; // Total tracking time of frame
  double d_feat_detection = 0.0;
  double d_feat_tracking = 0.0;
  double d_feat_tracking_mm = 0.0;
  double d_feat_tracking_pose_opt_mm = 0.0;
  double d_feat_tracking_rf = 0.0;
  double d_feat_tracking_pose_opt_rf = 0.0;
  double d_feat_tracking_rm = 0.0;
  double d_feat_tracking_pose_opt_rm = 0.0;
  double d_feat_tracking_lm = 0.0;
  double d_feat_tracking_pose_opt_lm = 0.0;

  double d_feat_mapping = 0.0;
  double d_feat_mapping_search_new_pts = 0.0;
  double d_feat_mapping_pose_opt_local = 0.0;
  double d_feat_mapping_global_step = 0.0;

  double d_feat_mapping_global_step_prepare = 0.0;
  double d_feat_mapping_global_step_BA = 0.0;
  double d_feat_mapping_global_step_update = 0.0;

  double d_feat_mapping_outliers_after_global = 0.0;

  double d_pose_init = 0.0;  // not measured yet

  // ------------------------
  // Features Statistics
  // ------------------------
  size_t i_matches_mm = 0;
  size_t i_matches_mm_outliers = 0;
  size_t i_matches_rf = 0;
  size_t i_matches_rf_outliers = 0;
  size_t i_matches_rm = 0;

  size_t i_matches_lm = 0;



  // ------------------------
  // Map Statistics
  // ------------------------
  size_t global_frames = 0;
  size_t global_landmarks = 0;
  size_t local_frames = 0;
  size_t local_landmarks = 0;

  size_t added_global_landmarks = 0;
  size_t added_local_landmarks = 0;
  size_t added_local_to_global_landmarks = 0;
  size_t removed_local_landmarks_outliers = 0;
  size_t removed_local_landmarks_inactive = 0;

  // ------------------------
  // Frame stats
  // ------------------------
  size_t global_step_id = 0;

  void startTimer(double & d_time)
  {
    #ifdef OPENMVG_USE_OPENMP
    if (b_enable_time_stats)
      d_time = omp_get_wtime();
    #endif
  }

  void stopTimer(double & d_time)
  {
    
    #ifdef OPENMVG_USE_OPENMP
    if (b_enable_time_stats)
      d_time = omp_get_wtime() - d_time;
    #endif
  }

  void restartData()
  {
    b_keyframe = false;
    frame_id = UndefinedIndexT;

    // ------------------------
    // Time Statistics
    // ------------------------
    d_track_track = 0.0; // Total tracking time of frame
    d_feat_detection = 0.0;

    d_feat_tracking = 0.0;
    d_feat_tracking_mm = 0.0;
    d_feat_tracking_pose_opt_mm = 0.0;
    d_feat_tracking_rf = 0.0;
    d_feat_tracking_pose_opt_rf = 0.0;
    d_feat_tracking_rm = 0.0;
    d_feat_tracking_pose_opt_rm = 0.0;
    d_feat_tracking_lm = 0.0;
    d_feat_tracking_pose_opt_lm = 0.0;

    d_feat_mapping = 0.0;
    d_feat_mapping_search_new_pts = 0.0;
    d_feat_mapping_pose_opt_local = 0.0;
    d_feat_mapping_global_step = 0.0;
    d_feat_mapping_global_step_prepare = 0.0;
    d_feat_mapping_global_step_BA = 0.0;
    d_feat_mapping_global_step_update = 0.0;

    d_feat_mapping_outliers_after_global = 0.0;

    d_pose_init = 0.0;  // not measured yet

    // ------------------------
    // Features Statistics
    // ------------------------
    i_matches_mm = 0;
    i_matches_mm_outliers = 0;
    i_matches_rf = 0;
    i_matches_rf_outliers = 0;
    i_matches_rm = 0;
    i_matches_lm = 0;
    global_frames = 0;
    global_landmarks = 0;
    local_landmarks = 0;
    added_global_landmarks = 0;
    added_local_landmarks = 0;
    added_local_to_global_landmarks = 0;
    removed_local_landmarks_outliers = 0;
    removed_local_landmarks_inactive = 0;
    keyframe_reason = 0;
  }
};


}
}
