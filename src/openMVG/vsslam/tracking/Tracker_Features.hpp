// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/types.hpp"
#include "openMVG/multiview/triangulation.hpp"

#include <openMVG/vsslam/tracking/Abstract_Tracker.hpp>
#include <openMVG/vsslam/features/Abstract_Feature_Extractor.hpp>
#include <openMVG/vsslam/features/Abstract_Feature_Matcher.hpp>

#include <openMVG/vsslam/mapping/MapLandmark.hpp>
#include <openMVG/vsslam/optimization/PoseEstimator.hpp>

#include <openMVG/vsslam/vsslam_parameters.hpp>



namespace openMVG {
namespace vsslam {


class Tracker_Features : public Abstract_Tracker
{
private:
  /// Feature extractor
  Abstract_Feature_Extractor * feature_extractor_;
  /// Feature matcher
  Abstract_Feature_Matcher * feature_matcher_;

  Hash_Map<MapLandmark *,IndexT> track_putative_matches_frame_current;


public:
  Tracker_Features
  (
    std::shared_ptr<VSSLAM_Parameters> & params
  );

  void reset() override;

  bool isReady() override;
  void setFeatureExtractor(Abstract_Feature_Extractor * extractor)
  {
    feature_extractor_ = extractor;
  }
  void setFeatureMatcher(Abstract_Feature_Matcher * matcher)
  {
    feature_matcher_ = matcher;
  }

  bool track
  (
    const image::Image<unsigned char> & ima,
    std::shared_ptr<Frame> & frame_current,
    const image::Image<unsigned char> * mask = nullptr
  ) override;

  void endOfFrameProcedure();

  // -------------------
  // -- Initialization
  // -------------------
  bool trackingInitialization(const image::Image<unsigned char> & ima);
  void startInitialization();
  void resetInitialization();
  void clearInitializationData();

  bool performPoseOptimization(Frame * frame, Hash_Map<MapLandmark *,IndexT> & map_putative_matches_map_cur_idx, bool b_use_robust_function);
  // --------------------------
  //   Tracking:
  // --------------------------
  bool trackWithMotionModel(Hash_Map<MapLandmark *,IndexT> & map_putative_matches_landmark_frame_idx);
  bool trackWithReferenceFrame(Frame * frame_reference, Hash_Map<MapLandmark *,IndexT> & map_putative_matches_landmark_frame_idx);
  bool trackWithReferenceFrameLocalMap(Frame * frame_reference, Hash_Map<MapLandmark *,IndexT> & map_putative_matches_landmark_frame_idx);


  bool trackLocalMap();

  // --------------------------
  //   New keyframes:
  // --------------------------
  size_t needNewKeyframe
  (
    Frame * frame
  ) const;

  size_t needNewKeyframe
  (
    Frame * frame,
    size_t n_new_landmarks
  ) const;

  void printKeyFrameReason( size_t & keyframe_reason) override;

  void findNewCandidateLandmarks(Frame * frame, NewMapLandmarks & vec_new_map_landmarks);
  // -------------------
  // -- Outlier removal
  // -------------------
  void removeOutliersInMatches(Frame * frame, Hash_Map<MapLandmark *,IndexT> & matches_map_frame_idx);
  void removeOutliersInFrame(Frame * frame);
  void removeOutliersInCandidateLandmarks(Frame * frame, NewMapLandmarks & vec_new_landmarks);
  void associateLandmarksWithFrame(Frame * frame, Hash_Map<MapLandmark *,IndexT> & matches_landmarks_frame, size_t associate_type= 0);

};

}
}
