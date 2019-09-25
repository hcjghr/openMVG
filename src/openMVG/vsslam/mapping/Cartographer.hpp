// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>

#include <algorithm>
#include <openMVG/types.hpp>

#include <openMVG/vsslam/vsslam_parameters.hpp>
#include <openMVG/vsslam/vsslam_data.hpp>
#include <openMVG/vsslam/features/Abstract_Feature_Extractor.hpp>
#include <openMVG/vsslam/optimization/VSSLAM_BA.hpp>
#include <openMVG/vsslam/optimization/ceres/VSSLAM_BA_Ceres.hpp>
#include <openMVG/vsslam/optimization/slampp/VSSLAM_BA_SlamPP.hpp>
#include <openMVG/vsslam/mapping/Map.hpp>

namespace openMVG {
namespace vsslam {

class Cartographer
{
private:
  int verbose_level = 0;

  /// Parameters Object
  std::shared_ptr<VSSLAM_Parameters> params_;
  // Feature extractor
  Abstract_Feature_Extractor * feature_extractor_ = nullptr;;
  // Map optimization
  MAP_OPTIMIZATION_TYPE global_BA_type_;
  MAP_OPTIMIZATION_TYPE local_BA_type_;

  // global optimization object
  std::unique_ptr<VSSLAM_BA> BA_obj_;


  // ---------------------
  // -- Global map
  // ---------------------
  bool b_global_map_intialized_ = false;

  // Global map
  VSSLAM_Map map_global_;
  VSSLAM_Map map_local_;


public:
  // Other data
  size_t step_id_ = 0;
  Cartographer
  (
    std::shared_ptr<VSSLAM_Parameters> & params,
    const MAP_FRAME_TYPE & map_frame_type,
    const MAP_LANDMARK_TYPE map_landmark_type,
    const MAP_OPTIMIZATION_TYPE & global_BA_type,
    const MAP_OPTIMIZATION_TYPE & local_BA_type
  );

  void reset();

  void setVerboseLevel(int level)
  {
    verbose_level = level;
    std::cout<<"Cartographer: Verbose level: " << verbose_level << "\n";
  }
  
  void setFeatureExtractor(Abstract_Feature_Extractor * extractor)
  {
    feature_extractor_ = extractor;
  }

  bool isReady()
  {
    if (!feature_extractor_)
      return false;

    return true;
  }

  // -------------------
  // -- Initialization
  // -------------------
  void clearInitializationData();

  void resetInitializationData();

  bool checkInitializationConditions();

  // -------------------
  // -- Map
  // -------------------
  bool isMapInitialized()
  {
    return b_global_map_intialized_;
  }
  bool addStep
  (
    std::shared_ptr<Frame> & frame,
    NewMapLandmarks * vec_new_landmarks,
    VSSLAM_Time_Stats * stats = nullptr
  );

  void getLocalMapPoints
  (
    Frame * frame,
    std::vector<Frame*> & local_frames,
    std::vector<MapLandmark*> & local_points
  );

  void removeOutliersInLocalMapLandmarks(Frame * frame);

  void removeInactiveInLocalMapLandmarks(Frame * frame);
  // -------------------
  // -- Frames
  // -------------------
  bool addFrameToLocalMap(const std::shared_ptr<Frame> & frame, bool b_fixed_frame = false);

  bool addFrameToGlobalMap(const std::shared_ptr<Frame> & frame, bool b_fixed_frame);

  // -------------------
  // -- Landmarks
  // -------------------
  void addLandmarksToStructure
  (
    Frame * frame,
    NewMapLandmarks & vec_new_landmarks,
    const float & f_min_quality_landmark
  );

  MapLandmark * addLandmarkToLocalMap(std::unique_ptr<MapLandmark> & landmark_new);
  MapLandmark * addLandmarkToGlobalMap(std::unique_ptr<MapLandmark> & landmark_new);
  size_t addLocalLandmarksToGlobalMap(const float & f_min_quality_landmark);


  // -------------------
  // -- Observations
  // -------------------
  void addObservationsToLandmarks
  (
    Frame * frame,
    const float & f_min_quality_landmark
  );

  // -------------------
  // -- Descriptor
  // -------------------
  void updateBestLandmarkDescriptor(MapLandmark * ml);


  // ------------------------------
  // -- Map quality
  // ------------------------------
  bool checkLandmarkQuality(MapLandmark * map_landmark, const float & f_thresh_quality);

  // -------------------
  // -- Optimization
  // -------------------
  void initGlobalOptimization();

  void addObservationToIncSystem(MapLandmark * map_landmark, MapObservation * map_observation);

  void addLandmarkToIncSystem(MapLandmark * map_landamrk);

  void addFrameToIncSystem(Frame * frame, bool b_frame_fixed = false);


  bool optimizeLocalMap
  (
    Frame * frame_i,
    NewMapLandmarks & vec_new_landmarks,
    bool b_use_loss_function = true
  );

  bool optimizePose
  (
    Frame * frame_i,
    Hash_Map<MapLandmark *,IndexT> & matches_map_cur_idx,
    bool b_use_loss_function = true
  );

  void printMapStats();

  // -------------------
  // -- Export
  // -------------------
  bool exportSceneToPly(const std::string & filename, bool b_export_local_scene);

  bool exportStateSE3(std::string filename);
  bool exportDiagonalMarginals(std::string filename);


  void setMapStats(VSSLAM_Time_Stats & stats);
};

}
}
