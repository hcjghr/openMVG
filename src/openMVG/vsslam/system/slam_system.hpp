// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once


#include <openMVG/vsslam/vsslam_parameters.hpp>
#include <openMVG/vsslam/system/Frame.hpp>
#include <openMVG/vsslam/tracking/Abstract_Tracker.hpp>
#include <openMVG/vsslam/features/Abstract_Feature_Extractor.hpp>
#include <openMVG/vsslam/features/Abstract_Feature_Matcher.hpp>
#include <openMVG/vsslam/mapping/Cartographer.hpp>

namespace openMVG {
namespace vsslam {

class SLAM_System
{
private:
  int verbose_level = 0;
  std::shared_ptr<VSSLAM_Parameters> params_;
  // Map of cameras
  Hash_Map<IndexT, std::shared_ptr<Camera> > map_cameras_;

  // Tracker
  std::unique_ptr<Abstract_Tracker> tracker_;
  std::unique_ptr<Abstract_Feature_Extractor> feat_extractor_;
  std::unique_ptr<Abstract_Feature_Matcher> feat_matcher_;

  // Tracker
  std::unique_ptr<Cartographer> cartographer_;

  // Frames
  std::shared_ptr<Frame> frame_current_;
public:
  // Constructor
  SLAM_System(std::shared_ptr<VSSLAM_Parameters> & params);

  ~SLAM_System();

  void prepareStatsFile();
  void setVerboseLevel(int level);
  // Set up tracker
  void setTracker(std::unique_ptr<Abstract_Tracker> & tracker);
  void setFeatureExtractor(std::unique_ptr<Abstract_Feature_Extractor> & extractor);
  void setFeatureMatcher(std::unique_ptr<Abstract_Feature_Matcher> & matcher);

  bool createCartographer
  (
    MAP_FRAME_TYPE map_frame_type,
    MAP_LANDMARK_TYPE map_landmark_type,
    MAP_OPTIMIZATION_TYPE global_BA_type,
    MAP_OPTIMIZATION_TYPE local_BA_type
  );

  // Check
  bool isReady();

  bool isMapInitialized()
  {
    return cartographer_->isMapInitialized();
  }

  // Insert a camera together with a mask image for processing
  IndexT createCamera(const CameraParameters & param_cam);
  IndexT createCamera(const CameraParameters & param_cam, image::Image<unsigned char> & mask);
  bool addMaskImageToCamera(const IndexT & id_cam, image::Image<unsigned char> & mask);

  // Process images
  bool nextFrame
  (
    const image::Image<unsigned char> & ima,
    const IndexT & id_frame,
    const IndexT & id_cam,
    const double & time_frame
  );

  Frame * getCurrentFramePtr()
  {
    return tracker_->getCurrentFramePtr();
  }

  void printCameraParameters(IndexT & cam_id);

  void printStatistics(VSSLAM_Time_Stats & stats);


  void disableOutput();
  void setOutputDirectory(std::string s_output_folder);
  void setIntermediateResultOutput(bool b_intermediate_output);

  void exportStatisticsHeader();
  void exportStatistics(VSSLAM_Time_Stats & stats);
};

}
}
