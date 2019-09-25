// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include "openMVG/types.hpp"
#include <openMVG/vsslam/vsslam_parameters.hpp>
#include <openMVG/vsslam/vsslam_data.hpp>
#include <openMVG/vsslam/system/Frame.hpp>
#include <openMVG/vsslam/tracking/MotionModel.hpp>
#include <openMVG/vsslam/mapping/Cartographer.hpp>

#include <openMVG/vsslam/display/vsslam_display.hpp>
#include <openMVG/vsslam/display/vsslam_time_stats.hpp>

extern openMVG::vsslam::VSSLAM_Display display_data;
extern openMVG::vsslam::VSSLAM_Time_Stats time_data;

namespace openMVG {
namespace vsslam {

class Abstract_Tracker
{
protected:
  int verbose_level = 0;  // 0 - none; 1 - stats; 2 - all

  /// Parameters Object
  std::shared_ptr<VSSLAM_Parameters> params_;
  /// Tracking status
  TRACKING_STATUS tracking_status_ = TRACKING_STATUS::NOT_INIT;
  IndexT frame_last_relocalization_id = UndefinedIndexT;
  IndexT frame_last_ok_frame_id = UndefinedIndexT;


  MotionModel motion_model_;
  // Tracking frames
  std::shared_ptr<Frame> frame_track_prev;
  std::shared_ptr<Frame> frame_track_current;
  std::shared_ptr<Frame> frame_track_last_reference;
  


  // Initialization obj
  std::shared_ptr<Frame> frame_track_init;

  Cartographer * cartographer_;

public:
  virtual ~Abstract_Tracker(){};

  virtual bool isReady() = 0;

  virtual void printKeyFrameReason( size_t & keyframe_reason)=0;

  virtual void reset()=0;
  
  void setVerboseLevel(int level)
  {
    verbose_level = level;
    std::cout<<"Tracker: Verbose level: " << verbose_level << "\n";
  }
  
  void setCartographer(Cartographer * cartographer)
  {
    cartographer_ = cartographer;
  }

  Frame* getCurrentFramePtr()
  {
    return frame_track_current.get();
  }
  virtual bool track
  (
    const image::Image<unsigned char> & ima,
    std::shared_ptr<Frame> & frame_current,
    const image::Image<unsigned char> * mask = nullptr
  ) = 0;

  void printTrackingStatus()
  {
    std::cout<<"VSSLAM [System] Tracker status: ";
    switch (tracking_status_)
    {
    case TRACKING_STATUS::OK:
      std::cout<<"OK\n";
      break;
    case TRACKING_STATUS::LOST:
      std::cout<<"LOST\n";
      break;
    case TRACKING_STATUS::INIT:
      std::cout<<"INIT\n";
      break;
    case TRACKING_STATUS::NOT_INIT:
      std::cout<<"NOT_INIT\n";
      break;
    case TRACKING_STATUS::IDLE:
      std::cout<<"IDLE\n";
      break;
    }
  }

  TRACKING_STATUS getTrackingStatus()
  {
    return tracking_status_;
  }

  IndexT getLastGoodFrameID()
  {
    return frame_last_ok_frame_id;
  }
  void setLastGoodFrameID(IndexT frame_id)
  {
    frame_last_ok_frame_id = frame_id;
  }
};

}
}
