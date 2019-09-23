// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>

#include <openMVG/types.hpp>

#include <openMVG/vsslam/vsslam_data.hpp>
#include <openMVG/vsslam/mapping/Map.hpp>
#include <openMVG/vsslam/mapping/MapLandmark.hpp>

namespace openMVG {
namespace vsslam {

class VSSLAM_BA
{
protected:
  //graph file
  std::ofstream slamPP_GraphFile;
  int verbose_level = false;
public:
  struct BA_options
  {
    bool b_verbose_;
    MAP_LANDMARK_TYPE map_landmark_type_;
    MAP_FRAME_TYPE map_camera_type_;

    bool b_export_graph_file = false;
    std::string s_graph_file = "";

    BA_options
    (
      const MAP_FRAME_TYPE map_camera_type = MAP_FRAME_TYPE::GLOBAL,
      const MAP_LANDMARK_TYPE map_landmark_type = MAP_LANDMARK_TYPE::GLOBAL_EUCLIDEAN,
      const bool b_verbose = true
    ): b_verbose_(b_verbose), map_camera_type_(map_camera_type), map_landmark_type_(map_landmark_type){};
  };

  virtual ~VSSLAM_BA() = default;


  virtual bool addObservationToGlobalSystem(MapLandmark * map_point, MapObservation * map_observation) =0;
  virtual bool addLandmarkToGlobalSysyem(MapLandmark * map_point) =0;
  virtual bool addFrameToGlobalSystem(Frame * frame, bool b_frame_fixed) =0;
  virtual bool optimizeGlobal(VSSLAM_Map & map_global, VSSLAM_Time_Stats * stats) =0;


  virtual bool exportStateSE3(std::string filepath) =0;
  virtual bool exportDiagonalMarginals(std::string filepath) =0;


  void setVerboseLevel(int level)
  {
    verbose_level = level;
    std::cout<<"VSSLAM BA: Verbose level: " << verbose_level << "\n";
  }
public:
};

}
}
