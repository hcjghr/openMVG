// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>
#include <cstdlib>

#include <openMVG/types.hpp>
#include <openMVG/numeric/numeric.h>

#include <openMVG/vsslam/optimization/VSSLAM_BA.hpp>
#include <openMVG/vsslam/system/Frame.hpp>


#include <openMVG/vsslam/optimization/slampp/optimizers/SlamPP_Optimizer_Sim3_gXYZ_gXYZ.hpp>




namespace openMVG {
namespace vsslam {


class VSSLAM_BA_SlamPP : public VSSLAM_BA
{
public:
  struct BA_options_SlamPP : public BA_options
  {
    size_t undefined_cam_id = size_t(-1);//std::numeric_limits<size_t>::max() / 2 + 1;

    // Slam++ settings
    bool b_all_batch = false;
    bool b_force_inc_schur = false;

    bool b_use_schur_ = true;
    bool b_do_marginals_ = true;
    bool b_do_icra_style_marginals_ = false;
    size_t n_skip_optimization = 0; //Optimize every n included cameras (0 - means every)

    /*
     The default nonlinear solve threshold is 0.005, the default final number of iterations
     is 5, the default update threshold is 10^-4 and the default trust radius is 0.2.
     */

    double f_update_thresh = 0.0001;//10^(-4);  //dx-threshold  (using update threshold 0, disabling update threshold)
    double f_relin_thresh = 0.0;
    double f_trust_radius = 0.2;  // trust-radius
    bool b_trust_radius_persistent = false;
    int n_max_inc_iters = 5;  //max-nonlinear-solve-iters

    int n_max_final_iters = 5;  //max-final-nonlinear-solve-iters
    double f_inc_nlsolve_thresh = .005; //nonlinear-solve-error-thresh
    double f_final_nlsolve_thresh = .005; //final-nonlinear-solve-error-thresh
    
    

    bool b_use_loss_function_;


    BA_options_SlamPP
    (
      const MAP_FRAME_TYPE map_camera_type = MAP_FRAME_TYPE::GLOBAL,
      const MAP_LANDMARK_TYPE map_landmark_type = MAP_LANDMARK_TYPE::GLOBAL_EUCLIDEAN,
      const bool b_verbose = true, bool bmultithreaded = true
    ) : BA_options(map_camera_type, map_landmark_type, b_verbose),
    b_use_loss_function_(false)
    {
    }
  }; // BA_Options_SlamPP
private:
  BA_options_SlamPP options_;
  std::unique_ptr<SlamPP_Optimizer> problem_;
  Hash_Map<IndexT, std::pair<size_t,double *> > map_poses_;
  Hash_Map<IndexT, std::pair<size_t,double *> > map_landmarks_;
  Hash_Map<IndexT, std::vector<IndexT> > map_observations_;

  size_t next_vertex_idx_slampp_ = 0;

  size_t getNextVertexId()
  {
    return next_vertex_idx_slampp_++;
  }

public:
  VSSLAM_BA_SlamPP
  (
    BA_options_SlamPP options = BA_options_SlamPP()
  );

  ~VSSLAM_BA_SlamPP();

  BA_options_SlamPP & getOptions();

  // Local System
  static bool OptimizeLocalSystem
  (
    Frame * frame_i,
    NewMapLandmarks & vec_new_landmarks,
    bool b_use_loss_function,
    BA_options_SlamPP & ba_options,
    int verbose_level
  );

  static bool OptimizePose
  (
    Frame * frame,
    Hash_Map<MapLandmark *,IndexT> & matches_map_cur_idx,
    bool b_use_loss_function,
    BA_options_SlamPP & ba_options,
    int verbose_level
  );

  bool addObservationToGlobalSystem(MapLandmark * map_point, MapObservation * map_observation)override;
  bool addLandmarkToGlobalSysyem(MapLandmark * map_point)override;
  bool addFrameToGlobalSystem(Frame * frame, bool b_frame_fixed = false)override;
  bool optimizeGlobal(VSSLAM_Map & map_global, VSSLAM_Time_Stats * stats = nullptr)override;

  bool exportStateSE3(std::string filepath)override;
  bool exportDiagonalMarginals(std::string filepath)override;

};

}
}
