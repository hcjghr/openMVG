// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>

#include <openMVG/types.hpp>
#include <openMVG/numeric/numeric.h>

#include <openMVG/vsslam/optimization/VSSLAM_BA.hpp>
#include <openMVG/vsslam/system/Frame.hpp>

// Ceres
#include <ceres/ceres.h>
#include <ceres/types.h>
#include <ceres/cost_function.h>

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif
// swine - otherwise causes trouble

namespace openMVG {

namespace cameras{
struct IntrinsicBase;
}

namespace vsslam {

ceres::CostFunction * IntrinsicsToCostFunction
(
  IntrinsicBase * intrinsic,
  const Vec2 & observation,
  const Eigen::Matrix<double, 2, 2> & inf_matrix,
  const bool b_sim3 = false,
  const double weight = 0.0
);

class VSSLAM_BA_Ceres : public VSSLAM_BA
{
public:
  struct BA_options_Ceres : public BA_options
  {
    unsigned int nb_threads_;
    bool b_ceres_summary_;
    ceres::LinearSolverType linear_solver_type_;
    ceres::PreconditionerType preconditioner_type_;
    ceres::SparseLinearAlgebraLibraryType sparse_linear_algebra_library_type_;
    double parameter_tolerance_;
    bool b_use_loss_function_;

    bool b_use_sim3_local = false;
    bool b_use_sim3_global = true;

    BA_options_Ceres
    (
      const MAP_FRAME_TYPE map_camera_type = MAP_FRAME_TYPE::GLOBAL,
      const MAP_LANDMARK_TYPE map_landmark_type = MAP_LANDMARK_TYPE::GLOBAL_EUCLIDEAN,
      const bool b_verbose = true, bool bmultithreaded = true
    ) : BA_options(map_camera_type, map_landmark_type, b_verbose),
    nb_threads_(1),
    parameter_tolerance_(1e-8), //~= numeric_limits<float>::epsilon()
    b_use_loss_function_(false)
    {
      #ifdef OPENMVG_USE_OPENMP
        nb_threads_ = omp_get_max_threads();
      #endif // OPENMVG_USE_OPENMP
      if (!bmultithreaded)
        nb_threads_ = 1;

      b_ceres_summary_ = false;

      // Default configuration use a DENSE representation
      linear_solver_type_ = ceres::DENSE_SCHUR;
      preconditioner_type_ = ceres::JACOBI;
      // If Sparse linear solver are available
      // Descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
      if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
      {
        sparse_linear_algebra_library_type_ = ceres::SUITE_SPARSE;
        linear_solver_type_ = ceres::SPARSE_SCHUR;
      }
      else
      {
        if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
        {
          sparse_linear_algebra_library_type_ = ceres::CX_SPARSE;
          linear_solver_type_ = ceres::SPARSE_SCHUR;
        }
        else
        if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
        {
          sparse_linear_algebra_library_type_ = ceres::EIGEN_SPARSE;
          linear_solver_type_ = ceres::SPARSE_SCHUR;
        }
      }
    }
  }; // BA_Options_Ceres
private:
  BA_options_Ceres options_;
  ceres::Problem problem_;
  // Data wrapper for global system
  Hash_Map<IndexT, std::vector<double> > map_intrinsics_;
  Hash_Map<IndexT, std::vector<double> > map_poses_;
  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction_;

  // Graphfile
  Hash_Map<IndexT,IndexT> camera_ids_omvg_slamPP;
  Hash_Map<IndexT,IndexT> track_ids_omvg_slamPP;
  Hash_Map<IndexT,IndexT> camera_ids_slamPP_omvg;
  Hash_Map<IndexT,IndexT> track_ids_slamPP_omvg;
  IndexT next_slampp_id = 0;

public:
  VSSLAM_BA_Ceres
  (
    BA_options_Ceres options = BA_options_Ceres()
  );
  ~VSSLAM_BA_Ceres();

  BA_options_Ceres & getOptions();

  // Local System
  static bool OptimizeLocalSystem
  (
    Frame * frame_i,
    NewMapLandmarks & vec_new_landmarks,
    bool b_use_loss_function,
    BA_options_Ceres & ba_options,
    int verbose_level
  );

  static bool OptimizePose
  (
    Frame * frame,
    Hash_Map<MapLandmark *,IndexT> & matches_map_cur_idx,
    bool b_use_loss_function,
    BA_options_Ceres & ba_options,
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
