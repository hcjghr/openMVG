// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres_camera_functor.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/types.hpp"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SVD>

namespace openMVG {
namespace sfm {

using namespace openMVG::cameras;
using namespace openMVG::geometry;

/// Create the appropriate cost functor according the provided input camera intrinsic model.
/// The residual can be weighetd if desired (default 0.0 means no weight).
ceres::CostFunction * IntrinsicsToCostFunction
(
  IntrinsicBase * intrinsic,
  const Vec2 & observation,
  const double weight
)
{
  switch(intrinsic->getType())
  {
    case PINHOLE_CAMERA:
        return ResidualErrorFunctor_Pinhole_Intrinsic::Create(observation, weight);
     break;
    case PINHOLE_CAMERA_RADIAL1:
      return ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K1::Create(observation, weight);
    break;
    case PINHOLE_CAMERA_RADIAL3:
      return ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K3::Create(observation, weight);
    break;
    case PINHOLE_CAMERA_BROWN:
      return ResidualErrorFunctor_Pinhole_Intrinsic_Brown_T2::Create(observation, weight);
    break;
    case PINHOLE_CAMERA_FISHEYE:
      return ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye::Create(observation, weight);
    default:
      return nullptr;
  }
}

Bundle_Adjustment_Ceres::BA_Ceres_options::BA_Ceres_options
(
  const bool bVerbose,
  bool bmultithreaded
)
: bVerbose_(bVerbose),
  nb_threads_(1),
  parameter_tolerance_(1e-8), //~= numeric_limits<float>::epsilon()
  bUse_loss_function_(true)
{
  #ifdef OPENMVG_USE_OPENMP
    nb_threads_ = omp_get_max_threads();
  #endif // OPENMVG_USE_OPENMP
  if (!bmultithreaded)
    nb_threads_ = 1;

  bCeres_summary_ = false;

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


Bundle_Adjustment_Ceres::Bundle_Adjustment_Ceres
(
  Bundle_Adjustment_Ceres::BA_Ceres_options options
)
: ceres_options_(options)
{}

Bundle_Adjustment_Ceres::BA_Ceres_options &
Bundle_Adjustment_Ceres::ceres_options()
{
  return ceres_options_;
}

bool Bundle_Adjustment_Ceres::Adjust
(
  SfM_Data & sfm_data,     // the SfM scene to refine
  const Optimize_Options options
)
{
  //----------
  // Add camera parameters
  // - intrinsics
  // - poses [R|t]

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  //----------

  ceres::Problem problem;

  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_intrinsics;
  Hash_Map<IndexT, std::vector<double> > map_poses;

  // Setup Poses data & subparametrization
 for (Poses::const_iterator itPose = sfm_data.poses.begin(); itPose != sfm_data.poses.end(); ++itPose)
  {
    const IndexT indexPose = itPose->first;

    const Pose3 & pose = itPose->second;
    const Mat3 R = pose.rotation();
    const Vec3 t = pose.translation();

    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
    // angleAxis + translation
    map_poses[indexPose] = {angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2)};

    double * parameter_block = &map_poses[indexPose][0];
    problem.AddParameterBlock(parameter_block, 6);
    if (options.extrinsics_opt == Extrinsic_Parameter_Type::NONE)
    {
      // set the whole parameter block as constant for best performance
      problem.SetParameterBlockConstant(parameter_block);
    }
    else  // Subset parametrization
    {
      std::vector<int> vec_constant_extrinsic;
      // If we adjust only the translation, we must set ROTATION as constant
      if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_TRANSLATION)
      {
        // Subset rotation parametrization
        vec_constant_extrinsic.push_back(0);
        vec_constant_extrinsic.push_back(1);
        vec_constant_extrinsic.push_back(2);
      }
      // If we adjust only the rotation, we must set TRANSLATION as constant
      if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_ROTATION)
      {
        // Subset translation parametrization
        vec_constant_extrinsic.push_back(3);
        vec_constant_extrinsic.push_back(4);
        vec_constant_extrinsic.push_back(5);
      }
      if (!vec_constant_extrinsic.empty())
      {
        ceres::SubsetParameterization *subset_parameterization =
          new ceres::SubsetParameterization(6, vec_constant_extrinsic);
        problem.SetParameterization(parameter_block, subset_parameterization);
      }
    }
  }

  // Setup Intrinsics data & subparametrization
  for (Intrinsics::const_iterator itIntrinsic = sfm_data.intrinsics.begin();
    itIntrinsic != sfm_data.intrinsics.end(); ++itIntrinsic)
  {
    const IndexT indexCam = itIntrinsic->first;

    if (isValid(itIntrinsic->second->getType()))
    {
      map_intrinsics[indexCam] = itIntrinsic->second->getParams();

      double * parameter_block = &map_intrinsics[indexCam][0];
      problem.AddParameterBlock(parameter_block, map_intrinsics[indexCam].size());
      if (options.intrinsics_opt == Intrinsic_Parameter_Type::NONE)
      {
        // set the whole parameter block as constant for best performance
        problem.SetParameterBlockConstant(parameter_block);
      }
      else
      {
        const std::vector<int> vec_constant_intrinsic =
          itIntrinsic->second->subsetParameterization(options.intrinsics_opt);
        if (!vec_constant_intrinsic.empty())
        {
          ceres::SubsetParameterization *subset_parameterization =
            new ceres::SubsetParameterization(
              map_intrinsics[indexCam].size(), vec_constant_intrinsic);
          problem.SetParameterization(parameter_block, subset_parameterization);
        }
      }
    }
    else
    {
      std::cerr << "Unsupported camera type." << std::endl;
    }
  }

  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction =
    ceres_options_.bUse_loss_function_ ?
      new ceres::HuberLoss(Square(4.0))
      : nullptr;

  // For all visibility add reprojections errors:
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks!= sfm_data.structure.end(); ++iterTracks)
  {
    const Observations & obs = iterTracks->second.obs;

    for (Observations::const_iterator itObs = obs.begin();
      itObs != obs.end(); ++itObs)
    {
      // Build the residual block corresponding to the track observation:
      const View * view = sfm_data.views.at(itObs->first).get();

      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
        IntrinsicsToCostFunction(sfm_data.intrinsics[view->id_intrinsic].get(), itObs->second.x);

      if (cost_function)
        problem.AddResidualBlock(cost_function,
          p_LossFunction,
          &map_intrinsics[view->id_intrinsic][0],
          &map_poses[view->id_pose][0],
          iterTracks->second.X.data());
    }
    if (options.structure_opt == Structure_Parameter_Type::NONE)
      problem.SetParameterBlockConstant(iterTracks->second.X.data());
  }

  if (options.control_point_opt.bUse_control_points)
  {
    // Use Ground Control Point:
    // - fixed 3D points with weighted observations
    for (Landmarks::iterator iterGCPTracks = sfm_data.control_points.begin();
      iterGCPTracks!= sfm_data.control_points.end(); ++iterGCPTracks)
    {
      const Observations & obs = iterGCPTracks->second.obs;

      for (Observations::const_iterator itObs = obs.begin();
        itObs != obs.end(); ++itObs)
      {
        // Build the residual block corresponding to the track observation:
        const View * view = sfm_data.views.at(itObs->first).get();

        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.
        ceres::CostFunction* cost_function =
          IntrinsicsToCostFunction(
            sfm_data.intrinsics[view->id_intrinsic].get(),
            itObs->second.x,
            options.control_point_opt.weight);

        if (cost_function)
          problem.AddResidualBlock(cost_function,
            nullptr,
            &map_intrinsics[view->id_intrinsic][0],
            &map_poses[view->id_pose][0],
            iterGCPTracks->second.X.data());
      }
      // Set the 3D point as FIXED (it's a GCP)
      problem.SetParameterBlockConstant(iterGCPTracks->second.X.data());
    }
  }

  // Configure a BA engine and run it
  //  Make Ceres automatically detect the bundle structure.
  ceres::Solver::Options ceres_config_options;
  ceres_config_options.preconditioner_type = ceres_options_.preconditioner_type_;
  ceres_config_options.linear_solver_type = ceres_options_.linear_solver_type_;
  ceres_config_options.sparse_linear_algebra_library_type = ceres_options_.sparse_linear_algebra_library_type_;
  ceres_config_options.minimizer_progress_to_stdout = false;
  ceres_config_options.logging_type = ceres::SILENT;
  ceres_config_options.num_threads = ceres_options_.nb_threads_;
  ceres_config_options.num_linear_solver_threads = ceres_options_.nb_threads_;
  ceres_config_options.parameter_tolerance = ceres_options_.parameter_tolerance_;

  // Solve BA
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_config_options, &problem, &summary);
  if (ceres_options_.bCeres_summary_)
    std::cout << summary.FullReport() << std::endl;

  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    if (ceres_options_.bVerbose_)
      std::cout << "Bundle Adjustment failed." << std::endl;
    return false;
  }
  else // Solution is usable
  {
    if (ceres_options_.bVerbose_)
    {
      // Display statistics about the minimization
      std::cout << std::endl
        << "Bundle Adjustment statistics (approximated RMSE):\n"
        << " #views: " << sfm_data.views.size() << "\n"
        << " #poses: " << sfm_data.poses.size() << "\n"
        << " #intrinsics: " << sfm_data.intrinsics.size() << "\n"
        << " #tracks: " << sfm_data.structure.size() << "\n"
        << " #residuals: " << summary.num_residuals << "\n"
        << " Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals) << "\n"
        << " Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals) << "\n"
        << " Time (s): " << summary.total_time_in_seconds << "\n"
        << std::endl;
    }

    // Update camera poses with refined data
    if (options.extrinsics_opt != Extrinsic_Parameter_Type::NONE)
    {
      for (Poses::iterator itPose = sfm_data.poses.begin();
        itPose != sfm_data.poses.end(); ++itPose)
      {
        const IndexT indexPose = itPose->first;

        Mat3 R_refined;
        ceres::AngleAxisToRotationMatrix(&map_poses[indexPose][0], R_refined.data());
        Vec3 t_refined(map_poses[indexPose][3], map_poses[indexPose][4], map_poses[indexPose][5]);
        // Update the pose
        Pose3 & pose = itPose->second;
        pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
      }
    }

    // Update camera intrinsics with refined data
    if (options.intrinsics_opt != Intrinsic_Parameter_Type::NONE)
    {
      for (Intrinsics::iterator itIntrinsic = sfm_data.intrinsics.begin();
        itIntrinsic != sfm_data.intrinsics.end(); ++itIntrinsic)
      {
        const IndexT indexCam = itIntrinsic->first;

        const std::vector<double> & vec_params = map_intrinsics[indexCam];
        itIntrinsic->second.get()->updateFromParams(vec_params);
      }
    }
    // Structure is already updated directly if needed (no data wrapping)
    return true;
  }
}




Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< Eigen::MatrixXd > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().transpose();
}
Eigen::SparseMatrix<double, Eigen::RowMajor> pseudoInverse(const Eigen::SparseMatrix<double, Eigen::RowMajor> &a, double epsilon = std::numeric_limits<double>::epsilon()){
  Eigen::MatrixXd dense_a = Eigen::MatrixXd(a);
  Eigen::MatrixXd dense_a_inv = pseudoInverse(dense_a);
  Eigen::SparseMatrix<double, Eigen::RowMajor> sparse_a_inv =  dense_a_inv.sparseView();
  sparse_a_inv.makeCompressed();
  return sparse_a_inv;
}


bool Bundle_Adjustment_Ceres::EstimateUncertainty
(
  SfM_Data & sfm_data,     // the SfM scene to refine
  const Optimize_Options options,
  const bool evaluateLandmarks
)
{
  //----------
  // Add camera parameters
  // - intrinsics
  // - poses [R|t]

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  //----------

  ceres::Problem problem;

  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_intrinsics;
  Hash_Map<IndexT, std::vector<double> > map_poses;

  // Setup Poses data & subparametrization
 for (Poses::const_iterator itPose = sfm_data.poses.begin(); itPose != sfm_data.poses.end(); ++itPose)
  {
    const IndexT indexPose = itPose->first;

    const Pose3 & pose = itPose->second;
    const Mat3 R = pose.rotation();
    const Vec3 t = pose.translation();

    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
    // angleAxis + translation
    map_poses[indexPose] = {angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2)};

    double * parameter_block = &map_poses[indexPose][0];
    problem.AddParameterBlock(parameter_block, 6);
    if (options.extrinsics_opt == Extrinsic_Parameter_Type::NONE)
    {
      // set the whole parameter block as constant for best performance
      problem.SetParameterBlockConstant(parameter_block);
    }
    else  // Subset parametrization
    {
      std::vector<int> vec_constant_extrinsic;
      // If we adjust only the translation, we must set ROTATION as constant
      if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_TRANSLATION)
      {
        // Subset rotation parametrization
        vec_constant_extrinsic.push_back(0);
        vec_constant_extrinsic.push_back(1);
        vec_constant_extrinsic.push_back(2);
      }
      // If we adjust only the rotation, we must set TRANSLATION as constant
      if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_ROTATION)
      {
        // Subset translation parametrization
        vec_constant_extrinsic.push_back(3);
        vec_constant_extrinsic.push_back(4);
        vec_constant_extrinsic.push_back(5);
      }
      if (!vec_constant_extrinsic.empty())
      {
        ceres::SubsetParameterization *subset_parameterization =
          new ceres::SubsetParameterization(6, vec_constant_extrinsic);
        problem.SetParameterization(parameter_block, subset_parameterization);
      }
    }
  }

  // Setup Intrinsics data & subparametrization
  for (Intrinsics::const_iterator itIntrinsic = sfm_data.intrinsics.begin();
    itIntrinsic != sfm_data.intrinsics.end(); ++itIntrinsic)
  {
    const IndexT indexCam = itIntrinsic->first;

    if (isValid(itIntrinsic->second->getType()))
    {
      map_intrinsics[indexCam] = itIntrinsic->second->getParams();

      double * parameter_block = &map_intrinsics[indexCam][0];
      problem.AddParameterBlock(parameter_block, map_intrinsics[indexCam].size());
      if (options.intrinsics_opt == Intrinsic_Parameter_Type::NONE)
      {
        // set the whole parameter block as constant for best performance
        problem.SetParameterBlockConstant(parameter_block);
      }
      else
      {
        const std::vector<int> vec_constant_intrinsic =
          itIntrinsic->second->subsetParameterization(options.intrinsics_opt);
        if (!vec_constant_intrinsic.empty())
        {
          ceres::SubsetParameterization *subset_parameterization =
            new ceres::SubsetParameterization(
              map_intrinsics[indexCam].size(), vec_constant_intrinsic);
          problem.SetParameterization(parameter_block, subset_parameterization);
        }
      }
    }
    else
    {
      std::cerr << "Unsupported camera type." << std::endl;
    }
  }

  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction =
    ceres_options_.bUse_loss_function_ ?
      new ceres::HuberLoss(Square(4.0))
      : nullptr;

  // For all visibility add reprojections errors:
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks!= sfm_data.structure.end(); ++iterTracks)
  {
    const Observations & obs = iterTracks->second.obs;

    for (Observations::const_iterator itObs = obs.begin();
      itObs != obs.end(); ++itObs)
    {
      // Build the residual block corresponding to the track observation:
      const View * view = sfm_data.views.at(itObs->first).get();

      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
        IntrinsicsToCostFunction(sfm_data.intrinsics[view->id_intrinsic].get(), itObs->second.x);

      if (cost_function)
        problem.AddResidualBlock(cost_function,
          p_LossFunction,
          &map_intrinsics[view->id_intrinsic][0],
          &map_poses[view->id_pose][0],
          iterTracks->second.X.data());
    }
    if (options.structure_opt == Structure_Parameter_Type::NONE)
      problem.SetParameterBlockConstant(iterTracks->second.X.data());
  }

  if (options.control_point_opt.bUse_control_points)
  {
    // Use Ground Control Point:
    // - fixed 3D points with weighted observations
    for (Landmarks::iterator iterGCPTracks = sfm_data.control_points.begin();
      iterGCPTracks!= sfm_data.control_points.end(); ++iterGCPTracks)
    {
      const Observations & obs = iterGCPTracks->second.obs;

      for (Observations::const_iterator itObs = obs.begin();
        itObs != obs.end(); ++itObs)
      {
        // Build the residual block corresponding to the track observation:
        const View * view = sfm_data.views.at(itObs->first).get();

        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.
        ceres::CostFunction* cost_function =
          IntrinsicsToCostFunction(
            sfm_data.intrinsics[view->id_intrinsic].get(),
            itObs->second.x,
            options.control_point_opt.weight);

        if (cost_function)
          problem.AddResidualBlock(cost_function,
            nullptr,
            &map_intrinsics[view->id_intrinsic][0],
            &map_poses[view->id_pose][0],
            iterGCPTracks->second.X.data());
      }
      // Set the 3D point as FIXED (it's a GCP)
      problem.SetParameterBlockConstant(iterGCPTracks->second.X.data());
    }
  }


  // Confgure problem evaluator
  ceres::Problem::EvaluateOptions evaluate_options_;
  evaluate_options_.num_threads = ceres_options_.nb_threads_;
  ceres::CRSMatrix jacobian;
  double cost;
  // Evaluate problem
  problem.Evaluate(evaluate_options_, &cost, NULL, NULL, &jacobian);

  // -----------------------------------------------
  // Convert Ceres Jacobian to Sparse Eigen Matrix
  // -----------------------------------------------
  typedef Eigen::SparseMatrix<double, Eigen::RowMajor> EigenSparseMatrix;
  // Convert the matrix to column major order
  EigenSparseMatrix sparse_jacobian =
      Eigen::MappedSparseMatrix<double, Eigen::RowMajor>(
          jacobian.num_rows, jacobian.num_cols,
          static_cast<int>(jacobian.values.size()),
          jacobian.rows.data(), jacobian.cols.data(), jacobian.values.data());
          
  // -----------------------------------------------
  // Compute size of parameters
  // -----------------------------------------------
  // Basic sizes of parameters
  const int single_cam_ext_param = 6;
  // Compute the size of different parameter blocks
  const int total_observations = sparse_jacobian.rows();
  const int total_cam_ext_param = single_cam_ext_param*sfm_data.poses.size();
  const int total_landmark_param = 3*sfm_data.structure.size();
  const int total_control_param = 3*sfm_data.control_points.size();
  int total_intrinsic_param = 0;
  // Save the index of the start of next intrinsic block of params
  Hash_Map<IndexT, int> start_intrinsic_param_per_row;
  for (Intrinsics::const_iterator itIntrinsic = sfm_data.intrinsics.begin();
    itIntrinsic != sfm_data.intrinsics.end(); ++itIntrinsic)
  {
    const int n_intrinsic_param = map_intrinsics[itIntrinsic->first].size();
    // Save the start index of the intrinsics
    start_intrinsic_param_per_row[itIntrinsic->first] = total_intrinsic_param;
    // Compute total number of intrinsics
    total_intrinsic_param += n_intrinsic_param;
  }
    
  // -----------------------------------------------
  // Extract Jacobian matrix relating cameras and landmarks
  // -----------------------------------------------
  EigenSparseMatrix J_A = sparse_jacobian.block(0,0,total_observations,total_cam_ext_param + total_intrinsic_param);
  J_A.makeCompressed();
  EigenSparseMatrix J_B = sparse_jacobian.block(0,total_cam_ext_param + total_intrinsic_param,total_observations,total_landmark_param + total_control_param);
  J_B.makeCompressed();
  
  
  // -----------------------------------------------
  // Compute E_x_inv - Covariance matrix of detected features
  // -----------------------------------------------
  if (ceres_options_.bVerbose_)
  {
    std::cout<<"Computing E_x_inv\n";
  }
  
  EigenSparseMatrix E_x_inv(total_observations,total_observations);
  
  {
  // Assume all features with same covariance -> 2px
  Eigen::Matrix2d Ex;
  Ex << 0.5,0,0,0.5;
  IndexT obsID=0;
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks!= sfm_data.structure.end(); ++iterTracks)
  {
    const Observations & obs = iterTracks->second.obs;
    for (Observations::const_iterator itObs = obs.begin();
      itObs != obs.end(); ++itObs)
    {
	  for(int r=0;r<2;r++){
	    for(int c=0;c<2;c++){
	      E_x_inv.insert(obsID*2+r,obsID*2+c) = Ex(r,c);
	    }
	  }
	  obsID++;
	}
  }
  }
  
  // -----------------------------------------------
  // Compute U
  // ----------------------------------------------- 
  if (ceres_options_.bVerbose_)
  { 
    std::cout<<"Computing U\n";  
  }
  
  EigenSparseMatrix U = (J_A.transpose() * E_x_inv * J_A).pruned(0.0);
  U.makeCompressed();
  
  // -----------------------------------------------
  // Compute V_inverse
  // ----------------------------------------------- 
  if (ceres_options_.bVerbose_)
  { 
    std::cout<<"Computing V_inverse\n";
  }  
  
  EigenSparseMatrix V_inv = (J_B.transpose() * E_x_inv * J_B).pruned(0.0);
  V_inv.makeCompressed();
  
  // Invert diagonal blocks
  {
  Eigen::Matrix3d diag;
  for(int t_id=0;t_id<V_inv.rows();t_id+=3){
      diag = V_inv.block(t_id,t_id,3,3);
      diag = diag.inverse().eval();      
      for(int r=0;r<3;r++){
        for(int c=0;c<3;c++){
          V_inv.coeffRef(t_id+r,t_id+c) = diag(r,c);
        }
      }
  }
  }
  V_inv.makeCompressed();
  
  // -----------------------------------------------
  // Compute W
  // ----------------------------------------------- 
  if (ceres_options_.bVerbose_)
  { 
    std::cout<<"Computing W\n";
  }
  
  EigenSparseMatrix W = (J_A.transpose() * E_x_inv * J_B).pruned(0.0);
  W.makeCompressed();
  

  // -----------------------------------------------
  // Compute Y and W * V_inverse * W'
  // ----------------------------------------------- 
  if (ceres_options_.bVerbose_)
  { 
    std::cout<<"Computing Y and W * V_inv * W'\n";
  }
  
  EigenSparseMatrix Y = (W * V_inv).pruned(0.0);
  Y.makeCompressed();
  EigenSparseMatrix WVW = (Y * W.transpose()).pruned(0.0);
  WVW.makeCompressed();
  
  // -----------------------------------------------
  // Compute E_A
  // ----------------------------------------------- 
  if (ceres_options_.bVerbose_)
  { 
    std::cout<<"Computing E_A\n";
  }  
  
  EigenSparseMatrix E_A = pseudoInverse(EigenSparseMatrix(U-WVW));
  E_A.makeCompressed();
  
  sfm_data.uncertainty_poses_intrinsics.covariance = E_A;

  // -----------------------------------------------
  // Compute E_B
  // -----------------------------------------------
  //Eigen::MatrixXd E_B = Eigen::MatrixXd::Zero(total_landmark_param + total_control_param,3);
  if(evaluateLandmarks){
    if (ceres_options_.bVerbose_)
    { 
      std::cout<<"Computing E_B\n";
    }

    EigenSparseMatrix E_B_sparse = (Y.transpose() * (E_A) * Y).pruned(0.0);
    E_B_sparse.makeCompressed();
    for(int o_i=0;o_i<(total_landmark_param + total_control_param)/3;o_i++){
      UncertaintyLandmark un_landmark;
      un_landmark.covariance = E_B_sparse.block(o_i*3,o_i*3,3,3);
      //E_B.block(o_i*3,0,3,3) = E_B_sparse.block(o_i*3,o_i*3,3,3);
      sfm_data.uncertainty_structure[o_i] = un_landmark;
    }
  }
  //sfm_data.uncertainty_structure = E_B;
  return true;
}


} // namespace sfm
} // namespace openMVG

