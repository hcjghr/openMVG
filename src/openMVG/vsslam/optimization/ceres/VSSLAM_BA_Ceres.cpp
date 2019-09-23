// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include <openMVG/vsslam/optimization/ceres/VSSLAM_BA_Ceres.hpp>
#include <openMVG/vsslam/optimization/ceres/VSSLAM_BA_Ceres_camera_functor.hpp>


namespace openMVG {
namespace vsslam {


/// Create the appropriate cost functor according the provided input camera intrinsic model.
/// The residual can be weighetd if desired (default 0.0 means no weight).
ceres::CostFunction * IntrinsicsToCostFunction
(
  IntrinsicBase * intrinsic,
  const Vec2 & observation,
  const Eigen::Matrix<double, 2, 2> & inf_matrix,
  const bool b_sim3,
  const double weight
)
{
  switch(intrinsic->getType())
  {
    case PINHOLE_CAMERA:
      //return ResidualErrorFunctor_Pinhole_Intrinsic_Rts::Create(observation, weight);
      //return ResidualErrorFunctor_Pinhole_Intrinsic_Rts::Create(observation, inf_matrix, weight);
      if (b_sim3)
      {
        return Chi2ErrorFunctor_Pinhole_Intrinsic_Rts::Create(observation, inf_matrix, weight);
      }
      else
      {
        return Chi2ErrorFunctor_Pinhole_Intrinsic_Rt::Create(observation, inf_matrix, weight);
      }
     break;
    default:
      return nullptr;
  }

}

VSSLAM_BA_Ceres::VSSLAM_BA_Ceres
(
  BA_options_Ceres options
)
:options_(options)
{
  if (options_.b_use_loss_function_)
    p_LossFunction_ = new ceres::HuberLoss(5.991);
  else
    p_LossFunction_ = nullptr;

  if (options_.b_export_graph_file)
  {
    slamPP_GraphFile.open(options_.s_graph_file.c_str(),std::ios::out );
  }
}

VSSLAM_BA_Ceres::~VSSLAM_BA_Ceres()
{

  // Export consistency marker
  if (options_.b_export_graph_file)
  {
    // Close the graphfile
    slamPP_GraphFile.close();
  }
}

VSSLAM_BA_Ceres::BA_options_Ceres & VSSLAM_BA_Ceres::getOptions()
{
  return options_;
}

// Local optimization
bool VSSLAM_BA_Ceres::OptimizeLocalSystem
(
  Frame * frame_i,
  NewMapLandmarks & vec_new_landmarks,
  bool b_use_loss_function,
  BA_options_Ceres & ba_options,
  int verbose_level
)
{
  if (!frame_i)
  {
    return true;
  }

  if (verbose_level>1)
  {
    std::cout<<"Cartographer: [Local][BA - Ceres] Optimize local map\n";
    std::cout<<"Cartographer: [Local][BA - Ceres] Use sim3: " << ba_options.b_use_sim3_local << "\n";
  }

  ceres::Problem problem;

  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_intrinsics;
  Hash_Map<IndexT, std::vector<double> > map_poses;

  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction;
  if (b_use_loss_function)
    //p_LossFunction = new ceres::HuberLoss(frame_i->reproj_thresh_sq_);
    p_LossFunction = new ceres::HuberLoss(5.991);
  else
    p_LossFunction = nullptr;

  // Pose parameters
  Mat3 R;
  double angleAxis[3];
  Vec3 t;
  double s;

  // Add frame of interest
  const IndexT & frame_i_id = frame_i->getFrameId();
  const IndexT & cam_i_idx = frame_i->getCamId();
  IntrinsicBase * & cam_i_intrinsic = frame_i->getCameraIntrinsics();

  // Get pose in the WORLD reference frame
  frame_i->getPose_sRt_Inverse(R,t,s,nullptr);

  // Convert rotation matrix to angleaxis representation
  ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
  // Save data about the poses
  map_poses[frame_i_id] = {angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2),s};
  // Add pose data to problem
  double * parameter_block = &map_poses[frame_i_id][0];
  problem.AddParameterBlock(parameter_block, 7);
  // Add intrinsic parameters
  map_intrinsics[cam_i_idx] = frame_i->getCameraIntrinsics()->getParams();
  parameter_block = &map_intrinsics[cam_i_idx][0];
  problem.AddParameterBlock(parameter_block, map_intrinsics[cam_i_idx].size());
  // If camera is calibrated we fix the parameters
  if (frame_i->isCamCalibrated())
    problem.SetParameterBlockConstant(parameter_block);


  // Impose 3D-2D matches that are already associated with frame_i
  // Global landmarks are fixed
  // Local landmarks are free -> we add all cameras involved as fixed
  std::vector<MapLandmark *>  & landmarks_frame_i = frame_i->getLandmarks();
  for (size_t mp_i = 0; mp_i < landmarks_frame_i.size(); ++mp_i)
  {
    MapLandmark * & map_landmark = landmarks_frame_i[mp_i];
    if (!map_landmark)
      continue;

    if (map_landmark->isActive())
    {
      // Add measurement in frame_i to landmark
      // Create the cost function for the measurement
      ceres::CostFunction* cost_function = IntrinsicsToCostFunction(cam_i_intrinsic,
          frame_i->getFeaturePosition(mp_i),
          frame_i->getFeatureSqrtInfMatrix(mp_i),
          ba_options.b_use_sim3_local);


      // Add cost term
      if (cost_function)
      {
        problem.AddResidualBlock(cost_function,
          p_LossFunction,
          &map_intrinsics[cam_i_idx][0],
          &map_poses[frame_i_id][0],
          map_landmark->X_.data());
        // If landmark is global we fix it
        problem.SetParameterBlockConstant(map_landmark->X_.data());
      }
    }
    else
    {
      // If local map point we add all other cameras that see the point
      for (auto & m_o : map_landmark->getObservations())
      {
        Frame * & frame_obs = m_o.second.frame_ptr;

        // Frame of interest (current frame)
        const IndexT & frame_obs_id = frame_obs->getFrameId();
        const IndexT & cam_obs_idx = frame_obs->getCamId();
        IntrinsicBase * & cam_obs_intrinsic = frame_obs->getCameraIntrinsics();

        // Add frame to the problem if its not added yet
        if (map_poses.find(frame_obs_id) == map_poses.end())
        {
          // Get pose in the WORLD reference frame
          frame_obs->getPose_sRt_Inverse(R,t,s,nullptr);
          // Convert rotation matrix to angleaxis representation
          ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
          // Save data about the poses
          map_poses[frame_obs_id] = {angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2),s};
          // Add parameter to the block
          double * parameter_block = &map_poses[frame_obs_id][0];
          problem.AddParameterBlock(parameter_block, 7);
          // Set the observing camera as fixed
          problem.SetParameterBlockConstant(parameter_block);

          if (map_intrinsics.find(cam_obs_idx) == map_intrinsics.end())
          {
            // Add intrinsic parameters
            map_intrinsics[cam_obs_idx] = frame_obs->getCameraIntrinsics()->getParams();
            parameter_block = &map_intrinsics[cam_obs_idx][0];
            problem.AddParameterBlock(parameter_block, map_intrinsics[cam_obs_idx].size());
            // Fix the calibration parameteres (even if the camera is not calibrated - we are not changing this cam)
            problem.SetParameterBlockConstant(parameter_block);
          }
        }

        // Create the cost function for the measurement
        ceres::CostFunction* cost_function = IntrinsicsToCostFunction(cam_obs_intrinsic,
            frame_obs->getFeaturePosition(m_o.second.feat_id),
            frame_obs->getFeatureSqrtInfMatrix(m_o.second.feat_id),
            ba_options.b_use_sim3_local);

        // Add cost term
        if (cost_function)
        {
          problem.AddResidualBlock(cost_function,
            p_LossFunction,
            &map_intrinsics[cam_obs_idx][0],
            &map_poses[frame_obs_id][0],
            map_landmark->X_.data());
        }
      }
    }
  }

  // Add newly triangulated points (3d points are free)
  // Cameras added are fixed as they are already global
  if(!vec_new_landmarks.empty())
  {
    for (std::unique_ptr<MapLandmark> & ptr_new_landmark : vec_new_landmarks)
    {
      LandmarkObservations & obs = ptr_new_landmark->getObservations();
      for(auto & m_o : obs)
      {
        Frame * frame_obs = m_o.second.frame_ptr;
        const IndexT & frame_obs_id = frame_obs->getFrameId();
        const IndexT & cam_obs_idx = frame_obs->getCamId();
        IntrinsicBase * & cam_obs_intrinsic = frame_obs->getCameraIntrinsics();

        // Add frame to the problem if its not added yet
        if (map_poses.find(frame_obs_id) == map_poses.end())
        {
          // Get pose in the WORLD reference frame
          frame_obs->getPose_sRt_Inverse(R,t,s,nullptr);
          // Convert rotation matrix to angleaxis representation
          ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
          // Save data about the poses
          map_poses[frame_obs_id] = {angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2),s};
          // Add parameter to the block
          double * parameter_block = &map_poses[frame_obs_id][0];
          problem.AddParameterBlock(parameter_block, 7);
          // Set the observing camera as fixed
          problem.SetParameterBlockConstant(parameter_block);

          if (map_intrinsics.find(cam_obs_idx) == map_intrinsics.end())
          {
            // Add intrinsic parameters
            map_intrinsics[cam_obs_idx] = frame_obs->getCameraIntrinsics()->getParams();
            parameter_block = &map_intrinsics[cam_obs_idx][0];
            problem.AddParameterBlock(parameter_block, map_intrinsics[cam_obs_idx].size());
            // Fix the calibration parameteres (even if the camera is not calibrated - we are not changing this cam)
            problem.SetParameterBlockConstant(parameter_block);
          }
        }

        // Create the cost function for the measurement
        ceres::CostFunction* cost_function = IntrinsicsToCostFunction(cam_obs_intrinsic,
            frame_obs->getFeaturePosition(m_o.second.feat_id),
            frame_obs->getFeatureSqrtInfMatrix(m_o.second.feat_id),
            ba_options.b_use_sim3_local);

        // Add cost term
        if (cost_function)
        {
          problem.AddResidualBlock(cost_function,
            p_LossFunction,
            &map_intrinsics[cam_obs_idx][0],
            &map_poses[frame_obs_id][0],
            ptr_new_landmark->X_.data());
        }
      }
    }
  }


  // Configure a BA engine and run it
  //  Make Ceres automatically detect the bundle structure.
  ceres::Solver::Options ceres_config_options;
  //ceres_config_options.max_num_iterations = 500;
  ceres_config_options.preconditioner_type = ba_options.preconditioner_type_;
  ceres_config_options.linear_solver_type = ba_options.linear_solver_type_;
  ceres_config_options.sparse_linear_algebra_library_type = ba_options.sparse_linear_algebra_library_type_;
  ceres_config_options.minimizer_progress_to_stdout = ba_options.b_verbose_;
  ceres_config_options.logging_type = ceres::SILENT;
  ceres_config_options.num_threads = ba_options.nb_threads_;
  ceres_config_options.num_linear_solver_threads = ba_options.nb_threads_;
  ceres_config_options.parameter_tolerance = ba_options.parameter_tolerance_;

  // Solve BA
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_config_options, &problem, &summary);
  if (ba_options.b_ceres_summary_)
    std::cout << summary.FullReport() << std::endl;

  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    if (verbose_level>1)
    {
      if (ba_options.b_verbose_)
        std::cout << "Bundle Adjustment failed." << std::endl;
    }
    return false;
  }
  else // Solution is usable
  {
    if (verbose_level>1)
    {
      if (ba_options.b_verbose_)
      {
        // Display statistics about the minimization
        std::cout << std::endl
          << "Bundle Adjustment statistics (approximated RMSE):\n"
          << " #keyframes: " << map_poses.size() << "\n"
          << " #intrinsics: " << map_intrinsics.size() << "\n"
          //<< " #points: " << slam_data.structure.size() << "\n"
          << " #residuals: " << summary.num_residuals << "\n"
          << " Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals) << "\n"
          << " Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals) << "\n"
          << " Time (s): " << summary.total_time_in_seconds << "\n"
          << std::endl;
      }
    }

    // Update camera poses with refined data
    Mat3 R_refined;
    // Get frame and update values
    IndexT frame_id = frame_i->getFrameId();
    ceres::AngleAxisToRotationMatrix(&(map_poses[frame_id][0]), R_refined.data());
    Vec3 t_refined = Vec3(map_poses[frame_id][3], map_poses[frame_id][4], map_poses[frame_id][5]);
    double s_refined = map_poses[frame_id][6];

    // Update the pose
    frame_i->setPose_sRt_Inverse(R_refined,t_refined,s_refined, nullptr);

    return true;
  }
}

// Pose optimization
bool VSSLAM_BA_Ceres::OptimizePose
(
  Frame * frame,
  Hash_Map<MapLandmark *,IndexT> & matches_map_cur_idx,
  bool b_use_loss_function,
  BA_options_Ceres & ba_options,
  int verbose_level
)
{
  if (!frame || matches_map_cur_idx.empty())
  {
    return true;
  }

  if (verbose_level>1)
    std::cout<<"Cartographer: [Local][BA - Ceres] Optimize pose\n";

  // Create the local problem
  ceres::Problem problem;

  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_intrinsics;
  Hash_Map<IndexT, std::vector<double> > map_poses;

  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction;
  if (b_use_loss_function)
    //p_LossFunction = new ceres::HuberLoss(frame_i->reproj_thresh_sq_);
    p_LossFunction = new ceres::HuberLoss(5.991);
  else
    p_LossFunction = nullptr;

  // Pose parameters
  Mat3 R;
  double angleAxis[3];
  Vec3 t;
  double s;

  // Add frame of interest
  const IndexT & frame_id = frame->getFrameId();
  const IndexT & cam_idx = frame->getCamId();
  IntrinsicBase * & cam_intrinsic = frame->getCameraIntrinsics();

  // Get pose in the WORLD reference frame
  frame->getPose_sRt_Inverse(R,t,s,nullptr);
  // Convert rotation matrix to angleaxis representation
  ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
  // Save data about the poses
  map_poses[frame_id] = {angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2),s};
  // Add pose data to problem
  double * parameter_block = &map_poses[frame_id][0];
  problem.AddParameterBlock(parameter_block, 7);
  // Add intrinsic parameters
  map_intrinsics[cam_idx] = cam_intrinsic->getParams();
  parameter_block = &map_intrinsics[cam_idx][0];
  problem.AddParameterBlock(parameter_block, map_intrinsics[cam_idx].size());
  // If camera is calibrated we fix the parameters
  if (frame->isCamCalibrated())
    problem.SetParameterBlockConstant(parameter_block);

  // Impose 3D-2D matches that are already associated with frame
  // Global landmarks are fixed
  std::vector<MapLandmark *>  & vec_landmarks_frame = frame->getLandmarks();

  for (IndexT mp_i = 0; mp_i < vec_landmarks_frame.size(); ++mp_i)
  {
    MapLandmark * & map_landmark = vec_landmarks_frame[mp_i];
    if (!map_landmark)
      continue;
    // Add measurement in frame_i to landmark
    // Create the cost function for the measurement

    // TODO: Check IntrinsicsToCostFunction
    ceres::CostFunction* cost_function = IntrinsicsToCostFunction(cam_intrinsic,
        frame->getFeaturePosition(mp_i),
        frame->getFeatureSqrtInfMatrix(mp_i),
        ba_options.b_use_sim3_local);

    // Add cost term
    if (cost_function)
    {
      problem.AddResidualBlock(cost_function,
        p_LossFunction,
        &map_intrinsics[cam_idx][0],
        &map_poses[frame_id][0],
        map_landmark->X_.data());

      // If landmark is global we fix it
      problem.SetParameterBlockConstant(map_landmark->X_.data());
    }

  }


  // Add all 3D-2D matchings -> they are fixed
  for (auto & match : matches_map_cur_idx)
  {
    MapLandmark * map_landmark = match.first;
    // Skip as we already added it in the previous section
    if (frame->getLandmark(match.second))
      continue;

    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    
    ceres::CostFunction* cost_function = IntrinsicsToCostFunction(cam_intrinsic,
                                                                  frame->getFeaturePosition(match.second),
                                                                  frame->getFeatureSqrtInfMatrix(match.second),
                                                                  ba_options.b_use_sim3_local);

    if (cost_function)
    {
      problem.AddResidualBlock(cost_function,
        p_LossFunction,
        &map_intrinsics[cam_idx][0],
        &map_poses[frame_id][0],
        map_landmark->X_.data());
      // Set structure fixed
        problem.SetParameterBlockConstant(map_landmark->X_.data());
    }
  }

  // Configure a BA engine and run it
  //  Make Ceres automatically detect the bundle structure.
  ceres::Solver::Options ceres_config_options;
  //ceres_config_options.max_num_iterations = 500;
  ceres_config_options.preconditioner_type = ba_options.preconditioner_type_;
  ceres_config_options.linear_solver_type = ba_options.linear_solver_type_;
  ceres_config_options.sparse_linear_algebra_library_type = ba_options.sparse_linear_algebra_library_type_;
  ceres_config_options.minimizer_progress_to_stdout = ba_options.b_verbose_;
  ceres_config_options.logging_type = ceres::SILENT;
  ceres_config_options.num_threads = ba_options.nb_threads_;
  ceres_config_options.num_linear_solver_threads = ba_options.nb_threads_;
  ceres_config_options.parameter_tolerance = ba_options.parameter_tolerance_;

  // Solve BA
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_config_options, &problem, &summary);
  if (ba_options.b_ceres_summary_)
    std::cout << summary.FullReport() << std::endl;

  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    if (verbose_level>1)
    {
      if (ba_options.b_verbose_)
        std::cout << "Bundle Adjustment failed." << std::endl;
    }
    return false;
  }
  else // Solution is usable
  {
    if (verbose_level>1)
    {
      if (ba_options.b_verbose_)
      {
        // Display statistics about the minimization
        std::cout << std::endl
          << "Bundle Adjustment statistics (approximated RMSE):\n"
          << " #keyframes: " << map_poses.size() << "\n"
          << " #intrinsics: " << map_intrinsics.size() << "\n"
          //<< " #points: " << slam_data.structure.size() << "\n"
          << " #residuals: " << summary.num_residuals << "\n"
          << " Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals) << "\n"
          << " Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals) << "\n"
          << " Time (s): " << summary.total_time_in_seconds << "\n"
          << std::endl;
      }
    }

    // Update camera poses with refined data
    Mat3 R_refined;
    // Get frame and update values
    ceres::AngleAxisToRotationMatrix(&(map_poses[frame_id][0]), R_refined.data());
    Vec3 t_refined = Vec3(map_poses[frame_id][3], map_poses[frame_id][4], map_poses[frame_id][5]);
    double s_refined = map_poses[frame_id][6];

    // Update the pose
    frame->setPose_sRt_Inverse(R_refined,t_refined,s_refined, nullptr);

    return true;
  }
}


bool VSSLAM_BA_Ceres::addFrameToGlobalSystem(Frame * frame, bool b_frame_fixed)
{

  // Pose parameters
  Mat3 R;
  double angleAxis[3];
  Vec3 t;
  double s;

  // Add frame of interest
  const IndexT & frame_id = frame->getFrameId();
  const IndexT & cam_id = frame->getCamId();

  if (map_poses_.find(frame_id)!=map_poses_.end())
  {
    if (verbose_level>1)
      std::cout<<"Cartographer: [Global][BA - Ceres] Frame "<<frame->getFrameId()<< "already in the global system!";
    return false;
  }

  // Get pose in the WORLD reference frame
  frame->getPose_sRt_Inverse(R,t,s,nullptr);
  // Convert rotation matrix to angleaxis representation
  ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
  // Save data about the poses
  map_poses_[frame_id] = {angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2),s};
  // Add pose data to problem
  double * parameter_block = &map_poses_[frame_id][0];
  problem_.AddParameterBlock(parameter_block, 7);

  if (b_frame_fixed)
    problem_.SetParameterBlockConstant(parameter_block);

  if (map_intrinsics_.find(cam_id)==map_intrinsics_.end())
  {
    map_intrinsics_[cam_id] = frame->getCameraIntrinsics()->getParams();
    parameter_block = &map_intrinsics_[cam_id][0];
    problem_.AddParameterBlock(parameter_block, map_intrinsics_[cam_id].size());
    if (frame->isCamCalibrated())
    {
      problem_.SetParameterBlockConstant(parameter_block);
    }
  }


  if (options_.b_export_graph_file)
  {
    // Export to graph file
    Mat4 T_graph;
    frame->getPose_T(T_graph,nullptr);
    R = T_graph.block(0,0,3,3);
    ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);

    // Get slam++ id
    IndexT frame_slampp_id = next_slampp_id;
    next_slampp_id++;
    // Add index to map
    camera_ids_omvg_slamPP[frame->getFrameId()] = frame_slampp_id;

    slamPP_GraphFile << "VERTEX_CAM:SIM3"
      << " " << frame_slampp_id
      << " " << T_graph.block(0,3,1,1)
      << " " << T_graph.block(1,3,1,1)
      << " " << T_graph.block(2,3,1,1)
      << " " << angleAxis[0]
      << " " << angleAxis[1]
      << " " << angleAxis[2]
      << " " << T_graph.block(0,0,3,1).norm()
      << " " << frame->getK()(0,0)
      << " " << frame->getK()(1,1)
      << " " << frame->getK()(0,2)
      << " " << frame->getK()(1,2)
      << " " << "0.0"
      << std::endl;
  }


  if (verbose_level>1)
    std::cout<<"Cartographer: [Global][BA - Ceres] Add frame: "<<frame->getFrameId()<< " Fixed: "<<b_frame_fixed<<" to global map!\n";
  return true;
}

bool VSSLAM_BA_Ceres::addLandmarkToGlobalSysyem(MapLandmark * map_landmark)
{
  LandmarkObservations & obs = map_landmark->getObservations();

  // Graphfile

  if (options_.b_export_graph_file)
  {
    // Get slam++ id
    IndexT landmark_slampp_id = next_slampp_id;
    next_slampp_id++;
    // Add index to map
    track_ids_omvg_slamPP[map_landmark->id_] = landmark_slampp_id;
    track_ids_slamPP_omvg[landmark_slampp_id] = map_landmark->id_;

    slamPP_GraphFile << "VERTEX_XYZ"
    << " " << landmark_slampp_id
    << " " << map_landmark->X_(0)
    << " " << map_landmark->X_(1)
    << " " << map_landmark->X_(2)
    << std::endl;

  }

  for(auto & m_o : obs)
  {
    Frame * frame = m_o.second.frame_ptr;
    const IndexT & feat_id_frame = m_o.second.feat_id;

    const IndexT & frame_id = frame->getFrameId();
    const IndexT & cam_id_frame = frame->getCamId();
    IntrinsicBase * & cam_intrinsic = frame->getCameraIntrinsics();


    // Add frame to the problem if its not added yet
    if (map_poses_.find(frame_id) == map_poses_.end())
    {
      if (verbose_level>1)
        std::cout<<"Cartographer: [Global][BA - Ceres] Adding landmark: Frame "<<frame_id<<" is not yet in the system!! Skipping landmark!";
      return false;
    }

    ceres::CostFunction* cost_function = IntrinsicsToCostFunction(cam_intrinsic,
      frame->getFeaturePosition(feat_id_frame),
      frame->getFeatureSqrtInfMatrix(feat_id_frame),
      options_.b_use_sim3_global);

    if (cost_function)
    {
      ceres::ResidualBlockId residual_id = problem_.AddResidualBlock(cost_function,
        p_LossFunction_,
        &map_intrinsics_[cam_id_frame][0],
        &map_poses_[frame_id][0],
        map_landmark->X_.data());

      //  Graphfile

      if (options_.b_export_graph_file)
      {
        Eigen::Matrix2d inf_mat = frame->getFeatureSqrtInfMatrix(feat_id_frame).cwiseProduct(frame->getFeatureSqrtInfMatrix(feat_id_frame));
        IndexT frame_slampp_id = camera_ids_omvg_slamPP[frame->getFrameId()];
        IndexT landmark_slampp_id = track_ids_omvg_slamPP[map_landmark->id_];
        slamPP_GraphFile << "EDGE_PROJECT_P2MC"
        << " " << landmark_slampp_id
        << " " << frame_slampp_id
        << " " << frame->getFeaturePosition(feat_id_frame)(0)
        << " " << frame->getFeaturePosition(feat_id_frame)(1)
        << " " << inf_mat(0,0) <<" "<< inf_mat(0,1)<<" "<<inf_mat(1,1)
        << std::endl;
      }
    }
    else
    {
      if (verbose_level>1)
        std::cout<<"Cartographer: [Global][BA - Ceres] Adding landmark: "<<map_landmark->id_<<" cost function error!! Skipping landmark!";
    }

  }

  return true;
}

bool VSSLAM_BA_Ceres::addObservationToGlobalSystem(MapLandmark * map_landmark, MapObservation * map_observation)
{
  Frame * frame = map_observation->frame_ptr;
  const IndexT & feat_id_frame = map_observation->feat_id;

  const IndexT & frame_id = frame->getFrameId();
  const IndexT & cam_id_frame = frame->getCamId();
  IntrinsicBase * & cam_intrinsic = frame->getCameraIntrinsics();


  // Add frame to the problem if its not added yet
  if (map_poses_.find(frame_id) == map_poses_.end())
  {
    if (verbose_level>1)
      std::cout<<"Cartographer: [Global][BA - Ceres] Adding landmark: Frame "<<frame_id<<" is not yet in the system!! Skipping landmark!";
    return false;
  }

  ceres::CostFunction* cost_function = IntrinsicsToCostFunction(cam_intrinsic,
    frame->getFeaturePosition(feat_id_frame),
    frame->getFeatureSqrtInfMatrix(feat_id_frame),
    options_.b_use_sim3_global);

  if (cost_function)
  {
    ceres::ResidualBlockId residual_id = problem_.AddResidualBlock(cost_function,
      p_LossFunction_,
      &map_intrinsics_[cam_id_frame][0],
      &map_poses_[frame_id][0],
      map_landmark->X_.data());

    //  Graphfile

    if (options_.b_export_graph_file)
    {
      Eigen::Matrix2d inf_mat = frame->getFeatureSqrtInfMatrix(feat_id_frame).cwiseProduct(frame->getFeatureSqrtInfMatrix(feat_id_frame));
      IndexT frame_slampp_id = camera_ids_omvg_slamPP[frame->getFrameId()];
      IndexT landmark_slampp_id = track_ids_omvg_slamPP[map_landmark->id_];
      slamPP_GraphFile << "EDGE_PROJECT_P2MC"
      << " " << landmark_slampp_id
      << " " << frame_slampp_id
      << " " << frame->getFeaturePosition(feat_id_frame)(0)
      << " " << frame->getFeaturePosition(feat_id_frame)(1)
      << " " << inf_mat(0,0) <<" "<< inf_mat(0,1)<<" "<<inf_mat(1,1)
      << std::endl;
    }
  }
  else
  {
    if (verbose_level>1)
      std::cout<<"Cartographer: [Global][BA - Ceres] Adding landmark: "<<map_landmark->id_<<" cost function error!! Skipping landmark!";
  }
  return true;
}

bool VSSLAM_BA_Ceres::optimizeGlobal(VSSLAM_Map & map_global, VSSLAM_Time_Stats * stats)
{
  // Export consistency marker
  if (options_.b_export_graph_file)
  {
    slamPP_GraphFile << "CONSISTENCY_MARKER\n";
    // Close the graphfile
    slamPP_GraphFile.flush();
  }

  // Configure a BA engine and run it
  //  Make Ceres automatically detect the bundle structure.
  ceres::Solver::Options ceres_config_options;
  ceres_config_options.max_num_iterations = 500;
  ceres_config_options.preconditioner_type = options_.preconditioner_type_;
  ceres_config_options.linear_solver_type = options_.linear_solver_type_;
  ceres_config_options.sparse_linear_algebra_library_type = options_.sparse_linear_algebra_library_type_;
  ceres_config_options.minimizer_progress_to_stdout = options_.b_verbose_;
  ceres_config_options.logging_type = ceres::SILENT;
  ceres_config_options.num_threads = options_.nb_threads_;
  ceres_config_options.num_linear_solver_threads = options_.nb_threads_;
  ceres_config_options.parameter_tolerance = options_.parameter_tolerance_;


  if (stats)
  {
    stats->startTimer(stats->d_feat_mapping_global_step_BA);
  }
  // Solve BA
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_config_options, &problem_, &summary);

  if (stats)
  {
    stats->stopTimer(stats->d_feat_mapping_global_step_BA);
  }

  if (options_.b_ceres_summary_)
    std::cout << summary.FullReport() << std::endl;

  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    if (verbose_level>1)
    {
      if (options_.b_verbose_)
        std::cout << "Bundle Adjustment failed." << std::endl;
    }
    return false;
  }
  else // Solution is usable
  {
    if (verbose_level>1)
    {
      if (options_.b_verbose_)
      {
        // Display statistics about the minimization
        std::cout << std::endl
          << "Bundle Adjustment statistics (approximated RMSE):\n"
          << " #keyframes: " << map_poses_.size() << "\n"
          << " #intrinsics: " << map_intrinsics_.size() << "\n"
          //<< " #points: " << slam_data.structure.size() << "\n"
          << " #residuals: " << summary.num_residuals << "\n"
          << " Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals) << "\n"
          << " Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals) << "\n"
          << " Time (s): " << summary.total_time_in_seconds << "\n"
          << std::endl;
      }
    }

    if (stats)
    {
      stats->startTimer(stats->d_feat_mapping_global_step_update);
    }
    // Recover camera data

    // Update camera poses with refined data
    Mat3 R_refined; Vec3 t_refined; double s_refined;

    for (auto pose_it : map_poses_)
    {
      const IndexT frame_id = pose_it.first;

      // Get frame and update values
      Frame * frame = map_global.map_frame_[frame_id].get();

      ceres::AngleAxisToRotationMatrix(&(map_poses_[frame_id][0]), R_refined.data());
      t_refined = Vec3(map_poses_[frame_id][3], map_poses_[frame_id][4], map_poses_[frame_id][5]);
      s_refined = map_poses_[frame_id][6];

      // Update the pose
      frame->setPose_sRt_Inverse(R_refined,t_refined,s_refined, nullptr);

      if (verbose_level>1)
        std::cout<<"Cartographer: [BA - Ceres][Global] Frame: "<<frame->getFrameId()<<" updated!\n";

    }

    if (stats)
    {
      stats->stopTimer(stats->d_feat_mapping_global_step_update);
    }

    return true;
  }
}

bool VSSLAM_BA_Ceres::exportStateSE3(std::string p_s_filename)
{
  std::cout << "Export State SE3 with Ceres NOT IMPLEMENTED\n";
  return true;
}

bool VSSLAM_BA_Ceres::exportDiagonalMarginals(std::string p_s_filename)
{
  std::cout << "Export State SE3 with Ceres NOT IMPLEMENTED\n";
  return true;
}

}
}
