// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include <openMVG/vsslam/optimization/slampp/VSSLAM_BA_SlamPP.hpp>


namespace openMVG {
namespace vsslam {



VSSLAM_BA_SlamPP::VSSLAM_BA_SlamPP
(
  BA_options_SlamPP options
)
:options_(options)
{
  //if (options_.b_use_loss_function_)

  problem_ = std::unique_ptr<SlamPP_Optimizer>(new SlamPP_Optimizer_Sim3_gXYZ_gXYZ(options_.undefined_cam_id, options_.b_verbose_, options_.b_use_schur_, options_.b_do_marginals_, options_.b_do_icra_style_marginals_));
  // Set initial settings
  problem_->Set_AllBatch(options_.b_all_batch);
  problem_->Set_ForceIncSchur(options_.b_force_inc_schur);
  problem_->Set_UpdateThreshold(options_.f_update_thresh);
  problem_->Set_RelinThreshold(options_.f_relin_thresh);
  problem_->Set_TrustRadius(options_.f_trust_radius);
  problem_->Set_TrustRadius_Persistence(options_.b_trust_radius_persistent);

  if (options_.b_export_graph_file)
  {
    slamPP_GraphFile.open(options_.s_graph_file.c_str(),std::ios::out );
  }

}


VSSLAM_BA_SlamPP::~VSSLAM_BA_SlamPP()
{

  // Export consistency marker
  if (options_.b_export_graph_file)
  {
    // Close the graphfile
    slamPP_GraphFile.close();
  }
}

VSSLAM_BA_SlamPP::BA_options_SlamPP & VSSLAM_BA_SlamPP::getOptions()
{
  return options_;
}

// Local optimization
bool VSSLAM_BA_SlamPP::OptimizeLocalSystem
(
  Frame * frame_i,
  NewMapLandmarks & vec_new_landmarks,
  bool b_use_loss_function,
  BA_options_SlamPP & ba_options,
  int verbose_level
)
{

  return false;
}

// Pose optimization
bool VSSLAM_BA_SlamPP::OptimizePose
(
  Frame * frame,
  Hash_Map<MapLandmark *,IndexT> & matches_map_cur_idx,
  bool b_use_loss_function,
  BA_options_SlamPP & ba_options,
  int verbose_level
)
{
  return false;
}


bool VSSLAM_BA_SlamPP::addFrameToGlobalSystem(Frame * frame, bool b_frame_fixed)
{

  // Add frame of interest
  const IndexT & frame_id = frame->getFrameId();
  const IndexT & cam_id = frame->getCamId();

  if (map_poses_.find(frame_id)!=map_poses_.end())
  {
    if (verbose_level>1)
      std::cout<<"Cartographer: [Global][BA - Slam++] Frame "<<frame->getFrameId()<< "already in the system!";
    return false;
  }

  Eigen::Matrix<double, 12, 1> vec_state_frame;
  // Get pose in the WORLD reference frame
  frame->getPose_StateVector(vec_state_frame,nullptr);

  size_t frame_slampp_id = getNextVertexId();
  //std::cout<<"Slam++: Add frame id: "<<frame_slampp_id<<"\n";
  double * ptr_state_frame;
  if (b_frame_fixed)
  {
    // Set the observing camera as fixed
    ptr_state_frame = problem_->Add_CamVertexFixed(frame_slampp_id,vec_state_frame);
  }
  else
  {
    ptr_state_frame = problem_->Add_CamVertex(frame_slampp_id,vec_state_frame);
  }
  // Add camera to map
      map_poses_[frame_id] = std::make_pair(frame_slampp_id,ptr_state_frame);

  if (verbose_level>1)
  {
    std::cout<<"Cartographer: [Global][BA - Slam++] Add frame: "<<frame->getFrameId()<< " Fixed: "<<b_frame_fixed<<" to global map!\n";
  }

  // Graph file
  if (options_.b_export_graph_file)
  {
    // Export to graph file
    Mat4 T_graph;
    frame->getPose_T(T_graph,nullptr);
    slamPP_GraphFile << "VERTEX_CAM:SIM3"
      << " " << frame_slampp_id
      << " " << T_graph.block(0,3,1,1)
      << " " << T_graph.block(1,3,1,1)
      << " " << T_graph.block(2,3,1,1)
      << " " << vec_state_frame(3)
      << " " << vec_state_frame(4)
      << " " << vec_state_frame(5)
      << " " << T_graph.block(0,0,3,1).norm()
      << " " << frame->getK()(0,0)
      << " " << frame->getK()(1,1)
      << " " << frame->getK()(0,2)
      << " " << frame->getK()(1,2)
      << " " << "0.0"
      << std::endl;
  }
  return true;


}

bool VSSLAM_BA_SlamPP::addLandmarkToGlobalSysyem(MapLandmark * map_landmark)
{
  // Create vertex for Landmark
  const size_t landmark_slampp_id = getNextVertexId();
  
  if (verbose_level>1)
  {
    std::cout<<"Cartographer: [Global][BA - Slam++] Add landmark id: "<<landmark_slampp_id<<"\n";
  }
  // Add landmarks as global point : m_undefined_camera_id as no owner
  double * landmark_ptr = problem_->Add_XYZVertex(landmark_slampp_id,options_.undefined_cam_id, map_landmark->X_);

  // Graphfile
  if (options_.b_export_graph_file)
  {
    slamPP_GraphFile << "VERTEX_XYZ"
    << " " << landmark_slampp_id
    << " " << map_landmark->X_(0)
    << " " << map_landmark->X_(1)
    << " " << map_landmark->X_(2)
    << std::endl;
  }

  // Add landmark to map
  map_landmarks_[map_landmark->id_] = std::make_pair(landmark_slampp_id,landmark_ptr);


  LandmarkObservations & vec_obs = map_landmark->getObservations();
  for(auto & m_o : vec_obs)
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
        std::cout<<"Cartographer: [Global][BA - Slam++] Adding landmark: Frame "<<frame_id<<" is not yet in the system!! Skipping landmark!";
      return false;
    }

    // Add measurement edge
    Eigen::Matrix2d inf_mat = frame->getFeatureSqrtInfMatrix(feat_id_frame).cwiseProduct(frame->getFeatureSqrtInfMatrix(feat_id_frame));
    const size_t frame_slampp_id = map_poses_.find(frame_id)->second.first;
    problem_->Add_P2CSim3GEdge(landmark_slampp_id,frame_slampp_id,frame->getFeaturePosition(feat_id_frame), inf_mat);

    //  Graphfile

    if (options_.b_export_graph_file)
    {
      slamPP_GraphFile << "EDGE_PROJECT_P2MC"
      << " " << landmark_slampp_id
      << " " << frame_slampp_id
      << " " << frame->getFeaturePosition(feat_id_frame)(0)
      << " " << frame->getFeaturePosition(feat_id_frame)(1)
      << " " << inf_mat(0,0) <<" "<< inf_mat(0,1)<<" "<<inf_mat(1,1)
      << std::endl;
    }


  }
  return true;
}

bool VSSLAM_BA_SlamPP::addObservationToGlobalSystem(MapLandmark * map_landmark, MapObservation * map_observation)
{

  Frame * frame = map_observation->frame_ptr;
  const IndexT & feat_id_frame = map_observation->feat_id;

  const IndexT & frame_id = frame->getFrameId();
  const IndexT & landmark_id = map_landmark->id_;
  const IndexT & cam_id_frame = frame->getCamId();
  IntrinsicBase * & cam_intrinsic = frame->getCameraIntrinsics();

  // Add frame to the problem if its not added yet
  if (map_poses_.find(frame_id) == map_poses_.end())
  {
    if (verbose_level>1)
      std::cout<<"Cartographer: [Global][BA - Slam++] Adding landmark: Frame "<<frame_id<<" is not yet in the system!! Skipping landmark!";
    return false;
  }

  // Check if landmark exists in the system
  if (map_landmarks_.find(landmark_id) == map_landmarks_.end() || !map_landmark->isActive())
  {
    if (verbose_level>1)
      std::cout<<"Cartographer: [Global][BA - Slam++] Adding landmark: "<<landmark_id<<" is not yet in the system!! Skipping landmark!";
    return false;
  }

  if (map_observations_.find(frame_id)!=map_observations_.end())
  {
    bool b_exists = false;
    for (auto mo : map_observations_[frame_id])
    {
      if (mo == landmark_id)
      {
        b_exists = true;
        break;
      }
    }

    if (b_exists)
    {
      if (verbose_level>1)
        std::cout<<"Cartographer: [Global][BA - Slam++] Observation exists!\n";
      return false;
    }
    else
    {
      map_observations_[frame_id].push_back(landmark_id);
    }
  }
  else
  {
    map_observations_[frame_id].push_back(landmark_id);
  }

  // Add measurement edge
  const size_t frame_slampp_id = map_poses_.find(frame_id)->second.first;
  const size_t landmark_slampp_id = map_landmarks_.find(landmark_id)->second.first;


  if (verbose_level>1)
  {
    std::cout<<"Cartographer: [Global][BA - Slam++] Add observation: landmark id: "<<landmark_slampp_id<<" frame: "<<frame_slampp_id<<"\n";
  }

  // Add measurement edge
  Eigen::Matrix2d inf_mat = frame->getFeatureSqrtInfMatrix(feat_id_frame).cwiseProduct(frame->getFeatureSqrtInfMatrix(feat_id_frame));
  problem_->Add_P2CSim3GEdge(landmark_slampp_id,frame_slampp_id,frame->getFeaturePosition(feat_id_frame), inf_mat);

  //  Graphfile

  if (options_.b_export_graph_file)
  {
    slamPP_GraphFile << "EDGE_PROJECT_P2MC"
    << " " << landmark_slampp_id
    << " " << frame_slampp_id
    << " " << frame->getFeaturePosition(feat_id_frame)(0)
    << " " << frame->getFeaturePosition(feat_id_frame)(1)
    << " " << inf_mat(0,0) <<" "<< inf_mat(0,1)<<" "<<inf_mat(1,1)
    << std::endl;
  }
  return true;
}


bool VSSLAM_BA_SlamPP::optimizeGlobal(VSSLAM_Map & map_global, VSSLAM_Time_Stats * stats)
{

  // Export consistency marker
  if (options_.b_export_graph_file)
  {
    slamPP_GraphFile << "CONSISTENCY_MARKER\n";
    // Close the graphfile
    slamPP_GraphFile.flush();
  }

  if (stats)
  {
    stats->startTimer(stats->d_feat_mapping_global_step_BA);
  }

  // Optimize the solution
  problem_->Optimize(options_.n_max_inc_iters,options_.f_inc_nlsolve_thresh, options_.f_inc_nlsolve_thresh);

  if (stats)
  {
    stats->stopTimer(stats->d_feat_mapping_global_step_BA);
    stats->startTimer(stats->d_feat_mapping_global_step_update);
  }

  // Update frames with refined data
  for (auto & frame_it : map_poses_)
  {
    const IndexT & frame_id = frame_it.first;
    Frame * frame = map_global.map_frame_[frame_id].get();

    Eigen::Map<Eigen::VectorXd> frame_state_after = problem_->r_Vertex_State(frame_it.second.first);;
    Eigen::Matrix<double, 7, 1>  vec_state = frame_state_after.template head<7>();
    frame->setPose_sim3(vec_state,frame->getReferenceFrame());
  }



  // Upate landmarks with refined data
  for (auto & landmark_it : map_landmarks_)
  {
    const IndexT & landmark_id = landmark_it.first;
    const size_t & landmark_slampp_id = landmark_it.second.first;
    // Get landmark pointer from the structure
    MapLandmark * landmark = map_global.getLandmark(landmark_id).get();

    // Recover value from slampp
    Eigen::Map<Eigen::VectorXd> landmark_state_after = problem_->r_Vertex_State(landmark_slampp_id);
    landmark->X_ = landmark_state_after;

    // Recover covariance
    landmark->cov_X_ = problem_->Get_CovarianceBlock(landmark_slampp_id,landmark_slampp_id);

    std::cout << "Landmark: " << landmark_slampp_id << "\n";
    std::cout << landmark->cov_X_ << "\n";

  }

  if (stats)
  {
    stats->stopTimer(stats->d_feat_mapping_global_step_update);
  }

  if (verbose_level>1)
    std::cout<<"Cartographer: [Global][BA - SlamPP] Optimized OK!\n";


  return true;
}

bool VSSLAM_BA_SlamPP::exportStateSE3(std::string filename)
{
  if (verbose_level>1)
    std::cout<<"Cartographer: [SlamPP] Export State SE3\n";

  problem_->Dump_State_SE3(filename.c_str());
  return true;
}

bool VSSLAM_BA_SlamPP::exportDiagonalMarginals(std::string filename)
{
  if (verbose_level>1)
    std::cout<<"Cartographer: [SlamPP] Export Diagonal Marginals\n";

  problem_->Dump_Marginals(filename.c_str());
  return true;
}

}
}
