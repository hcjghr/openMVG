// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <openMVG/vsslam/optimization/VSSLAM_BA.hpp>
#include <openMVG/vsslam/optimization/ceres/VSSLAM_BA_Ceres.hpp>
#include <openMVG/vsslam/mapping/Cartographer.hpp>

namespace openMVG {
namespace vsslam {

  Cartographer::Cartographer
  (
    std::shared_ptr<VSSLAM_Parameters> & params,
    const MAP_FRAME_TYPE & map_frame_type,
    const MAP_LANDMARK_TYPE map_landmark_type,
    const MAP_OPTIMIZATION_TYPE & global_BA_type,
    const MAP_OPTIMIZATION_TYPE & local_BA_type
  )
  {
    params_ = params->share_ptr();

    map_global_.map_frame_type_ = map_frame_type;
    map_global_.map_landmark_type_ = map_landmark_type;
    map_local_.map_frame_type_ = map_frame_type;
    map_local_.map_landmark_type_ = map_landmark_type;

    // Create BA optimization objects
    global_BA_type_ = global_BA_type;
    local_BA_type_ = local_BA_type;
    initGlobalOptimization();



  }

  // -------------------
  // -- Initialization
  // -------------------
  void Cartographer::clearInitializationData()
  {
  }

  void Cartographer::resetInitializationData()
  {
    clearInitializationData();
    map_global_.clear();
    map_local_.clear();
    b_global_map_intialized_ = false;
  }

  bool Cartographer::checkInitializationConditions()
  {
    // Go through local map and check if we have enough points with sufficient quality
    // and all frames see enough landmarks
    std::map<Frame *, size_t> n_obs_per_frame;
    size_t n_landmarks_ok = 0;

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Initialization Map] Checking map conditions\n";

    for (auto & map_landmark_local : map_local_.map_landmarks_)
    {
      MapLandmark * map_landmark = map_landmark_local.second.get();
      if (checkLandmarkQuality(map_landmark, params_->map_min_quality_landmark))
      {
        n_landmarks_ok++;
        for (auto obs_it: map_landmark->getObservations())
        {
          Frame * frame = obs_it.second.frame_ptr;
          if (n_obs_per_frame.find(frame) == n_obs_per_frame.end())
            n_obs_per_frame[frame] = 1;
          else
            n_obs_per_frame[frame] = n_obs_per_frame[frame] + 1;
        }
      }
    }

    if (n_landmarks_ok < params_->map_min_init_pts)
    {
      if (verbose_level > 1)
        std::cout<<"Cartographer: [Initialization Map] Failed: Insufficient quality local landmarks: "<<n_landmarks_ok<<"\n";
      return false;
    }

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Initialization Map] Quality local landmarks: "<<n_landmarks_ok<<"\n";

    bool b_frames_ok = true;
    for (auto & frame_local : map_local_.map_frame_)
    {
      Frame * frame = frame_local.second.get();
      if (n_obs_per_frame.find(frame) == n_obs_per_frame.end())
      {
        b_frames_ok = false;
        break;
      }
      else if(n_obs_per_frame[frame] < params_->map_min_obs_per_frame)
        b_frames_ok = false;
        break;
    }

    if (!b_frames_ok)
    {
      if (verbose_level > 1)
        std::cout<<"Cartographer: [Initialization Map] Failed: Frames are not well defined!\n";
      return false;
    }

    if (verbose_level > 1)
    {
      printMapStats();
    }

    return true;
  }


  double d_feat_mapping_global_step_prepare = 0.0;
  double d_feat_mapping_global_step_BA = 0.0;
  double d_feat_mapping_global_step_update = 0.0;
  // -------------------
  // -- Map
  // -------------------
  bool Cartographer::addStep
  (
    std::shared_ptr<Frame> & frame,
    NewMapLandmarks * vec_new_landmarks,
    VSSLAM_Time_Stats * stats
  )
  {
    bool b_step_ok = false;
    const IndexT & frame_id = frame->getFrameId();

    // Increase step
    step_id_++;

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Step #" << step_id_ << "] Frame id: " << frame_id << "\n";

    if (stats)
    {
      stats->global_step_id = step_id_;
    }

    if (!b_global_map_intialized_)
    {
      if (verbose_level > 1)
        std::cout<<"Cartographer: [Step #" << step_id_ << "] Initialization\n";

      if (stats)
      {
        stats->startTimer(stats->d_feat_mapping_global_step_prepare);
      }
      
      // Add frame to local map
      addFrameToLocalMap(frame, false);

      // Add observations to local structure
      addObservationsToLandmarks(frame.get(), params_->map_min_quality_landmark);

      // If we have new landmarks
      if (vec_new_landmarks)
      {
        addLandmarksToStructure(frame.get(),*vec_new_landmarks,  params_->map_min_quality_landmark);
      }

      // Show some stats
      if (verbose_level > 1)
      {
        printMapStats();
      }
      // Try to initialize if enough frames
      if (map_local_.getNumberOfFrames() < params_->map_min_frame_init)
      {
        if (stats)
        {
          stats->stopTimer(stats->d_feat_mapping_global_step_prepare);
        }
        return false;
      }
      else
      {
        // Once we have enough frames we try to initialize map
        if (verbose_level > 1)
          std::cout<<"Cartographer: [Initialization] Try to initialize map in step " << step_id_ << "\n";

        if (!checkInitializationConditions())
        {
          if (verbose_level > 1)
            std::cout<<"Cartographer: [Initialization] Map initialization failed!\n";

          if (stats)
          {
            stats->stopTimer(stats->d_feat_mapping_global_step_prepare);
          }

          resetInitializationData();
          return false;
        }

        // --------------------
        // -- Add frames to global map
        // --------------------
        for (auto it_frame : map_local_.map_frame_)
        {
          const size_t id_frame = it_frame.first;
          addFrameToGlobalMap(it_frame.second,it_frame.second->isActive());

          // Remove frame from local map
          map_local_.removeFrame(id_frame);
        }

        // --------------------
        // -- Add landmarks which satisfy min_quality_threshold add from local to global
        // --------------------
        size_t n_map_landmarks_ok = addLocalLandmarksToGlobalMap(params_->map_min_quality_landmark);

        if (n_map_landmarks_ok < params_->map_min_init_pts)
        {
          if (verbose_level > 1)
            std::cout<<"Cartographer: [Initialization Map] Failed: Insufficient quality local landmarks: "<<n_map_landmarks_ok<<"\n";

          if (stats)
          {
            stats->stopTimer(stats->d_feat_mapping_global_step_prepare);
          }

          resetInitializationData();
          return false;
        }

        if (stats)
        {
          stats->stopTimer(stats->d_feat_mapping_global_step_prepare);
        }

        // Globally optimize the system
        if (verbose_level > 1)
          std::cout<<"Cartographer: [Step #" << step_id_ << "] Global optimization\n";

        // Perform global optimization
        bool b_global_ok = BA_obj_->optimizeGlobal(map_global_, stats);

        if (!b_global_ok)
        {
          if (verbose_level > 1)
            std::cout<<"Cartographer: [Initialization Map] Failed: Initial global optimization falied!\n";

          resetInitializationData();
          return false;

        }
        b_global_map_intialized_ = true;

        // Clear all unnecessary data
        clearInitializationData();

        if (verbose_level > 1)
        {
          // Show some stats
          printMapStats();
          std::cout<<"Cartographer: [Initialization Map] Success\n";
        }
        
        return true;
      }
    }
    else
    {
      if (verbose_level > 1)
        std::cout<<"Cartographer: [Step #" << step_id_ << "] Regular\n";

      if (stats)
      {
        stats->startTimer(stats->d_feat_mapping_global_step_prepare);
      }

      // Add frame to local map
      addFrameToGlobalMap(frame, frame->isActive());
      
      // Add observations to local structure
      addObservationsToLandmarks(frame.get(), params_->map_min_quality_landmark);

      // If we have new landmarks
      if (vec_new_landmarks)
      {
        addLandmarksToStructure(frame.get(),*vec_new_landmarks,  params_->map_min_quality_landmark);
      }

      // Check if we have more points that have been sufficiently observed to add to the system
      addLocalLandmarksToGlobalMap(params_->map_min_quality_landmark);

      if (verbose_level > 1)
      {
        // Show some stats
        printMapStats();
      }

      if (stats)
      {
        stats->stopTimer(stats->d_feat_mapping_global_step_prepare);
      }

      // Globally optimize the system
      if (verbose_level > 1)
        std::cout<<"Cartographer: [Step #" << step_id_ << "] Global optimization\n";

      // Perform global optimization
      bool b_global_ok = BA_obj_->optimizeGlobal(map_global_, stats);

      return b_global_ok;
    }
  }

  void Cartographer::printMapStats()
  {
    std::cout<<"Cartographer: [Map stats] Global Frames/Points: "<<map_global_.getNumberOfFrames()<<" / "<<map_global_.getNumberOfLandmarks()<<"\n"
            <<"Cartographer: [Map stats] Local Frames/Points: "<<map_local_.getNumberOfFrames()<<" / "<<map_local_.getNumberOfLandmarks()<<"\n";
  }

  void Cartographer::setMapStats(VSSLAM_Time_Stats & stats)
  {
    stats.global_frames = map_global_.getNumberOfFrames();
    stats.global_landmarks = map_global_.getNumberOfLandmarks();
    stats.local_frames = map_local_.getNumberOfFrames();
    stats.local_landmarks = map_local_.getNumberOfLandmarks();
  }

  void Cartographer::getLocalMapPoints(Frame * frame, std::vector<Frame*> & local_frames, std::vector<MapLandmark*> & local_points)
  {
    const IndexT & frame_id = frame->getFrameId();
    const std::vector<MapLandmark *> & vec_landmarks_frame = frame->getLandmarks();

    std::vector<MapLandmark *> vec_changed_ids;

    // Get all already reconstructed points from all the frames
    for(Frame * & frame_i : local_frames)
    {
      for(MapLandmark * & map_landmark: frame_i->getLandmarks())
      {
        // Check that we did not include it yet
        if (!map_landmark)
          continue;
        if (map_landmark->last_local_map_frame_id_ == frame_id)
          continue;

        // Mark landmark as inspected
        map_landmark->last_local_map_frame_id_ = frame_id;
        vec_changed_ids.push_back(map_landmark);

        // Check that the point is not already used in current frame
        /*bool b_found = false;
        for (auto map_landmark_frame : vec_landmarks_frame)
        {
          if (map_landmark_frame == map_landmark)
          {
            b_found = true;
            break;
          }
        }*/
        //if(!b_found)
        //  local_points.push_back(map_landmark);

        if (std::find(vec_landmarks_frame.begin(), vec_landmarks_frame.end(), map_landmark) == vec_landmarks_frame.end())
        {
          // Add to locak map
          local_points.push_back(map_landmark);
        }
      }
    }

    // Reset local counter
    for (auto map_landmark : vec_changed_ids)
    {
      map_landmark->last_local_map_frame_id_ = UndefinedIndexT;
    }
  }



  // -------------------
  // -- Frames
  // -------------------
  bool Cartographer::addFrameToLocalMap(const std::shared_ptr<Frame> & frame, bool b_fixed_frame)
  {
    if (verbose_level > 1)
      std::cout<<"Cartographer: [Augment Map] Add local frame with id "<<frame->getFrameId()<<"!\n";

    return map_local_.addFrame(frame);
  }

  bool Cartographer::addFrameToGlobalMap(const std::shared_ptr<Frame> & frame, bool b_fixed_frame)
  {
    bool b_ok = map_global_.addFrame(frame);

    if (!b_ok)
    {
      return false;
    }
    // Set frame as active
    frame->setActive();

    // Add new frame to global system
    addFrameToIncSystem(frame.get(), b_fixed_frame);

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Augment Map] Add global frame with id "<<frame->getFrameId()<<"!\n";

    return true;
  }



  // -------------------
  // -- Landmarks
  // -------------------
  void Cartographer::updateBestLandmarkDescriptor(MapLandmark * map_landmark)
  {
    // Set best descriptor as the last observation
    // TODO: Decide which is optimal
    MapObservation & m_o = (map_landmark->getObservations().rbegin())->second;

    feature_extractor_->getDescriptorRaw(m_o.frame_ptr->getRegions(),m_o.feat_id,& map_landmark->getBestDesc());
  }

  void Cartographer::addLandmarksToStructure
  (
    Frame * frame,
    NewMapLandmarks & vec_new_landmarks,
    const float & f_min_quality_landmark
  )
  {
    const IndexT frame_id = frame->getFrameId();

    size_t n_local_landmarks = 0; 
    size_t n_global_landmarks = 0; 
    for (std::unique_ptr<MapLandmark> & landmark_new : vec_new_landmarks)
    {
      MapLandmark * map_landmark;

      if (b_global_map_intialized_ && checkLandmarkQuality(landmark_new.get(), landmark_new->hasFrameObservation(frame_id) ? f_min_quality_landmark : params_->map_min_quality_landmark))
      {
        // Add landmark to global map
        map_landmark = addLandmarkToGlobalMap(landmark_new);
        n_global_landmarks++;

        if (time_data.b_enable_features_stats)
        {
          time_data.added_global_landmarks++;
        }
      }
      else
      {
        // Add landmark to local map
        map_landmark = addLandmarkToLocalMap(landmark_new);
        n_local_landmarks++;

        // Mark local landmark as seen in this step
        map_landmark->setObservedInStep(step_id_);

        if (time_data.b_enable_features_stats)
        {
          time_data.added_local_landmarks++;
        }

      }
      // Update the best descriptor
      updateBestLandmarkDescriptor(map_landmark);


      // Create connections between frame and map_point
      for (auto & obs: map_landmark->getObservations())
      {
        MapObservation & mo = obs.second;
        mo.frame_ptr->setLandmark(mo.feat_id, map_landmark);
      }
    }

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Augment Map] Add " << n_global_landmarks << " / " << n_local_landmarks << " (global/local) landmarks to structure from frame with id "<<frame_id<<"\n";
  }

  MapLandmark * Cartographer::addLandmarkToLocalMap(std::unique_ptr<MapLandmark> & landmark_new)
  {
    // Add landmark to local map
    MapLandmark * ptr_map_landmark = map_local_.addLandmark(landmark_new);

    return ptr_map_landmark;
  }

  MapLandmark * Cartographer::addLandmarkToGlobalMap(std::unique_ptr<MapLandmark> & landmark_new)
  {
    // Add landmark to local map
    MapLandmark * ptr_map_landmark = map_global_.addLandmark(landmark_new);
    // Mark landmark as active
    ptr_map_landmark->setActive();

    // Add to INC system
    addLandmarkToIncSystem(ptr_map_landmark);

    return ptr_map_landmark;
  }

  size_t Cartographer::addLocalLandmarksToGlobalMap(const float & f_min_quality_landmark)
  {
    size_t n_landmarks_ok = 0;

    for (MapLandmarks::iterator it_map_landmark = map_local_.map_landmarks_.begin(); it_map_landmark != map_local_.map_landmarks_.end();)
    {
      std::unique_ptr<MapLandmark> & map_landmark = it_map_landmark->second;
      if (checkLandmarkQuality(map_landmark.get(), f_min_quality_landmark))
      {

        n_landmarks_ok++;

        MapLandmark * g_lm = addLandmarkToGlobalMap(map_landmark);

        it_map_landmark = map_local_.map_landmarks_.erase(it_map_landmark);

        if (time_data.b_enable_features_stats)
        {
          time_data.added_local_to_global_landmarks ++;
        }

      }
      else
      {
        ++it_map_landmark;
      }
    }

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Augment Map] Add " << n_landmarks_ok << " landmarks from local to global structure\n";

    return n_landmarks_ok;
  }


  void Cartographer::removeInactiveInLocalMapLandmarks(Frame * frame)
  {
    std::vector<size_t> vec_tmp_structure_outliers;

    #ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel for schedule(dynamic)
    #endif
    for(size_t local_landmark_i = 0; local_landmark_i < map_local_.getNumberOfLandmarks(); ++local_landmark_i)
    {
      MapLandmarks::iterator local_landmark_it = map_local_.map_landmarks_.begin();
      std::advance(local_landmark_it, local_landmark_i);
      MapLandmark * map_landmark = local_landmark_it->second.get();


      // Check if its not obsure (not seen for a while)
      if (map_landmark->getObservedInStep() + params_->map_max_inactive_f_local_landmark < step_id_)
      {
        #pragma omp critical
        {
          if (verbose_level > 1)
            std::cout<<"Map landmark id: "<<map_landmark->id_<<" inactive last: "<<map_landmark->getObservedInStep()<<" limit: "<<params_->map_max_inactive_f_local_landmark<<" C: "<<step_id_<<"\n";

          vec_tmp_structure_outliers.push_back(local_landmark_i);

          if (time_data.b_enable_features_stats)
          {
            time_data.removed_local_landmarks_inactive++;
          }

        }
        continue;
      }

    }

    // sort indexes of outliers
    std::sort(vec_tmp_structure_outliers.begin(),vec_tmp_structure_outliers.end());
    size_t n_local_landmarks_before = map_local_.getNumberOfLandmarks();

    // Remove any map landmarks that dont have enough measurements
    std::vector<MapLandmark *>  & vec_landmarks_frame = frame->getLandmarks();
    for (size_t o_i = 0; o_i < vec_tmp_structure_outliers.size(); ++o_i)
    {
      MapLandmarks::iterator outlier_it = map_local_.map_landmarks_.begin();
      // Reduce the number by the number of elements already deleted
      std::advance(outlier_it,vec_tmp_structure_outliers[o_i]-o_i);

      MapLandmark * map_landmark = outlier_it->second.get();
      LandmarkObservations & vec_obs = map_landmark->getObservations();

      for(LandmarkObservations::iterator mo_it = vec_obs.begin(); mo_it != vec_obs.end();++mo_it)
      {
        mo_it->second.frame_ptr->removeLandmark(mo_it->second.feat_id);
      }

      // Remove possible connection with current local frame
      std::vector<MapLandmark*>::iterator it_landmark_frame = std::find(vec_landmarks_frame.begin(),vec_landmarks_frame.end(),outlier_it->second.get());

      if (it_landmark_frame != vec_landmarks_frame.end())
      {
        IndexT feat_id = std::distance(vec_landmarks_frame.begin(), it_landmark_frame);
        frame->removeLandmark(feat_id);
      }

      // Delete 3D point (all other references have been deleted before)
      map_local_.map_landmarks_.erase(outlier_it);
    }

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Map verification] Local structure before/after outlier rejection: "<<n_local_landmarks_before<<"/"<<map_local_.getNumberOfLandmarks()<<"\n";

  }

  void Cartographer::removeOutliersInLocalMapLandmarks(Frame * frame)
  {
    std::vector<size_t> vec_tmp_structure_outliers;

    #ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel for schedule(dynamic)
    #endif
    for(size_t local_landmark_i = 0; local_landmark_i < map_local_.getNumberOfLandmarks(); ++local_landmark_i)
    {
      MapLandmarks::iterator local_landmark_it = map_local_.map_landmarks_.begin();
      std::advance(local_landmark_it, local_landmark_i);
      MapLandmark * map_landmark = local_landmark_it->second.get();


      // Check if its not obsure (not seen for a while)
      if (map_landmark->getObservedInStep() + params_->map_max_inactive_f_local_landmark < step_id_)
      {
        #pragma omp critical
        {
          if (verbose_level > 1)
            std::cout<<"Map landmark id: "<<map_landmark->id_<<" inactive last: "<<map_landmark->getObservedInStep()<<" limit: "<<params_->map_max_inactive_f_local_landmark<<" C: "<<step_id_<<"\n";
          vec_tmp_structure_outliers.push_back(local_landmark_i);

          if (time_data.b_enable_features_stats)
          {
            time_data.removed_local_landmarks_inactive++;
          }

        }
        continue;
      }

      // Check reprojection errors on each frame that sees it
      Vec2 pt_2D_frame;
      LandmarkObservations & vec_obs = map_landmark->getObservations();
      bool b_outlier_landmark = false;
      for(LandmarkObservations::iterator map_obs_it = vec_obs.begin(); map_obs_it != vec_obs.end();++map_obs_it)
      {
        MapObservation & map_obs = map_obs_it->second;
        Frame * frame_i = map_obs.frame_ptr;
        IndexT & feat_id_frame_i = map_obs.feat_id;
        // Project point to frame coordinate system
        if (!frame_i->getProjectedPoint(map_landmark,pt_2D_frame) || !frame_i->checkFeatureAssociation(pt_2D_frame,feat_id_frame_i,5.991))
        {
          #pragma omp critical
          {
            if (verbose_level > 1)
              std::cout<<"Map landmark id: "<<map_landmark->id_<<" observation error\n";
            vec_tmp_structure_outliers.push_back(local_landmark_i);

            if (time_data.b_enable_features_stats)
            {
              time_data.removed_local_landmarks_outliers++;
            }
          }
          b_outlier_landmark = true;
          break;
        }
      }

    }

    // sort indexes of outliers
    std::sort(vec_tmp_structure_outliers.begin(),vec_tmp_structure_outliers.end());
    size_t n_local_landmarks_before = map_local_.getNumberOfLandmarks();

    // Remove any map landmarks that dont have enough measurements
    std::vector<MapLandmark *>  & vec_landmarks_frame = frame->getLandmarks();
    for (size_t o_i = 0; o_i < vec_tmp_structure_outliers.size(); ++o_i)
    {
      MapLandmarks::iterator outlier_it = map_local_.map_landmarks_.begin();
      // Reduce the number by the number of elements already deleted
      std::advance(outlier_it,vec_tmp_structure_outliers[o_i]-o_i);

      MapLandmark * map_landmark = outlier_it->second.get();
      LandmarkObservations & vec_obs = map_landmark->getObservations();

      for(LandmarkObservations::iterator mo_it = vec_obs.begin(); mo_it != vec_obs.end();++mo_it)
      {
        mo_it->second.frame_ptr->removeLandmark(mo_it->second.feat_id);
      }


      // Remove possible connection with current local frame
      std::vector<MapLandmark*>::iterator it_landmark_frame = std::find(vec_landmarks_frame.begin(),vec_landmarks_frame.end(),outlier_it->second.get());

      if (it_landmark_frame != vec_landmarks_frame.end())
      {
        IndexT feat_id = std::distance(vec_landmarks_frame.begin(), it_landmark_frame);
        frame->removeLandmark(feat_id);
      }

      // Delete 3D point (all other references have been deleted before)
      map_local_.map_landmarks_.erase(outlier_it);
    }

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Map verification] Local structure before/after outlier rejection: "<<n_local_landmarks_before<<"/"<<map_local_.getNumberOfLandmarks()<<"\n";

  }

  // -------------------
  // -- Observations
  // -------------------
  void Cartographer::addObservationsToLandmarks
  (
    Frame * frame,
    const float & f_min_quality_landmark
  )
  {
    const IndexT & frame_id = frame->getFrameId();

    size_t n_observations_added = 0;
    // Loop through matches and add observations:
    //   - if valid frame and local point we increase the counter
    for (IndexT feat_id = 0; feat_id < frame->getNumberOfFeatures(); ++feat_id)
    {
      MapLandmark * & map_landmark = frame->getLandmark(feat_id);

      if (!map_landmark)
        continue;
      // Add observation to the landmark
      map_landmark->addObservation(frame,feat_id);
      n_observations_added++;
      // If landmark is already in global we add observation to the system
      // If landmark becomes valid with this observation we add it to global map
      if (b_global_map_intialized_)
      {
        if (map_landmark->isActive())
        {
          // Add observation to INC system
          MapObservation * map_observation = &(map_landmark->getObservation(frame_id));
          addObservationToIncSystem(map_landmark,map_observation);
        }
        else if(checkLandmarkQuality(map_landmark, f_min_quality_landmark))
        {
          // Add landmark from local to global system
          IndexT id_local_landmark = map_landmark->id_;

          addLandmarkToGlobalMap(map_local_.getLandmark(id_local_landmark));
          // Remove landmark from local map
          map_local_.removeLandmark(id_local_landmark);

          if (time_data.b_enable_features_stats)
          {
            time_data.added_local_to_global_landmarks++;
          }
          n_observations_added++;
        }
      }
      else
      {
        // Mark landmark as seen in this step (for inactivity check)
        map_landmark->setObservedInStep(step_id_);
      }
      updateBestLandmarkDescriptor(map_landmark);
    }

    if (verbose_level > 1)
      std::cout<<"Cartographer: [Augment Map] Add " << n_observations_added << " observations to landmarks from frame with id "<<frame_id<<"!\n";
  }


  // ------------------------------
  // -- Map quality
  // ------------------------------
  bool Cartographer::checkLandmarkQuality(MapLandmark * map_landmark, const float & f_thresh_quality)
  {
    // Observation degree
    if (map_landmark->getNumberOfObservations() < f_thresh_quality)
      return false;
    return true;
  }


  // -------------------
  // -- Optimization
  // -------------------
  void Cartographer::initGlobalOptimization()
  {
    switch (global_BA_type_)
    {
    case MAP_OPTIMIZATION_TYPE::CERES:
    {
      if (verbose_level > 1)
        std::cout<<"VSSLAM: [Cartographer] Ceres Global Optimization Mode\n";

      VSSLAM_BA_Ceres::BA_options_Ceres options(map_global_.map_frame_type_, map_global_.map_landmark_type_, true, true);
      options.b_export_graph_file = params_->b_export_graph_file;
      options.s_graph_file = stlplus::create_filespec(params_->s_output_folder, params_->s_graph_file_path);

      BA_obj_ = std::unique_ptr<VSSLAM_BA>(new VSSLAM_BA_Ceres(options));
      BA_obj_->setVerboseLevel(verbose_level);

      break;
    }
    case MAP_OPTIMIZATION_TYPE::SLAMPP:
    {
      if (verbose_level > 1)
        std::cout<<"VSSLAM: [Cartographer] Slam++ Global Optimization Mode\n";

      VSSLAM_BA_SlamPP::BA_options_SlamPP options(map_global_.map_frame_type_, map_global_.map_landmark_type_, true, true);
      options.b_export_graph_file = params_->b_export_graph_file;
      options.s_graph_file = stlplus::create_filespec(params_->s_output_folder, params_->s_graph_file_path);

      BA_obj_ = std::unique_ptr<VSSLAM_BA>(new VSSLAM_BA_SlamPP(options));
      BA_obj_->setVerboseLevel(verbose_level);
      break;
    }
    }
  }

  void Cartographer::addObservationToIncSystem(MapLandmark * map_landmark, MapObservation * map_observation)
  {
    BA_obj_->addObservationToGlobalSystem(map_landmark,map_observation);
  };
  void Cartographer::addLandmarkToIncSystem(MapLandmark * map_landmark)
  {
    BA_obj_->addLandmarkToGlobalSysyem(map_landmark);
  };
  void Cartographer::addFrameToIncSystem(Frame * frame, bool b_frame_fixed)
  {
    BA_obj_->addFrameToGlobalSystem(frame,b_frame_fixed);
  };


  bool Cartographer::optimizeLocalMap
  (
    Frame * frame_i,
    NewMapLandmarks & vec_new_landmarks,
    bool b_use_loss_function
  )
  {
    switch(local_BA_type_)
    {
    case MAP_OPTIMIZATION_TYPE::CERES:
    {
      VSSLAM_BA_Ceres::BA_options_Ceres options;
      options.b_use_sim3_local = true;
      return VSSLAM_BA_Ceres::OptimizeLocalSystem(frame_i,vec_new_landmarks,b_use_loss_function,options, verbose_level);
      break;
    }
    case MAP_OPTIMIZATION_TYPE::SLAMPP:
    {
      std::cout<<"VSSLAM: [Cartographer] SlamPP is not implemented for local BA\n";
      return false;
    }
    }
    return false;
  }

  bool Cartographer::optimizePose
  (
    Frame * frame,
    Hash_Map<MapLandmark *,IndexT> & matches_map_cur_idx,
    bool b_use_loss_function
  )
  {
    switch(local_BA_type_)
    {
    case MAP_OPTIMIZATION_TYPE::CERES:
    {
      VSSLAM_BA_Ceres::BA_options_Ceres options;
      return VSSLAM_BA_Ceres::OptimizePose(frame,matches_map_cur_idx,b_use_loss_function,options, verbose_level);
      break;
    }
    case MAP_OPTIMIZATION_TYPE::SLAMPP:
    {
      std::cout<<"VSSLAM: [Cartographer] SlamPP is not implemented for local BA\n";
      return false;
    }
    }
    return false;
  }

  bool Cartographer::exportSceneToPly(const std::string & filename, bool b_export_local_scene)
  {
    // Create the stream and check its status
    std::ofstream stream(filename.c_str());
    if (!stream.is_open())
      return false;

    bool bOk = false;
    // Count how many global frames
    size_t n_frames = map_global_.getNumberOfFrames();
    size_t n_landmarks = map_global_.getNumberOfLandmarks();
    if (b_export_local_scene)
    {
      n_frames += map_local_.getNumberOfFrames();
      n_landmarks += map_local_.getNumberOfLandmarks();
    }


    stream << std::fixed << std::setprecision (std::numeric_limits<double>::digits10 + 1);

    stream << "ply"
      << '\n' << "format ascii 1.0"
      << '\n' << "element vertex "
        // Vertex count: (#landmark + #GCP + #view_with_valid_pose)
        << ( n_frames + n_landmarks )
      << '\n' << "property double x"
      << '\n' << "property double y"
      << '\n' << "property double z"
      << '\n' << "property double uncertainty"
      //<< '\n' << "property uchar red"
      //<< '\n' << "property uchar green"
      //<< '\n' << "property uchar blue"
      << '\n' << "end_header" << std::endl;

    for (auto & frame_it : map_global_.map_frame_)
    {
      // Export pose as Green points
      Vec3 center = frame_it.second->getCameraCenter();
        stream
          << center(0) << ' '
          << center(1) << ' '
          << center(2) << ' '
          << "0.0"
          //<< "0 255 0"
          << "\n";
    }

    if (b_export_local_scene)
    {
      for (auto & frame_it : map_local_.map_frame_)
      {
        // Export pose as  points
        Vec3 center = frame_it.second->getCameraCenter();
          stream
            << center(0) << ' '
            << center(1) << ' '
            << center(2) << ' '
            << "0.0"
            //<< "255 255 0";
            << "\n";
      }
    }

    // Export structure points as White points
    for ( auto & map_landmark_it : map_global_.map_landmarks_ )
    {
      MapLandmark * map_landmark = map_landmark_it.second.get();
      Vec3 pt_3D_w;
      PoseEstimator::getRelativePointPosition(map_landmark->X_,map_landmark->getReferenceFrame(),pt_3D_w,nullptr);
      stream
        << pt_3D_w(0) << ' '
        << pt_3D_w(1) << ' '
        << pt_3D_w(2) << ' '
        << map_landmark->cov_X_.trace()
        //<< "255 255 255"
        << "\n";
    }

    if (b_export_local_scene)
    {
      // Export structure points as White points
      for ( auto & map_landmark_it : map_local_.map_landmarks_ )
      {
        MapLandmark * map_landmark = map_landmark_it.second.get();
        Vec3 pt_3D_w;
        PoseEstimator::getRelativePointPosition(map_landmark->X_,map_landmark->getReferenceFrame(),pt_3D_w,nullptr);
        stream
          << pt_3D_w(0) << ' '
          << pt_3D_w(1) << ' '
          << pt_3D_w(2) << ' '
          << "0.0"
          //<< "255 0 0"
          << "\n";
      }
    }

    stream.flush();
    bOk = stream.good();
    stream.close();

    return bOk;
  }

  bool Cartographer::exportStateSE3(std::string filename)
  {
    if (verbose_level>1)
      std::cout<<"Cartographer: Export State SE3\n";

    BA_obj_->exportStateSE3(filename);
    return true;
  }

  bool Cartographer::exportDiagonalMarginals(std::string filename)
  {
    if (verbose_level>1)
      std::cout<<"Cartographer: Export Diagonal Marginals\n";

    BA_obj_->exportDiagonalMarginals(filename);
    return true;
  }

}
}
