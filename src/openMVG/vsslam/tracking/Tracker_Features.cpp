// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "openMVG/types.hpp"
#include <openMVG/vsslam/tracking/Tracker_Features.hpp>

namespace openMVG {
namespace vsslam {

  Tracker_Features::Tracker_Features
  (
    std::shared_ptr<VSSLAM_Parameters> & params
  )
  {
    params_ = params->share_ptr();
  }

  bool Tracker_Features::isReady()
  {
    if (!feature_extractor_)
      return false;
    if (!feature_matcher_)
      return false;
    if (!cartographer_)
      return false;

    return true;
  }
  bool Tracker_Features::track
  (
    const image::Image<unsigned char> & ima,
    std::shared_ptr<Frame> & frame_current,
    const image::Image<unsigned char> * mask
  )
  {
    bool b_track_status = false;

    // Restart stats data
    time_data.restartData();

    // Copy shared pointer of current frame
    frame_track_prev.swap(frame_track_current);
    frame_track_current = frame_current->share_ptr();

    // -------------------------
    // -- Detect features
    // -------------------------
    // Time statistics
    time_data.startTimer(time_data.d_feat_detection);

    const size_t n_feats_detected = feature_extractor_->detect(ima,frame_track_current.get(),mask);
    feature_extractor_->describe(ima,frame_track_current.get());
    frame_track_current->updateFeaturesData();

    // Time statistics
    time_data.stopTimer(time_data.d_feat_detection);

    // Check if sufficient features were detected
    if ( ((tracking_status_ == TRACKING_STATUS::NOT_INIT || tracking_status_ == TRACKING_STATUS::INIT) && n_feats_detected < params_->init_track_min_matches)
          || (tracking_status_ == TRACKING_STATUS::OK && n_feats_detected < params_->track_min_matches) )
    {
      // No features detected on the frame
      if (verbose_level > 1)
        std::cout<<"Tracker: [Detection] Error - insufficient features detected -> next frame!\n";

      // Reset current frame (keep mPrevFrame from before)
      frame_track_current.reset();
      return true;
    }

    // -------------------------
    // -- Tracking
    // -------------------------
    // Time statistics
    time_data.startTimer(time_data.d_feat_tracking);

    if (tracking_status_ == TRACKING_STATUS::NOT_INIT || tracking_status_ == TRACKING_STATUS::INIT)
    {
      b_track_status = trackingInitialization(ima);
      // Time statistics
      time_data.stopTimer(time_data.d_feat_tracking);
      endOfFrameProcedure();
      return b_track_status;
    }

    // Flag for marking successful tracking
    bool b_track_OK = false;
    bool b_use_ref_frame= false;

    if (tracking_status_ == TRACKING_STATUS::OK)
    {
      // ----------------
      // -- Motion Model tracking
      // ----------------
      if (motion_model_.isValid())
      {
        // Time statistics
        time_data.startTimer(time_data.d_feat_tracking_mm);

        // Match current frame with lastFrame
        b_track_OK = trackWithMotionModel(track_putative_matches_frame_current);

        if (verbose_level > 1)
          std::cout<<"Tracker: [Track] Motion Model: "<<b_track_OK<<"\n";

        // Time statistics
        time_data.stopTimer(time_data.d_feat_tracking_mm);
      }

      // Mark

      // ----------------
      // -- Reference frame tracking
      // --  - if motion model is not good or available (we are lost)
      // --  - we try to match to last reference frame
      // ----------------

      if (!b_track_OK)
      {
        // Time statistics
        time_data.startTimer(time_data.d_feat_tracking_rf);

        b_use_ref_frame = true;
        b_track_OK = trackWithReferenceFrame(frame_track_last_reference.get(),track_putative_matches_frame_current);

        if (verbose_level > 1)
          std::cout<<"Tracker: [Track] Reference frame: "<<b_track_OK<<"\n";

        // Time statistics
        time_data.stopTimer(time_data.d_feat_tracking_rf);
      }

      if (!b_track_OK)
      {
        tracking_status_ = TRACKING_STATUS::LOST;

        if (verbose_level > 1)
          std::cout<<"Tracker: [Track] Set frame to LOST!\n";
      }
    }

    if (tracking_status_ == TRACKING_STATUS::LOST)
    {
      // ----------------
      // -- RELOCALIZATION
      // ----------------

      // Time statistics
      time_data.startTimer(time_data.d_feat_tracking_rm);

      track_putative_matches_frame_current.clear();
      // Try to track with with All-All with local map of landmarks from last reference frame
      b_track_OK = trackWithReferenceFrameLocalMap(frame_track_last_reference.get(),track_putative_matches_frame_current);

      if (b_track_OK)
      {
        tracking_status_ = TRACKING_STATUS::OK;
        frame_last_relocalization_id = frame_track_current->getFrameId();
      }

      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Relocalization: "<<b_track_OK<<"\n";

      // Time statistics
      time_data.stopTimer(time_data.d_feat_tracking_rm);
    }


    // if we are lost we are done :)
    if (!b_track_OK)
    {
      // Time statistics
      time_data.stopTimer(time_data.d_feat_tracking);
      motion_model_.setInvalid();
      endOfFrameProcedure();
      return false;
    }
    // Associate matches with frame
    associateLandmarksWithFrame(frame_track_current.get(),track_putative_matches_frame_current, (b_use_ref_frame ? 3 : 2));

    // ----------------
    // -- LOCAL MAP TRACKING
    // ----------------

    // Time statistics
    time_data.startTimer(time_data.d_feat_tracking_lm);

    //  - with estimated pose we try to match more landmarks from local map
    b_track_OK = trackLocalMap();

    // Time statistics
    time_data.stopTimer(time_data.d_feat_tracking_lm);

    if (!b_track_OK)
    {
      time_data.stopTimer(time_data.d_feat_tracking);
      tracking_status_ = TRACKING_STATUS::LOST;

      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Set frame to LOST!\n";

      motion_model_.setInvalid();
      endOfFrameProcedure();
      return false;
    }

    // Finished the tracking part
    time_data.stopTimer(time_data.d_feat_tracking);

    // ----------------
    // -- NEW KEYFRAME?
    // ----------------
    // Time statistics
    time_data.startTimer(time_data.d_feat_mapping);
    
    NewMapLandmarks vec_new_map_landmarks;
    
    // Check if new keyframe is needed
    size_t new_keyframe_reason = needNewKeyframe(frame_track_current.get());
    if (new_keyframe_reason > 0)
    {
      if (verbose_level > 1)
      {
        std::cout<<"Tracker: [Track] New Keyframe? YES: ";
        printKeyFrameReason(new_keyframe_reason);
        std::cout << "\n";
      }
      time_data.b_keyframe = true;
      
      // Time statistics
      time_data.startTimer(time_data.d_feat_mapping_search_new_pts);
      
      // Find new points for triangulation
      findNewCandidateLandmarks(frame_track_current.get(), vec_new_map_landmarks);

      // Time statistics
      time_data.stopTimer(time_data.d_feat_mapping_search_new_pts);

      // Time statistics
      time_data.startTimer(time_data.d_feat_mapping_pose_opt_local);
      bool b_use_robust_function = true;
      if (!cartographer_->optimizeLocalMap(frame_track_current.get(), vec_new_map_landmarks,b_use_robust_function))
      {
        // Time statistics
        time_data.stopTimer(time_data.d_feat_mapping_pose_opt_local);
        time_data.stopTimer(time_data.d_feat_mapping);

        if (verbose_level > 1)
          std::cout<<"Tracker: [Track] Optimize local FAILED\n";

        return false;
      }

      // Remove outliers after local BA
      removeOutliersInFrame(frame_track_current.get());
      removeOutliersInCandidateLandmarks(frame_track_current.get(),vec_new_map_landmarks);
      cartographer_->removeOutliersInLocalMapLandmarks(frame_track_current.get());

      // Time statistics
      time_data.stopTimer(time_data.d_feat_mapping_pose_opt_local);

      // Save for display
      if (display_data.b_enable_display)
      {
        display_data.addDisplayStep("Initialization: New triangulated points after BA",frame_track_current.get(), vec_new_map_landmarks);
      }


      if (params_->b_export_intermediate_scene_ply)
      {
        // Export step to Ply
        std::ostringstream os_before;
        os_before << "Scene_"<<cartographer_->step_id_<<"_frame_"<<frame_track_current->getFrameId()<<"_beforeBA";
        cartographer_->exportSceneToPly(stlplus::create_filespec(params_->s_output_folder, os_before.str(), ".ply"),true);
      }

      // Time statistics
      time_data.startTimer(time_data.d_feat_mapping_global_step);

      // Add data to map
      bool b_map_insertion = cartographer_->addStep(frame_track_current, &vec_new_map_landmarks, &time_data);

      // Time statistics
      time_data.stopTimer(time_data.d_feat_mapping_global_step);

      if (!b_map_insertion)
      {
        // Time statistics
        time_data.stopTimer(time_data.d_feat_mapping);
        endOfFrameProcedure();
        tracking_status_ = TRACKING_STATUS::NOT_INIT;
        return false;
        
      }
      if (params_->b_export_intermediate_scene_ply)
      {
        // Export step to Ply
        std::ostringstream os_after;
        os_after << "Scene_"<<cartographer_->step_id_<<"_frame_"<<frame_track_current->getFrameId()<<"_postBA";
        cartographer_->exportSceneToPly(stlplus::create_filespec(params_->s_output_folder, os_after.str(), ".ply"), false);


        std::ostringstream os_state;
        os_state << "solution_SE3_iter_" << std::setfill('0') << std::setw(5) << cartographer_->step_id_ << ".txt";

        std::cout << "EXPORT A: " << stlplus::create_filespec(params_->s_output_folder, os_state.str()) << "\n";

        cartographer_->exportStateSE3(stlplus::create_filespec(params_->s_output_folder, os_state.str()));

        std::cout << "EXPORT B: " << stlplus::create_filespec(params_->s_output_folder, os_state.str()) << "\n";

        std::ostringstream os_marginals;
        os_marginals << "marginals_iter_" << std::setfill('0') << std::setw(5) << cartographer_->step_id_ << ".txt";
        cartographer_->exportDiagonalMarginals(stlplus::create_filespec(params_->s_output_folder, os_marginals.str()));
        std::cout << "EXPORT M: " << stlplus::create_filespec(params_->s_output_folder, os_marginals.str()) << "\n";
      }

      // -------------------
      // -- Update last reference camera
      // -------------------
      frame_track_last_reference = frame_track_current->share_ptr();

    }
    else
    {
      // Time statistics
      time_data.startTimer(time_data.d_feat_mapping_outliers_after_global);
      // Remove outliers
      cartographer_->removeInactiveInLocalMapLandmarks(frame_track_current.get());
      // Time statistics
      time_data.stopTimer(time_data.d_feat_mapping_outliers_after_global);
    }

    // Time statistics
    time_data.stopTimer(time_data.d_feat_mapping);

    // -------------------
    // -- Update motion model
    // -------------------
    motion_model_.update(frame_track_prev.get(),frame_track_current.get());


    // Set current frame as previous
    endOfFrameProcedure();
    // Return if tracking is ok
    return true;
  }

  size_t Tracker_Features::needNewKeyframe
  (
    Frame * frame
  ) const
  {
    // Create new keyframe if:
    //  - more than X frames have passed from last keyframe
    //  - tracking is weak (less than 85% of matches from last reference frame are in the new one)
    //  - its necessary for creating new points

    // We create new keyframe if:
    // - more than X frames have passed from last mapping
    // - OR
    // - tracking is weak (less than 90% of matches than in last reference frame)
    // - OR
    // - its necessary for creating new points (more than 20% of points are new)

    const size_t n_matches_reference_global = frame_track_last_reference->getNumberOfMapPoints(cartographer_->isMapInitialized());
    const size_t n_matches_reference_all = frame_track_last_reference->getNumberOfMapPoints(false);
    const size_t n_matches_current_global = frame->getNumberOfMapPoints(cartographer_->isMapInitialized());
    const size_t n_matches_current_all =  frame->getNumberOfMapPoints(false);

    if (verbose_level > 1)
    {
      std::cout<<"Tracker: [Track] New Keyframe?" <<"\n";
      std::cout<<"\t# current all / global: "<<n_matches_current_all << " / " << n_matches_current_global << "\n"
              << "\t# reference all / global: "<<n_matches_reference_all << " / " << n_matches_reference_global << "\n";
    }

    if (frame_track_last_reference->getFrameId() + params_->n_max_frames_not_tracked < frame_track_current->getFrameId())
    {
      
      if (time_data.b_enable_features_stats)
      {
        time_data.keyframe_reason = 1;
      }
      return 1;
    }

    if (n_matches_current_global < params_->n_min_global_landmarks_tracked)
    {
      if (time_data.b_enable_features_stats)
      {
        time_data.keyframe_reason = 2;
      }
      return 2;
    }

    if (n_matches_current_global < params_->n_min_global_landmarks_to_check_prev_stats && n_matches_current_global < float(n_matches_reference_global) * params_->f_min_ratio_of_landmarks_tracked_reference_frame)
    {
      if (time_data.b_enable_features_stats)
      {
        time_data.keyframe_reason = 3;
      }
      return 3;
    }

    if (n_matches_current_all > float(n_matches_reference_global) * params_->f_min_ratio_of_landmarks_tracked_reference_frame && frame_track_last_reference->getFrameId() +5 < frame_track_current->getFrameId())
    {
      if (time_data.b_enable_features_stats)
      {
        time_data.keyframe_reason = 4;
      }
      return 4;
    }

   /* if (n_matches_current_all < float(n_matches_reference) * 0.70 || n_matches_current_all > float(n_matches_reference) * 1.40)
    {
      std::cout<<"New frame: tracking < 0.50*reference\n";
      return true;
    }
*/
    return 0;
  }

  void Tracker_Features::printKeyFrameReason( size_t & keyframe_reason)
  {
    switch(keyframe_reason)
    {
      case 0:
        std::cout << "Not keyframe";
        break;
      case 1:
        std::cout << "More than " << params_->n_max_frames_not_tracked << " frames from last keyframe";
        break;
      case 2:
        std::cout << "Less than " << params_->n_min_global_landmarks_tracked << " global landmarks tracked";
        break;
      case 3:
        std::cout << "Less than " << params_->n_min_global_landmarks_to_check_prev_stats << " global landmarks tracked and less than certain " << params_->f_min_ratio_of_landmarks_tracked_reference_frame << " ratio of points from ref frame tracked";
        break;
      case 4:
        std::cout << "More than " << params_->f_max_ratio_of_landmarks_tracked_prev_frame << " ratio of tracked points compared to ref frame";
        break;
    }
  }

  void Tracker_Features::endOfFrameProcedure()
  {
    track_putative_matches_frame_current.clear();

    if (verbose_level > 1)
      std::cout<<"Tracker: [End tracker]\n";

  }

  void Tracker_Features::associateLandmarksWithFrame(Frame * frame, Hash_Map<MapLandmark *,IndexT> & matches_landmarks_frame, size_t associate_type)
  {

    #ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel for schedule(dynamic)
    #endif
    for (size_t match_i = 0; match_i < matches_landmarks_frame.size(); ++match_i)
    {
      Hash_Map<MapLandmark *,IndexT>::iterator it_match = matches_landmarks_frame.begin();
      std::advance(it_match,match_i);

      frame->setLandmark(it_match->second, it_match->first);
      it_match->first->increaseNumberOfObservations();

      // Set association
      it_match->first->association_type_ = associate_type;
    }
  }

  void Tracker_Features::findNewCandidateLandmarks(Frame * frame, NewMapLandmarks & vec_new_map_landmarks)
  {
    // Get local map of frame
    std::vector<Frame *> local_map_frames;
    frame->getFrameVisibilityConnections(local_map_frames,params_->triangulation_local_map_n_frames);

    // Get info of frame
    const Vec3 frame_camera_center = frame->getCameraCenter();

    // -------------------
    // -- Loop through each neighbor frame and check if we can match any of the (unmatched) landmarks in frame to
    // -- (unmatched) features in the neighbor frames
    // -------------------
    std::vector<Hash_Map<IndexT,IndexT> > vec_matches_frame_local_idx(local_map_frames.size());

    #ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel for schedule(dynamic)
    #endif
    for (size_t f_i = 0; f_i < local_map_frames.size(); ++f_i)
    {
      Frame * local_frame_i = local_map_frames[f_i];

      // Compute statistics of the scene
      local_frame_i->computeSceneStatistics();
      const float f_scene_median_i = local_frame_i->getSceneMedian();

      // Compute ratio baseline/scene depth between the pair of frames
      const Vec3 frame_i_camera_center = local_frame_i->getCameraCenter();
      const float baseline = (frame_camera_center - frame_i_camera_center).norm();
      const float ratio = baseline / f_scene_median_i;

      if (verbose_level > 1)
      {
        #pragma omp critical
        {
          std::cout<<"Tracker: [NewCandidateLandmarks] Frame (current/local): " << frame->getFrameId() << " / " << local_frame_i->getFrameId() << " Ratio baseline/scene depth: "<<ratio<< " (baseline: "<<baseline<<")\n";
        }
      }

      if (ratio > 0)
      {
        // Estimate Fundamental matrix between two frames
        Mat3 F_local_frame; // p_local' * F * p_frame = 0
        PoseEstimator::computeFundamentalMatrix(frame,local_frame_i,F_local_frame);

        // Perform matching with epipolar constraints
        feature_matcher_->matching_Epipolar_2D_2D
        (
          feature_extractor_,
          frame,
          local_frame_i,
          F_local_frame,
          vec_matches_frame_local_idx[f_i],
          params_->triang_match_max_epipolar_distance,
          params_->triang_match_desc_ratio,
          params_->triang_match_max_scale_ratio,
          feature_extractor_->f_max_desc_dist_high_
        );


        if (verbose_level > 1)
        {
          #pragma omp critical
          {
            std::cout<<"Tracker: [NewCandidateLandmarks] Frame (current/local): " << frame->getFrameId() << " / " << local_frame_i->getFrameId() << "  Matched by epipolar: "<<vec_matches_frame_local_idx[f_i].size()<<"\n";
          }
        }

        /*if (display_data.b_enable_display)
        {
          display_data.addDisplayStep("Tracking epipolar: ",frame_track_current.get(),local_frame_i, vec_matches_frame_local_idx[f_i]);
        }*/
      }
    }

    // Go backwards and look if new landmarks can be triangulated
    // Landmark is good if:
    //   - the parallax is big enough
    //   - there are no outliers in the matching

    Mat3 Kinv_frame = frame->getKinv();
    Mat34 P_frame = frame->getCameraProjectionMatrix(nullptr);

    Hash_Map<IndexT, std::pair<bool,std::unique_ptr<MapLandmark> > > map_new_landmarks;

    for (size_t f_i = 0; f_i < local_map_frames.size(); ++f_i)
    {
      if (vec_matches_frame_local_idx[f_i].empty())
        continue;

      Frame * frame_local_i = local_map_frames[f_i];
      Mat34 P_local_i = frame_local_i->getCameraProjectionMatrix(nullptr);
      Mat3 Kinv_local_i = frame_local_i->getKinv();

      #ifdef OPENMVG_USE_OPENMP
      #pragma omp parallel for schedule(dynamic)
      #endif
      for (size_t match_i = 0; match_i < vec_matches_frame_local_idx[f_i].size(); ++match_i)
      {
        Hash_Map<IndexT,IndexT>::iterator match_it = vec_matches_frame_local_idx[f_i].begin();
        std::advance(match_it,match_i);

        IndexT feat_id_frame = match_it->first;
        IndexT feat_id_local_i = match_it->second;

        // If new landmark doesnt exist yet check if the conditions are met
        Hash_Map<IndexT, std::pair<bool,std::unique_ptr<MapLandmark> > >::iterator new_landmark_it = map_new_landmarks.find(feat_id_frame);

        if (new_landmark_it == map_new_landmarks.end())
        {
          // Get homogeneous coordiantes
          Vec3 pt_3D_frame = frame->getFeaturePositionHomogeneous(feat_id_frame);
          Vec3 pt_3D_local_i = frame_local_i->getFeaturePositionHomogeneous(feat_id_local_i);

          // Point is standing still - infinity
          if ((pt_3D_frame-pt_3D_local_i).squaredNorm() < 10)
            continue;

          // Get rays to the points
          Vec3 ray_frame = frame->getRayToPoint(Kinv_frame * pt_3D_frame,frame);
          Vec3 ray_local_i = frame_local_i->getRayToPoint(Kinv_local_i * pt_3D_local_i,frame_local_i);

          // Compute parallax between vectors
          const double cosParallax = PoseEstimator::computeCosParallaxBetweenRays(ray_frame,ray_local_i);

          if (cosParallax > params_->init_track_min_cos_parallax_pt)
          {
            continue;
          }

          // Create new landmark
          std::unique_ptr<MapLandmark> map_landmark = std::unique_ptr<MapLandmark>(new MapLandmark());
          TriangulateDLT(P_frame, pt_3D_frame, P_local_i, pt_3D_local_i, &(map_landmark->X_));

          // Add observations
          map_landmark->addObservation(frame,feat_id_frame);
          map_landmark->addObservation(frame_local_i,feat_id_local_i);
          map_landmark->setNumberOfObservations(2);
          cartographer_->updateBestLandmarkDescriptor(map_landmark.get());

          // Mark as point used in the motion initialization
          map_landmark->association_type_ = 6;

          #pragma omp critical
          {
            map_new_landmarks[feat_id_frame] = std::make_pair(true,std::move(map_landmark));
          }

        }
        else if (new_landmark_it->second.first)
        {
          // Check if the matched observation is consistent with previously triangulated point
          // If yes we add the measurement
          // If not we delete the landmark
          MapLandmark * map_landmark = new_landmark_it->second.second.get();

          Vec2 pt_2D_frame;
          if (!frame_local_i->getProjectedPoint(map_landmark,pt_2D_frame) || !frame_local_i->checkFeatureAssociation(pt_2D_frame,feat_id_local_i,5.991))
          {
            // Outlier
            new_landmark_it->second.first = false;
          }
          else
          {
            map_landmark->addObservation(frame_local_i,feat_id_local_i);
            map_landmark->increaseNumberOfObservations();
          }
        }
      }
    }

    if (verbose_level > 1)
      std::cout<<"Tracker: [NewCandidateLandmarks] All possible new landmarks: "<<map_new_landmarks.size()<<"\n";

    // Add landmarks that are valid to the vector of new landmarks
    for (auto & new_landmark : map_new_landmarks)
    {
      if (!new_landmark.second.first)
        continue;

      vec_new_map_landmarks.push_back(std::move(new_landmark.second.second));
    }

    if (verbose_level > 1)
      std::cout<<"Tracker: [NewCandidateLandmarks] Valid possible new landmarks: "<<vec_new_map_landmarks.size()<<"\n";
  }


  bool Tracker_Features::performPoseOptimization(Frame * frame, Hash_Map<MapLandmark *,IndexT> & map_putative_matches_map_cur_idx, bool b_use_robust_function)
  {
    // -------------------
    // -- Do tiny BA to refine the pose based on the matches
    // -------------------
    if (!cartographer_->optimizePose(frame, map_putative_matches_map_cur_idx,b_use_robust_function))
    {
      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Optimize pose FAILED\n";
      return false;
    }

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Optimize pose OK\n";

    // -------------------
    // -- Remove outliers among matches by chi2 test
    // -------------------

    removeOutliersInFrame(frame);
    removeOutliersInMatches(frame,map_putative_matches_map_cur_idx);

    return true;
  }

  // --------------------------
  //   Tracking: Motion Model
  // --------------------------
  bool Tracker_Features::trackWithMotionModel(Hash_Map<MapLandmark *,IndexT> & map_putative_matches_landmark_frame_idx)
  {
    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Use Motion Model\n";
    
    size_t n_matches = 0;

    // -------------------
    // -- Predict location of current frame (using MotionModel)
    // -------------------
    Mat4 T_predict = motion_model_.predict(frame_track_prev.get(),frame_track_current.get());

    // Set the predicted pose as pose of the current frame
    frame_track_current->setPose_T(T_predict, nullptr);

    // -------------------
    // -- Match by projecting tracked points from last frames to current frame
    // -------------------
    feature_matcher_->matching_Projection_3D_2D
    (
      feature_extractor_,
      frame_track_prev.get(),
      frame_track_current.get(),
      map_putative_matches_landmark_frame_idx,
      params_->track_match_mm_win_size,
      params_->track_match_mm_desc_ratio,
      params_->track_match_mm_max_scale_ratio,
      feature_extractor_->f_max_desc_dist_high_
    );
    n_matches = map_putative_matches_landmark_frame_idx.size();

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Match by projection: Frame id (prev/cur): "<<frame_track_prev->getFrameId()<<" -> "<<frame_track_current->getFrameId()<<" Total # matches: "<<map_putative_matches_landmark_frame_idx.size()<<"\n";

    if (display_data.b_enable_display)
    {
      display_data.addDisplayStep("Tracking MM: Narrow search",frame_track_prev.get(),frame_track_current.get(), map_putative_matches_landmark_frame_idx, params_->track_match_mm_win_size);
    }
    if (time_data.b_enable_features_stats)
    {
      time_data.i_matches_mm = n_matches;
    }

    // If not enough matches try matching with wider area
    if (n_matches < params_->track_min_matches)
    {
      feature_matcher_->matching_Projection_3D_2D
      (
        feature_extractor_,
        frame_track_prev.get(),
        frame_track_current.get(),
        map_putative_matches_landmark_frame_idx,
        params_->track_match_mm_win_size*4,
        params_->track_match_mm_desc_ratio,
        params_->track_match_mm_max_scale_ratio,
        feature_extractor_->f_max_desc_dist_high_
      );
      n_matches = map_putative_matches_landmark_frame_idx.size();

      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Match by projection wide: Frame id (prev/cur): "<<frame_track_prev->getFrameId()<<" -> "<<frame_track_current->getFrameId()<<" Total # matches: "<<map_putative_matches_landmark_frame_idx.size()<<"\n";

      if (display_data.b_enable_display)
      {
        display_data.addDisplayStep("Tracking MM: Wide search",frame_track_prev.get(),frame_track_current.get(), map_putative_matches_landmark_frame_idx, params_->track_match_mm_win_size*4);
      }

      if (time_data.b_enable_features_stats)
      {
        time_data.i_matches_mm = n_matches;
      }

      if (n_matches < params_->track_min_matches)
      {
        if (verbose_level > 1)
          std::cout<<"Tracker: [Track] Match by Motion Model Failed. # Matches: " << n_matches << "\n";
        return false;
      }
    }

    // Save old version to easily detect which were deleted - only for debug
    Hash_Map<MapLandmark *,IndexT> map_putative_matches_map_cur_idx_old;
    size_t n_matches_old = n_matches;
    if (display_data.b_enable_display)
    {
      map_putative_matches_map_cur_idx_old = map_putative_matches_landmark_frame_idx;
    }
    // -------------------
    // -- Do tiny BA to refine the pose based on the matches
    // -------------------

    // Time statistics
    time_data.startTimer(time_data.d_feat_tracking_pose_opt_mm);

    bool b_use_robust_function = true;
    if (!performPoseOptimization(frame_track_current.get(), map_putative_matches_landmark_frame_idx,b_use_robust_function))
    {
      // Time statistics
      time_data.stopTimer(time_data.d_feat_tracking_pose_opt_mm);
      return false;
    }
    n_matches = map_putative_matches_landmark_frame_idx.size();
    
    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] MM outlier removal (before/after): "<<n_matches_old<<"/"<<n_matches<<"\n";


    // Time statistics
    time_data.stopTimer(time_data.d_feat_tracking_pose_opt_mm);

    if (time_data.b_enable_features_stats)
    {
      time_data.i_matches_mm_outliers = n_matches - n_matches_old;
    }

    // Display matches after outlier removal
    if (display_data.b_enable_display)
    {
      display_data.addDisplayStep("Tracking MM: After BA and Outlier remvoal",frame_track_current.get(), map_putative_matches_map_cur_idx_old, map_putative_matches_landmark_frame_idx, 4);
    }

    if (n_matches < params_->track_min_matches)
    {
      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Match by Motion Model Failed. # Matches: " << n_matches << "\n";
      return false;
    }
    return true;
  }

  // --------------------------
  //   Tracking: Reference Frame
  // --------------------------
  bool Tracker_Features::trackWithReferenceFrame(Frame * frame_reference, Hash_Map<MapLandmark *,IndexT> & map_putative_matches_landmark_frame_idx)
  {
    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Use reference frame: "<<frame_reference->getFrameId()<<"\n";
    
    size_t n_matches = 0;

    // -------------------
    // -- Set location of current frame (as the one of last frame)
    // -------------------
    frame_track_current->setPose_T(frame_reference->getTransformationMatrix(), frame_reference->getReferenceFrame());


    // -------------------
    // -- Match by projecting tracked points from reference frame to current frame
    // -------------------
    feature_matcher_->matching_Projection_3D_2D
    (
      feature_extractor_,
      frame_reference,
      frame_track_current.get(),
      map_putative_matches_landmark_frame_idx,
      params_->track_match_rf_win_size,
      params_->track_match_rf_desc_ratio,
      params_->track_match_rf_max_scale_ratio,
      feature_extractor_->f_max_desc_dist_high_
    );
    n_matches = map_putative_matches_landmark_frame_idx.size();

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Match by reference frame: Frame id (prev/cur): "<<frame_reference->getFrameId()<<" -> "<<frame_track_current->getFrameId()<<" Total # matches: "<<map_putative_matches_landmark_frame_idx.size()<<"\n";

    if (display_data.b_enable_display)
    {
      display_data.addDisplayStep("Tracking reference frame: ",frame_reference,frame_track_current.get(), map_putative_matches_landmark_frame_idx, params_->track_match_rf_win_size);
    }

    if (time_data.b_enable_features_stats)
    {
      time_data.i_matches_rf = n_matches;
    }

    if (n_matches < params_->track_min_matches)
    {
      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Match by reference frame Failed. # Matches: " << n_matches << "\n";
      return false;
    }


    // Save old version to easily detect which were deleted - only for debug
    Hash_Map<MapLandmark *,IndexT> map_putative_matches_map_cur_idx_old;
    size_t n_matches_old = n_matches;
    if (display_data.b_enable_display)
    {
      map_putative_matches_map_cur_idx_old = map_putative_matches_landmark_frame_idx;
    }

    // -------------------
    // -- Do tiny BA to refine the pose based on the matches
    // -------------------
    // Time statistics
    time_data.startTimer(time_data.d_feat_tracking_pose_opt_rf);

    bool b_use_robust_function = true;
    if (!performPoseOptimization(frame_track_current.get(), map_putative_matches_landmark_frame_idx,b_use_robust_function))
    {
      // Time statistics
      time_data.stopTimer(time_data.d_feat_tracking_pose_opt_rf);
      return false;
    }
    n_matches = map_putative_matches_landmark_frame_idx.size();

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] RF outlier removal (before/after): "<<n_matches_old<<"/"<<n_matches<<"\n";

    // Time statistics
    time_data.stopTimer(time_data.d_feat_tracking_rf);

    if (time_data.b_enable_features_stats)
    {
      time_data.i_matches_rf_outliers = n_matches - n_matches_old;
    }

    // Display matches after outlier removal
    if (display_data.b_enable_display)
    {
      display_data.addDisplayStep("Tracking RF: After BA and Outlier remvoal",frame_track_current.get(), map_putative_matches_map_cur_idx_old, map_putative_matches_landmark_frame_idx, 4);
    }

    if (n_matches < params_->track_min_matches)
    {
      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Match by Reference frame Failed. # Matches: " << n_matches << "\n";
      return false;
    }
    return true;
  }

  // --------------------------
  //   Tracking: Local map of reference Frame
  // --------------------------
  bool Tracker_Features::trackWithReferenceFrameLocalMap(Frame * frame_reference, Hash_Map<MapLandmark *,IndexT> & map_putative_matches_landmark_frame_idx)
  {
    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Use reference frame: "<<frame_reference->getFrameId()<<"\n";
    
    size_t n_matches = 0;
    Hash_Map<MapLandmark *,IndexT> map_matching_matches_landmark_frame_idx;
    // -------------------
    // -- Try to match landmarks of local map of last reference frame with current frame
    // --  -local map: best N frames that have common landmarks with frame and all their landmarks
    // -------------------
    // -- Identify local map (frames & points)
    std::vector<Frame *> local_map_frames_reference;
    std::vector<MapLandmark *> local_map_landmarks_reference;

    // Identify [N = track_local_map_size] best frames that see the most points that are also seen from current camera
    frame_reference->getFrameVisibilityConnections(local_map_frames_reference,10);

    // Get all unmatched map points from frames in local map
    cartographer_->getLocalMapPoints(frame_reference,local_map_frames_reference,local_map_landmarks_reference);

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Get Frame Visibility. # local frames: " << local_map_frames_reference.size()<<" # local pts: "<<local_map_landmarks_reference.size()<<"\n";

    // -------------------
    // -- Try to match local map landmarks with the features in the current frame
    // -------------------
    feature_matcher_->matching_AllAll_3D_2D
    (
      feature_extractor_,
      local_map_landmarks_reference,
      frame_track_current.get(),
      map_matching_matches_landmark_frame_idx,
      params_->track_match_reloc_desc_ratio,
      params_->track_match_reloc_max_scale_ratio,
      feature_extractor_->f_max_desc_dist_high_
    );
    n_matches = map_matching_matches_landmark_frame_idx.size();

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Match by reference frame map: f: "<<frame_reference->getFrameId()<<" -> "<<frame_track_current->getFrameId()<<" Total # matches: "<<map_putative_matches_landmark_frame_idx.size()<<"\n";

    if (display_data.b_enable_display)
    {
      display_data.addDisplayStep("Tracking Matches All-All reference frame map: ",frame_reference,frame_track_current.get(), map_matching_matches_landmark_frame_idx, -1);
    }

    if (n_matches < params_->track_min_matches)
    {
      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Match by reference frame map Failed. # Matches: " << n_matches << "\n";
      return false;
    }

    // -------------------
    // -- Estimate the pose based on the 3D - 2D matching (use scale from last reference frame)
    // -------------------
    Mat4 T_frame_current;
    std::vector<uint32_t> vec_inliers;
    double f_model_thesh;

    bool b_resection = PoseEstimator::estimateRobustPose_Pinhole
    (
      frame_track_current.get(),
      map_matching_matches_landmark_frame_idx,
      T_frame_current,
      vec_inliers,
      f_model_thesh,
      params_.get()
    );

    // Use scale from reference frame
    T_frame_current.block(0,0,3,3) *= frame_reference->getPoseScale();

    // Check if we got enough matches
    if (!b_resection || vec_inliers.size() < params_->track_min_matches)
    {
      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Match with reference frame map Failed. # Matches: " << vec_inliers.size() << "\n";
      return false;
    }

    // Set pose
    frame_track_current->setPose_T(T_frame_current, nullptr);

    // Insert inliers
    for (auto inlier_i: vec_inliers)
    {
      Hash_Map<MapLandmark *,IndexT>::iterator it = map_matching_matches_landmark_frame_idx.begin();
      std::advance(it,inlier_i);
      map_putative_matches_landmark_frame_idx[it->first] = it->second;
    }

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Matches with Local Map of Reference frame #matches: "<<map_putative_matches_landmark_frame_idx.size()<<"\n";

    if (time_data.b_enable_features_stats)
    {
      time_data.i_matches_rm = map_putative_matches_landmark_frame_idx.size();
    }
    if (display_data.b_enable_display)
    {
      display_data.addDisplayStep("Tracking Inlier Matches reference frame map: ",frame_reference,frame_track_current.get(), map_putative_matches_landmark_frame_idx, -1);
    }


    // Save old version to easily detect which were deleted - only for debug
    Hash_Map<MapLandmark *,IndexT> map_putative_matches_map_cur_idx_old;
    size_t n_matches_old = n_matches;
    if (display_data.b_enable_display)
    {
      map_putative_matches_map_cur_idx_old = map_putative_matches_landmark_frame_idx;
    }

    // -------------------
    // -- Do tiny BA to refine the pose based on the matches
    // -------------------
    // Time statistics
    time_data.startTimer(time_data.d_feat_tracking_pose_opt_rm);
    bool b_use_robust_function = true;
    if (!performPoseOptimization(frame_track_current.get(), map_putative_matches_landmark_frame_idx,b_use_robust_function))
    {
      // Time statistics
      time_data.stopTimer(time_data.d_feat_tracking_pose_opt_rm);
      return false;
    }
    n_matches = map_putative_matches_landmark_frame_idx.size();

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] RF Map outlier removal (before/after): "<<n_matches_old<<"/"<<n_matches<<"\n";

    // Time statistics
    time_data.stopTimer(time_data.d_feat_tracking_pose_opt_rm);

    // Display matches after outlier removal
    if (display_data.b_enable_display)
    {
      display_data.addDisplayStep("Tracking RF Map: After BA and Outlier remvoal",frame_track_current.get(), map_putative_matches_map_cur_idx_old, map_putative_matches_landmark_frame_idx, 4);
    }

    if (n_matches < params_->track_min_matches)
    {
      if (verbose_level > 1)
        std::cout<<"Tracker: [Track] Match with reference frame map Failed\n";
      return false;
    }
    return true;
  }

  // --------------------------
  //   Tracking: Local map of current frame
  // --------------------------
  bool Tracker_Features::trackLocalMap()
  {
    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Track Local Map: \n";

    size_t n_matches = 0;
    Hash_Map<MapLandmark *,IndexT> map_putative_matches_map_frame_idx;

    // Local map
    std::vector<Frame *> local_map_frames;
    std::vector<MapLandmark *> local_map_landmarks;

    // Identify [N = track_local_map_size] best frames that see the most points that are also seen from current camera
    frame_track_current->getFrameVisibilityConnections(local_map_frames,params_->track_local_map_n_frames);

    // Get all unmatched map points from frames in local map
    cartographer_->getLocalMapPoints(frame_track_current.get(),local_map_frames,local_map_landmarks);

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Get local map. # local frames: " << local_map_frames.size()<<" # local pts: "<<local_map_landmarks.size()<<"\n";

    // If relocalization was recent we increase the search window

    feature_matcher_->matching_Projection_3D_2D
    (
      feature_extractor_,
      local_map_landmarks,
      frame_track_current.get(),
      map_putative_matches_map_frame_idx,
      params_->track_match_lm_win_size * (frame_last_relocalization_id+2 > frame_track_current->getFrameId() ? 2 : 1),
      params_->track_match_lm_desc_ratio,
      params_->track_match_lm_max_scale_ratio,
      feature_extractor_->f_max_desc_dist_high_
    );
    n_matches = map_putative_matches_map_frame_idx.size();

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Total # matches with local map: "<<map_putative_matches_map_frame_idx.size()<<"\n";

    // Save old version to easily detect which were deleted - only for debug
    Hash_Map<MapLandmark *,IndexT> map_putative_matches_map_cur_idx_old;
    size_t n_matches_old = n_matches;
    if (display_data.b_enable_display)
    {
      map_putative_matches_map_cur_idx_old = map_putative_matches_map_frame_idx;
    }
    // -------------------
    // -- Do tiny BA to refine the pose based on the matches
    // -------------------
    // Time statistics
    time_data.startTimer(time_data.d_feat_tracking_pose_opt_lm);

    bool b_use_robust_function = true;
    if (!performPoseOptimization(frame_track_current.get(), map_putative_matches_map_frame_idx,b_use_robust_function))
    {
      // Time statistics
      time_data.stopTimer(time_data.d_feat_tracking_pose_opt_lm);
      return false;
    }
    // Mark new inliers in the frame
    associateLandmarksWithFrame(frame_track_current.get(),map_putative_matches_map_frame_idx,5);

    n_matches = map_putative_matches_map_frame_idx.size();

    if (verbose_level > 1)
      std::cout<<"Tracker: [Track] Local Map matches (before/after): "<<n_matches_old<<"/"<<n_matches<<"\n";

    // Time statistics
    time_data.stopTimer(time_data.d_feat_tracking_pose_opt_lm);

    if (time_data.b_enable_features_stats)
    {
      time_data.i_matches_lm = n_matches;
    }

    // Display matches after outlier removal
    if (display_data.b_enable_display)
    {
      display_data.addDisplayStep("Tracking Local Map : matches with map",frame_track_current.get(), map_putative_matches_map_cur_idx_old, map_putative_matches_map_frame_idx, 4);
    }

    // Save local map for display
    display_data.display_local_map = local_map_landmarks;

    return true;

  }


  // -------------------
  // -- Outlier removal
  // -------------------
  void Tracker_Features::removeOutliersInFrame
  (
    Frame * frame
  )
  {
    const size_t & frame_id = frame->getFrameId();

    #ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel for schedule(dynamic)
    #endif
    for (IndexT feat_i = 0; feat_i < frame->getNumberOfFeatures(); ++feat_i)
    {
      // Get Landmark data
      MapLandmark * map_landmark = frame->getLandmark(feat_i);
      if (!map_landmark)
        continue;

      // Project point to frame coordinate system
      Vec2 pt_2D_frame;
      if (!frame->getProjectedPoint(map_landmark,pt_2D_frame) || !frame->checkFeatureAssociation(pt_2D_frame,feat_i,5.991))
      {
        frame->removeLandmark(feat_i);
        map_landmark->decreaseNumberOfObservations();
      }
    }
  }
  void Tracker_Features::removeOutliersInMatches
  (
    Frame * frame,
    Hash_Map<MapLandmark *,IndexT> & matches_map_frame_idx
  )
  {
    std::vector<size_t> vec_matches_outliers;

    const size_t & frame_id = frame->getFrameId();

    //#ifdef OPENMVG_USE_OPENMP
    //#pragma omp parallel for schedule(dynamic)
    //#endif
    for (size_t match_i = 0; match_i < matches_map_frame_idx.size(); ++match_i)
    {
      Hash_Map<MapLandmark *,IndexT>::iterator it_match = matches_map_frame_idx.begin();
      std::advance(it_match,match_i);

      // Get Landmark data
      MapLandmark * map_landmark = it_match->first;
      const IndexT feat_id_frame = it_match->second;

      // Project point to frame coordinate system
      Vec2 pt_2D_frame;
      if (!frame->getProjectedPoint(map_landmark,pt_2D_frame) || !frame->checkFeatureAssociation(pt_2D_frame,feat_id_frame,5.991))
      {
        // Outlier
        #pragma omp critical
        {
          vec_matches_outliers.push_back(match_i);
        }
      }
    }
    // sort indexes of outliers
    std::sort(vec_matches_outliers.begin(),vec_matches_outliers.end());

    // Remove any outlier matches
    for (size_t o_i = 0; o_i < vec_matches_outliers.size(); ++o_i)
    {
      Hash_Map<MapLandmark *,IndexT>::iterator it_outlier = matches_map_frame_idx.begin();
      // Reduce the number by the number of elements already deleted
      std::advance(it_outlier,vec_matches_outliers[o_i]-o_i);

      // Delete 3D point (all other references have been deleted before)
      matches_map_frame_idx.erase(it_outlier);
    }
  }

  // If we find an outlier in a candidate we remove the candidate
  void Tracker_Features::removeOutliersInCandidateLandmarks
  (
    Frame * frame,
    NewMapLandmarks & vec_new_landmarks
  )
  {
    std::vector<size_t> vec_outliers;
    //#ifdef OPENMVG_USE_OPENMP
    //#pragma omp parallel for schedule(dynamic)
    //#endif
    for(size_t t_i = 0; t_i < vec_new_landmarks.size(); ++t_i)
    {
      bool b_sufficient_parallax = false;
      bool b_all_obs_ok = true;

      // Get landmark
      MapLandmark * map_landmark = vec_new_landmarks[t_i].get();
      LandmarkObservations & obs = map_landmark->getObservations();
      const size_t n_feat_obs_init = obs.size();

      // Get observation in current frame
      Vec2 pt_2D_frame;
      const size_t & frame_id = frame->getFrameId();
      const MapObservation & m_o = obs.find(frame_id)->second;
      const Vec3 ray_frame = frame->getRayToMapLandmark(map_landmark);

      if (!frame->getProjectedPoint(map_landmark,pt_2D_frame) || !frame->checkFeatureAssociation(pt_2D_frame,m_o.feat_id,5.991))
      {
        // Delete point from the vector of new points
        #pragma omp critical
        {
          vec_outliers.push_back(t_i);
        }
        continue;
      }

      // Check the rest of the frames and check if there is any point with sufficient parallax
      for(LandmarkObservations::iterator iter_mo = obs.begin(); iter_mo != obs.end();)
      {
        MapObservation & m_o_i =  iter_mo->second;
        Frame * & frame_i = m_o_i.frame_ptr;
        // Put 3D point into coordinate system of frame
        Vec2 pt_2D_frame_i;

        if (!frame_i->getProjectedPoint(map_landmark,pt_2D_frame_i) || !frame_i->checkFeatureAssociation(pt_2D_frame_i,m_o_i.feat_id,5.991))
        {
          b_all_obs_ok = false;
          break;
          // Remove measurement from 3D point
          //iter_mo = obs.erase(iter_mo);
          //map_landmark->decreaseNumberOfObservations();
        }
        else
        {
          if (!b_sufficient_parallax)
          {
            // Compute ray angle between points
            const Vec3 ray_frame_i = frame_i->getRayToMapLandmark(map_landmark);

            const double cosParallax = PoseEstimator::computeCosParallaxBetweenRays(ray_frame,ray_frame_i);

            // If angle is smaller than threshold delete it
            if (cosParallax < params_->init_track_min_cos_parallax_pt)
            {
              b_sufficient_parallax = true;
            }
          }
          ++iter_mo;
        }
      }

      if (!b_all_obs_ok || !b_sufficient_parallax)
      {
        // Delete point from the vector of new points
        #pragma omp critical
        {
          vec_outliers.push_back(t_i);
        }
      }
      else if (obs.size()>2)
      {
        // Mark that point was triangulated from multiple points
        map_landmark->association_type_ = 7;
      }
    }

    // sort indexes of outliers
    std::sort(vec_outliers.begin(),vec_outliers.end());

    // Remove any triangulated landmarks that dont have enough measurements or are not ok in this frame
    for (size_t o_i = 0; o_i < vec_outliers.size(); ++o_i)
    {
      NewMapLandmarks::iterator it_outlier = vec_new_landmarks.begin();
      // Reduce the number by the number of elements already deleted
      std::advance(it_outlier,vec_outliers[o_i]-o_i);

      // Delete 3D point (all other references have been deleted before)
      vec_new_landmarks.erase(it_outlier);
    }
  }

}
}
