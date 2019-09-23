// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "openMVG/types.hpp"
#include <openMVG/vsslam/display/vsslam_display.hpp>
#include <openMVG/vsslam/tracking/Tracker_Features.hpp>



namespace openMVG {
namespace vsslam {

  bool Tracker_Features::trackingInitialization
  (
    const image::Image<unsigned char> & ima
  )
  {
    if (tracking_status_ == TRACKING_STATUS::NOT_INIT)
    {
      // Set new reference initialization frame
      startInitialization();
      return false;
    }
    else if (tracking_status_ == TRACKING_STATUS::INIT)
    {
      // Reset tracking if we dont already have reference frame (shouldnt happen)
      if (!frame_track_init)
      {
        if (verbose_level > 1)
          std::cout<<"Tracker: [Initialization] Missing reference frame! Start initialization with current\n";
       
        startInitialization();
        return false;
      }


      // -------------------
      // -- Match features of two images
      // -------------------

      matching::IndMatches vec_putative_matches_ref_cur_idx;

      // Match all-all features
      feature_matcher_->matching_AllAll_2D_2D
      (
        feature_extractor_,
        frame_track_init.get(),
        frame_track_current.get(),
        vec_putative_matches_ref_cur_idx,
        params_->match_init_desc_ratio,
        params_->match_init_max_scale_ratio,
        feature_extractor_->f_max_desc_dist_high_
      );


      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization] [Matching All-All 2D-2D]: " << vec_putative_matches_ref_cur_idx.size() << "\n";

      if (display_data.b_enable_display)
        display_data.addDisplayStep("Initialization: Initial 2D-2D matches",frame_track_init.get(), frame_track_current.get(),vec_putative_matches_ref_cur_idx);

      // If we dont have enough matches with reference image we set current frame as new reference image
      if (vec_putative_matches_ref_cur_idx.size() < params_->init_track_min_matches)
      {
        if (verbose_level > 1)
          std::cout<<"Tracker: [Initialization] Insufficient # matches! Setting this frame as new initialization reference frame\n";
        startInitialization();
        return false;
      }

      // -------------------
      // -- Estimate relative pose between two images
      // -------------------
      bool b_estimated_motion = false;

      Mat4 T;
      double f_model_thesh;
      std::vector<uint32_t> vec_inliers_M;

      if (frame_track_init->isCamCalibrated() && frame_track_current->isCamCalibrated())
      {
        // Estimate essential / homography matrix
        if (verbose_level > 1)
        {
          std::cout << "Tracker: [Initialization] Estimate relative pose: " << frame_track_init->getFrameId() << " :: " << frame_track_current->getFrameId() << "\n";

        }
        b_estimated_motion = PoseEstimator::estimateRobustRelativePose_HE_Pinhole(frame_track_init.get(), frame_track_current.get(), vec_putative_matches_ref_cur_idx, T, vec_inliers_M, f_model_thesh,params_.get());
      }
      else
      {
        // Estimate fundamental / homography matrix
        if (verbose_level > 1)
          std::cout<<"Tracker: [Initialization] No support for HF initializaton ("<<"-1"<<" s)\n";
        b_estimated_motion = false;
      }

      if(!b_estimated_motion)
      {
        if (verbose_level > 1)
          std::cout<<"Tracker: [Initialization] Motion estimation failed! Try with next frame\n";
        return false;
      }

      if (verbose_level > 1)
      {
        std::cout<<"Tracker: [Initialization] Motion estimation OK: #inliers: "<<vec_inliers_M.size()<<"( "<< time_data.d_pose_init<<" s)\n";
        std::cout<<"Tracker: [Initialization] T:\n"<<T<<"\n";
      }

      // Save for display
      if (display_data.b_enable_display)
      {
        matching::IndMatches vec_show_matches_ref_cur_idx;
        for ( size_t k_i = 0; k_i < vec_inliers_M.size(); ++k_i)
        {
          // Advance to the correct inlier
          matching::IndMatches::iterator m_iter = vec_putative_matches_ref_cur_idx.begin();
          std::advance(m_iter,vec_inliers_M[k_i]);
          vec_show_matches_ref_cur_idx.push_back(*m_iter);
        }
        display_data.addDisplayStep("Initialization: Matches with essential matrix",frame_track_init.get(), frame_track_current.get(),vec_show_matches_ref_cur_idx);
      }


      // -------------------
      // -- Use estimated pose as initial pose estimation
      // -------------------
      frame_track_init->setPose_T_withReferenceFrame(Mat4::Identity(),nullptr);
      frame_track_current->setPose_T_withReferenceFrame(T,nullptr);

      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization] Initial poses set. \n";

      // -------------------
      // -- Triangulate inliers
      // -------------------
      bool b_frame_init_cam_calib = frame_track_init->isCamCalibrated();
      bool b_frame_current_cam_calib = frame_track_current->isCamCalibrated();
      const IntrinsicBase * cam_intrinsic_init = frame_track_init->getCameraIntrinsics();
      const IntrinsicBase * cam_intrinsic_current = frame_track_current->getCameraIntrinsics();

      Mat34 P_init = frame_track_init->getCameraProjectionMatrix(nullptr);
      Mat34 P_current = frame_track_current->getCameraProjectionMatrix(nullptr);


      // Iterate through inliers and triangulate new landmarks
      NewMapLandmarks vec_new_map_landmarks(vec_inliers_M.size());
      /*#ifdef OPENMVG_USE_OPENMP
      #pragma omp parallel for schedule(dynamic)
      #endif*/
      for ( size_t k_i = 0; k_i < vec_inliers_M.size(); ++k_i)
      {
        // Advance to the correct inlier
        matching::IndMatches::iterator m_iter = vec_putative_matches_ref_cur_idx.begin();
        std::advance(m_iter,vec_inliers_M[k_i]);
        // Inlier feat ids
        const IndexT & feat_id_init = m_iter->i_;
        const IndexT & feat_id_current = m_iter->j_;

        // Initialize new map landmark
        std::unique_ptr<MapLandmark> & map_landmark = vec_new_map_landmarks[k_i];
        map_landmark.reset(new MapLandmark());


        // Position of detected features
        //   If we calibrated camera we just use undistorted
        //   If not calibrated we remove the distortion as best as we know it (current estimation)
        Vec3 pt_init = frame_track_init->getFeaturePositionHomogeneous(feat_id_init),
          pt_current = frame_track_current->getFeaturePositionHomogeneous(feat_id_current);

        // Triangulate results
        TriangulateDLT(P_init, pt_init, P_current, pt_current, &(map_landmark->X_));

        // Add observations
        map_landmark->addObservation(frame_track_init.get(),feat_id_init);
        map_landmark->addObservation(frame_track_current.get(),feat_id_current);
        map_landmark->setNumberOfObservations(2);

        // Mark as point used in the motion initialization
        map_landmark->association_type_ = 1;
      }
      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization] Triangulation OK. \n";

      // Save for display
      if (display_data.b_enable_display)
      {
        display_data.addDisplayStep("Initialization: Triangulated points",frame_track_current.get(), vec_new_map_landmarks);
      }


      // -------------------
      // -- Optimize initial poses
      // -------------------
      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization] Initial triangulated points before: "<<vec_new_map_landmarks.size()<<"\n";

      // Set first frame as fixed
      frame_track_init->setActive();

      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization preBA] # of tracked points before clearning: "<<vec_new_map_landmarks.size() << "\n";
      
      removeOutliersInCandidateLandmarks(frame_track_current.get(),vec_new_map_landmarks);
      
      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization preBA] # of tracked points after clearning: "<<vec_new_map_landmarks.size() << "\n";

      // Optimize first pair
      bool b_use_robust_function = true;
      if (!cartographer_->optimizeLocalMap(frame_track_current.get(), vec_new_map_landmarks,b_use_robust_function))
      {
        if (verbose_level > 1)
          std::cout<<"Tracker: [Initialization] Optimize local FAILED\n";
        return false;
      }

      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization] # of tracked points before clearning: "<<vec_new_map_landmarks.size() << "\n";
      
      removeOutliersInCandidateLandmarks(frame_track_current.get(),vec_new_map_landmarks);
    
      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization] # of tracked points after clearning: "<<vec_new_map_landmarks.size() << "\n";

      // Save for display
      if (display_data.b_enable_display)
      {
        display_data.addDisplayStep("Initialization: Triangulated points after BA",frame_track_current.get(), vec_new_map_landmarks);
      }

      // -------------------
      // -- Check that there are enough tracked points
      // -------------------
      if (vec_new_map_landmarks.size() < params_->init_track_min_matches)
      {
        if (verbose_level > 1)
          std::cout<<"Tracker: [Initialization] Insufficient # of tracked points: "<<vec_new_map_landmarks.size()<<"! Setting this frame as new initialization reference frame\n";
        startInitialization();
        return false;
      }

      // -------------------
      // -- Add first two frames to map
      // -------------------
      cartographer_->addStep(frame_track_init, nullptr);
      cartographer_->addStep(frame_track_current, &vec_new_map_landmarks);

      // -------------------
      // -- Update motion model
      // -------------------
      motion_model_.update(frame_track_init.get(),frame_track_current.get());

      // -------------------
      // -- Update last reference camera
      // -------------------
      frame_track_last_reference = frame_track_current->share_ptr();

      // Clear initialization data
      clearInitializationData();


      // Set tracking to OK
      if (verbose_level > 1)
        std::cout<<"Tracker: [Initialization] Set system status to OK\n";
      tracking_status_ = TRACKING_STATUS::OK;
      return true;
    }

  }

  void Tracker_Features::startInitialization()
  {
    // Clear all initialization settings
    clearInitializationData();

    // Set current frame as the new reference frame for initialization
    if (verbose_level > 1)
      std::cout<<"Tracker: [Initialization] Set system initialization reference frame\n";

    frame_track_init = frame_track_current->share_ptr();
    // Set system status to INIT
    tracking_status_ = TRACKING_STATUS::INIT;
  }
  void Tracker_Features::resetInitialization()
  {
    if (verbose_level > 1)
      std::cout<<"Tracker: [Initialization] Reset system initialization process!\n";

    clearInitializationData();
    motion_model_.setInvalid();
    tracking_status_ = TRACKING_STATUS::NOT_INIT;

  }
  void Tracker_Features::clearInitializationData()
  {
    if (verbose_level > 1)
      std::cout << "Tracker: [Initialization] Clear tracking initialization data!\n";

    if (frame_track_init) {
      frame_track_init.reset();
    }
  }

}
}
