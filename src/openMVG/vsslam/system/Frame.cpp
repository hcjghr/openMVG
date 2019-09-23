// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include <openMVG/vsslam/system/Frame.hpp>

namespace openMVG {
namespace vsslam {
  Frame::Frame
  (
    const IndexT & id_frame,
    const double & timestamp,
    Camera * ptr_cam
  ): id_(id_frame), d_timestamp_(timestamp), ptr_cam_(ptr_cam), b_active_(false)
  {
    T_ = Mat4::Identity();
    T_inv_ = Mat4::Identity();
  }

  void Frame::updateFeaturesData()
  {
    // Update number of features detected
    n_feats_ = regions_->RegionCount();
    // Initialize pointers to map points (to nullptr)
    map_landmark_ = std::vector<MapLandmark *>(n_feats_,static_cast<MapLandmark *>(nullptr));

    // Undistort points
    // If calibration is known undistort points (save process time later)
    // Otherwise just save distorted points
    vec_pts_undist_.resize(n_feats_);
    undistortPoints();
  }

  void Frame::undistortPoints()
  {
    // If we have distortion and can model it we remove it for faster processing later
    if (ptr_cam_->ptr_intrinsic_->have_disto() && ptr_cam_->isCalibrated())
    {
      #ifdef OPENMVG_USE_OPENMP
      #pragma omp parallel for
      #endif
      for ( IndexT i = 0; i < n_feats_; ++i )
      {
        vec_pts_undist_[i] = ptr_cam_->ptr_intrinsic_->remove_disto(regions_->GetRegionPosition(i));
      }
    }
    else
    {
      #ifdef OPENMVG_USE_OPENMP
      #pragma omp parallel for
      #endif
      for ( IndexT i = 0; i < n_feats_; ++i )
      {
        vec_pts_undist_[i] = regions_->GetRegionPosition(i);
      }
    }
  }


  bool Frame::getProjectedPoint(const Vec3 & pt_3D, const Frame * pt_3D_reference, Vec2 & pt_2D_frame) const
  {
    Vec3 pt_3D_frame;
    PoseEstimator::getRelativePointPosition(pt_3D,pt_3D_reference,pt_3D_frame,this);

    if (pt_3D_frame(2)<=0)
      return false;

    pt_2D_frame = getCameraIntrinsics()->cam2ima(getCameraIntrinsics()->have_disto()?getCameraIntrinsics()->add_disto(pt_3D_frame.hnormalized()):pt_3D_frame.hnormalized());

    return isPointInFrame(pt_2D_frame);
  }

  bool Frame::getProjectedPoint(const MapLandmark * map_landmark, Vec2 & pt_2D_frame) const
  {
    Vec3 pt_3D_frame;

    PoseEstimator::getRelativePointPosition(map_landmark->X_,map_landmark->getReferenceFrame(),pt_3D_frame,this);

    if (pt_3D_frame(2)<=0)
      return false;

    pt_2D_frame = getCameraIntrinsics()->cam2ima(getCameraIntrinsics()->have_disto()?getCameraIntrinsics()->add_disto(pt_3D_frame.hnormalized()):pt_3D_frame.hnormalized());

    return isPointInFrame(pt_2D_frame);
  }

  Vec3 Frame::getRayToMapLandmark(const MapLandmark * map_landmark)
  {
    return getRayToPoint(map_landmark->X_, map_landmark->getReferenceFrame());
  }

  Vec3 Frame::getRayToPoint(const Vec3 & pt_3D, const Frame * pt_3D_reference)
  {
    Vec3 pt_3D_frame;
    PoseEstimator::getRelativePointPosition(pt_3D,pt_3D_reference,pt_3D_frame,this);
    return Vec3(R_ * pt_3D_frame).normalized();
  }

  Vec3 Frame::getRayToPoint(const IndexT & feat_id)
  {
    return getRayToPoint(getFeaturePositionHomogeneous(feat_id), this);
  }
;

  bool Frame::isPointInFrame(const Vec2 & pt) const
  {
    return ptr_cam_->isPointInImage(pt);
  }

  bool Frame::checkFeatureAssociation(const Vec3 & pt_3D_frame, const IndexT feat_id, double thresh) const
  {
    IntrinsicBase * cam_intrinsic = getCameraIntrinsics();
    Vec2 pt_3D_frame_proj = cam_intrinsic->cam2ima(pt_3D_frame.hnormalized());

    // Check if projection is actually in the image borders
    if (!isPointInFrame(pt_3D_frame_proj))
      return false;

    // Error between projection and measurement
    const Vec2 pt_err = vec_pts_undist_[feat_id] - pt_3D_frame_proj;

    //// Chi2 error
    return (pt_err.transpose() * vec_pts_sqrt_inf_mat_[feat_id].cwiseProduct(vec_pts_sqrt_inf_mat_[feat_id]) * pt_err) < thresh;
  }

  bool Frame::checkFeatureAssociation(const Vec2 & pt_2D_frame, const IndexT feat_id, double thresh) const
  {
    // Error between projection and measurement
    const Vec2 pt_err = vec_pts_undist_[feat_id] - pt_2D_frame;

    //// Chi2 error
    return (pt_err.transpose() * vec_pts_sqrt_inf_mat_[feat_id].cwiseProduct(vec_pts_sqrt_inf_mat_[feat_id]) * pt_err) < thresh;
  }

  bool Frame::checkLandmarkPosition(const Vec3 & pt_3D_frame) const
  {
    if (pt_3D_frame(2)>0)
      return true;
    return false;
  }

  void Frame::setLandmark(IndexT & feat_id, MapLandmark * map_landmark)
  {
    map_landmark_[feat_id] = map_landmark;
  }
  void Frame::removeLandmark(IndexT & feat_id)
  {
    map_landmark_[feat_id] = static_cast<MapLandmark *>(nullptr);
  }

  size_t Frame::getNumberOfMapPoints(bool b_only_global) const
  {
    IndexT n_matches = 0;
    for (IndexT i = 0; i<map_landmark_.size(); ++i)
    {
      if (map_landmark_[i] && (!b_only_global || map_landmark_[i]->isActive()))
        n_matches++;
    }
    return n_matches;
  }

  void Frame::computeSceneStatistics()
  {
    std::vector<float> vec_depths;
    vec_depths.reserve(n_feats_);

    for (size_t landmark_i=0; landmark_i < map_landmark_.size(); ++landmark_i)
    {
      if (!map_landmark_[landmark_i])
        continue;

      MapLandmark * map_landmark = map_landmark_[landmark_i];
      // Get depth of Landmark with respect to frame
      vec_depths.emplace_back(getMapLandmarkDepth(map_landmark));
    }

    f_scene_median_ = vec_depths[(vec_depths.size()-1)/2];
  }

  // ------------------------------------------------
  //  Pose
  // ------------------------------------------------
  void Frame::updatePoseData(bool b_inverse_updated)
  {
    if (frame_reference_ != nullptr)
    {
      std::cout<<"Relative poses ARE NOT IMPLEMENTED...YET!\n";
      return;
    }

    if (b_inverse_updated)
      T_ = T_inv_.inverse();
    else
      T_inv_ = T_.inverse();

    R_ = T_.block(0,0,3,3) / T_.block(0,0,3,1).norm();
    R_inv_ = R_.transpose();
    origin_ = T_.block(0,3,3,1);

    if (isCamCalibrated())
    {
      P_ = dynamic_cast<const Pinhole_Intrinsic*>(getCameraIntrinsics())->K() * T_inv_.block(0,0,3,4);
    }

  }

  void Frame::setPose_T(const Mat4 & T, Frame * frame_reference)
  {
    if (frame_reference_ != frame_reference)
    {
      std::cout<<"Relative poses ARE NOT IMPLEMENTED...YET!\n";
      return;
    }

    T_ = T;
    updatePoseData(false);
  }

  void Frame::setPose_sim3
  (
    Eigen::Matrix<double, 7, 1> & vec_state,
    Frame * frame_reference
  )
  {
    if (frame_reference_ != frame_reference)
    {
      std::cout<<"Relative poses ARE NOT IMPLEMENTED...YET!\n";
      return;
    }
    Mat4 T;
    // From sim3 to Sim3
    Sim3_exp(vec_state,T);
    T_ = T;
    updatePoseData(false);
  }


  void Frame::setPose_sRt_Inverse(const Mat3 & R, const Vec3 & t, const double & s, Frame * frame_reference)
  {
    if (frame_reference_ != frame_reference)
    {
      std::cout<<"Relative poses ARE NOT IMPLEMENTED...YET!\n";
      return;
    }

    T_inv_ = Mat4::Identity();
    T_inv_.block(0,0,3,3) = s*R;
    T_inv_.block(0,3,3,1) = t;
    updatePoseData(true);
  }

  void Frame::setPose_T_withReferenceFrame (const Mat4 &T, Frame * frame_new_reference)
  {
    frame_reference_ = frame_new_reference;
    setPose_T(T, frame_new_reference);
  }

  void Frame::getPose_sRt_Inverse
  (
      Mat3 & R,
      Vec3 & t,
      double & s,
      Frame * frame_reference
  ) const
  {
    if (frame_reference_ != frame_reference)
    {
      std::cout<<"Relative poses ARE NOT IMPLEMENTED...YET!\n";
    }

    // T = [sR t]
    s = T_inv_.block(0,0,3,1).norm();
    R = T_inv_.block(0,0,3,3) / s;
    t = T_inv_.block(0,3,3,1);

  }


  void Frame::getPose_StateVector
  (
    Eigen::Matrix<double, 12, 1> & vec_state,
    Frame * frame_reference
  ) const
  {
    if (frame_reference_ != frame_reference)
    {
      std::cout<<"Relative poses ARE NOT IMPLEMENTED...YET!\n";
    }

    Eigen::Matrix<double,7,1> log_state;
    Sim3_log(T_,log_state);


    vec_state.template head<7>() = log_state;
    // Augment state vector with camera parameters

    const Mat3 K = getK();
    // fx,fy,ppx,ppy,d
    // TODO: Check how is with distortion parameters
    vec_state.template tail<5>() << K(0,0), K(0,0), K(0,2) , K(1,2), 0.0;


  }

  void Frame::getPose_T
  (
      Mat4 & T,
      Frame * frame_reference
  ) const
  {
    if (frame_reference_ != frame_reference)
    {
      std::cout<<"Relative poses ARE NOT IMPLEMENTED...YET!\n";
    }

    T = T_;
  }

  Mat34 Frame::getCameraProjectionMatrix(Frame * frame_reference)
  {
    if (frame_reference_ != frame_reference)
    {
      std::cout<<"Relative poses ARE NOT IMPLEMENTED...YET!\n";
      return Mat34::Zero();
    }

    if (isCamCalibrated())
    {
      return P_;
    }
    else
    {
      return dynamic_cast<const Pinhole_Intrinsic*>(getCameraIntrinsics())->K() * T_.block(0,0,3,4);
    }
  }

  // Get camera center expressed in world coordinates (regardless of representation of camera)
  const Vec3 Frame::getCameraCenter() const
  {
    return T_.block(0,3,3,1);
  }

  const float Frame::getMapLandmarkDepth(MapLandmark * map_landmark) const
  {
    Vec3 pt_3D_frame;
    PoseEstimator::getRelativePointPosition(map_landmark->X_,map_landmark->getReferenceFrame(),pt_3D_frame,this);
    return pt_3D_frame(2);
  }


  void Frame::getFrameVisibilityConnections(std::vector<Frame *> & frames_connected_ordered, const size_t n_best) const
  {
    // Min points two frames have to have in common to be considered a pair
    int min_common_pts = 0;//15; // Klemen - so that it always tries to add
    // Loop through all matched points of the frame and vote for each frame that sees any of the points
    Hash_Map<Frame *, size_t> frames_connected_weights;
    for (MapLandmark * map_landmark: map_landmark_)
    {
      if (!map_landmark)
        continue;
      LandmarkObservations & map_obs = map_landmark->getObservations();
      for (auto obs: map_obs)
      {
        if (obs.first == id_)
          continue;
        if (frames_connected_weights.find(obs.second.frame_ptr) == frames_connected_weights.end())
          frames_connected_weights[obs.second.frame_ptr]=1;
        else
          frames_connected_weights[obs.second.frame_ptr]++;
      }
    }
    if (frames_connected_weights.empty())
    {
      frames_connected_ordered.clear();
      return;
    }
    // Add all connections to frames that have at least min_points in common
    std::vector<std::pair<size_t,Frame*> > vec_frames_n_landmarks;
    vec_frames_n_landmarks.reserve(frames_connected_weights.size());

    for (auto it_frame : frames_connected_weights)
    {
      if (it_frame.second > min_common_pts)
      {
        vec_frames_n_landmarks.emplace_back(std::make_pair(it_frame.second, it_frame.first));
      }
    }
    // if none of the frames has more
    if (vec_frames_n_landmarks.empty())
    {
      frames_connected_ordered.clear();
      return;
    }
    // Determine how many frames we want to get back
    const size_t n_frames = vec_frames_n_landmarks.size();
    const size_t n_return_frames = (n_best==0 ? n_frames : std::min<size_t>(n_frames,n_best));
    // Sort all the frames
    std::sort(vec_frames_n_landmarks.begin(),vec_frames_n_landmarks.end());
    // Return N best
    frames_connected_ordered.resize(n_return_frames);
    for (size_t i = 0; i < n_return_frames; i++)
    {
      frames_connected_ordered[i] = vec_frames_n_landmarks[n_frames-i-1].second;
    }
/*
    for (size_t i = 0; i < n_return_frames; i++)
    {
      std::cout<<"Local frame: "<<frames_connected_ordered[i]->getFrameId()<<" #: "<<vec_frames_n_landmarks[n_frames-i-1].first<<"\n";
    }
*/
  }


}
}
