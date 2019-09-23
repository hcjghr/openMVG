// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <openMVG/vsslam/system/Frame.hpp>

namespace openMVG {
namespace vsslam {

class Frame;

/// Observation of a landmark in an image (feature)
struct MapObservation
{
  MapObservation()
  : feat_id (UndefinedIndexT),
    frame_ptr(nullptr)
  {}
  MapObservation(IndexT f_id, Frame * frame)
  : feat_id (f_id),
    frame_ptr(frame)
  {}

  IndexT feat_id;
  Frame * frame_ptr;
};

/// Define a collection of observations of a landmark
using LandmarkObservations = Hash_Map<size_t, MapObservation>;


class MapLandmark
{
public:
  IndexT id_;
  Vec3 X_;
  Eigen::Matrix3d cov_X_;

  // 1 - initialization point; 2- motion model/reference kf; 3-map tracking point; 4- new triangulated point
  size_t association_type_ = 0; // Through which tye of association the point was added
  IndexT last_local_map_frame_id_ = UndefinedIndexT;  // Id of frame for which the point was last added to local map

private:
  bool active_; // True if the point is in global map

  Frame * frame_reference_;

  // Observations
  LandmarkObservations obs_;  // map of keyframe_ids and map observation objects

  // Matching of landmark
  void * feat_best_desc_; // Pointer to the best descriptor of the point (most average of them all?)
  float feat_mean_scale_;  // Last scale of the features associated

  size_t n_all_obs_ = 0; // Number of all observations (both frames and keyframes)

  size_t last_observed_in_step_;


public:
  const bool & isActive() const
  {
    return active_;
  }

  void setActive()
  {
    active_ = true;
  }

  void * & getBestDesc()
  {
    return feat_best_desc_;
  }
  const Vec3 & getWorldPosition() const
  {
    if (frame_reference_ != nullptr)
    {
      std::cout<<" Relative landmark positions NOT IMPLEMENTED...YET\n";
      return X_;
    }

    return X_;
  }

  const Frame * getReferenceFrame() const
  {
    return frame_reference_;
  }

  const float & getFeatureMeanScale() const
  {
    return feat_mean_scale_;
  }


  // ------------------------------
  // -- Observations
  // ------------------------------
  LandmarkObservations & getObservations()
  {
    return obs_;
  }
  MapObservation & getObservation(const IndexT & feat_id)
  {
    return obs_[feat_id];
  }

  void addObservation(Frame * frame, const IndexT & feat_id);

  bool hasFrameObservation(const IndexT & frame_id);

  void setNumberOfObservations(size_t n_obs)
  {
    n_all_obs_ = n_obs;
  }
  void increaseNumberOfObservations()
  {
    n_all_obs_++;
  }
  void decreaseNumberOfObservations()
  {
    n_all_obs_ > 0 ? n_all_obs_-- : 0;
  }
  size_t getNumberOfObservations()
  {
    return n_all_obs_;
  }

  size_t & getObservedInStep()
  {
    return last_observed_in_step_;
  }
  void setObservedInStep(const size_t & step_id)
  {
    // Update only the step is bigger than before (should normally be)
    if (last_observed_in_step_ < step_id)
      last_observed_in_step_ = step_id;
  }

  void updateData();

  float computeVolumeOfUncertainty(float chi_2)
  {
    //return (2.0 * pow(M_PI, 1.5)) / (3*(std::sqrt(M_PI)/pow(2.0,1.5)));
    return ((2.0 * pow(M_PI, 1.5)) / (3*(std::sqrt(M_PI)/pow(2.0,1.5)))) * (pow(chi_2, 1.5) * pow(cov_X_.determinant(), 0.5));
  }

};

}
}
