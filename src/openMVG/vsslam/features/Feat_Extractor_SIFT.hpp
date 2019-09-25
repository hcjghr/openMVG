// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <openMVG/types.hpp>
#include "nonFree/sift/SIFT_describer.hpp"
#include <openMVG/vsslam/features/Abstract_Feature_Extractor.hpp>

namespace openMVG {
namespace vsslam {

class Feat_Extractor_SIFT : public Abstract_Feature_Extractor
{
private:
  using RegionT = features::SIFT_Regions;
  std::unique_ptr<features::Image_describer> image_describer;
public:
  Feat_Extractor_SIFT
  (
    std::shared_ptr<VSSLAM_Parameters> & params,
    features::EDESCRIBER_PRESET preset = features::NORMAL_PRESET
  )
  : Abstract_Feature_Extractor(params)
  {
    // Set feature dependent thresholds
    f_max_desc_dist_high_ = 100;
    f_max_desc_dist_low_ = 150;

    // Initialize detector/descriptor
    image_describer.reset(new features::SIFT_Image_describer
      (features::SIFT_Image_describer::Params(), false));

    //image_describer.reset(
    //  new features::SIFT_Anatomy_Image_describer(features::SIFT_Anatomy_Image_describer::Params()));
    image_describer->Set_configuration_preset(preset);
  }
  
  Feat_Extractor_SIFT
  (
    std::shared_ptr<VSSLAM_Parameters> & params,
    float thresh
  )
  : Abstract_Feature_Extractor(params)
  {
    // Set feature dependent thresholds
    f_max_desc_dist_high_ = 100;
    f_max_desc_dist_low_ = 150;

    auto sift_params = features::SIFT_Image_describer::Params();
    sift_params._peak_threshold = thresh;
    // Initialize detector/descriptor
    image_describer.reset(new features::SIFT_Image_describer
      (sift_params, false));
  }

  size_t getDescriptorLength() const override
  {
    return RegionT::DescriptorT::static_size;
  }

  // suggest new feature point for tracking (count point are kept)
  size_t detect
  (
    const image::Image<unsigned char> & ima,
    Frame * frame,
    const image::Image<unsigned char> * mask = nullptr
  ) const override
  {
    // Cast region
    std::unique_ptr<features::Regions> & regions_frame = frame->getRegionsPtr();
    regions_frame.reset(new RegionT);

    // Detect keypoints
    image_describer->Describe(ima, regions_frame, mask);

    // Get number of detected keypoints
    const size_t n_features = regions_frame->RegionCount();

    // Prepare the size of vectors
    std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> > & vec_feat_inf = frame->getFeatureSqrtInfMatrixVector();
    std::vector<float> & vec_feat_scale = frame->getFeatureScaleVector();
    vec_feat_inf.resize(n_features);
    vec_feat_scale.resize(n_features);

    // Estimate the uncertainty of each measurement
    const double d_sigma_inv_px = 1.0f / 4.0f;  // assume 4px sigma error
    std::vector<RegionT::FeatureT> vec_features = dynamic_cast<RegionT *>(regions_frame.get())->Features();
    for (size_t i = 0; i < vec_features.size(); i++)
    {
      vec_feat_scale[i] = vec_features[i].scale();
      vec_feat_inf[i] = Eigen::Matrix2d::Identity() * d_sigma_inv_px;
    }

    return n_features;
  }

  bool describe
  (
    const image::Image<unsigned char> & ima,
    const Frame * frame
  ) const override
  {
    // Features are already described in detect function
    return true;
  }

  void getDescriptorRaw
  (
    features::Regions * const regions,
    const IndexT i,
    void ** desc
  ) const override
  {
    *desc = (dynamic_cast<RegionT *>(regions)->Descriptors()[i].data());
  }


  double getDescriptorDistanceSquared
  (
    void * desc_A,
    void * desc_B
  ) const override
  {
    openMVG::matching::L2<RegionT::DescriptorT::bin_type> metric;
    RegionT::DescriptorT::bin_type * d_A = static_cast<RegionT::DescriptorT::bin_type *>(desc_A);
    RegionT::DescriptorT::bin_type * d_B = static_cast<RegionT::DescriptorT::bin_type *>(desc_B);

    return metric(d_A, d_B, RegionT::DescriptorT::static_size);
  }

};

}
}
