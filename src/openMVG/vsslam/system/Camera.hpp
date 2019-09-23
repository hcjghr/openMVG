// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>

#include <openMVG/types.hpp>
#include "openMVG/stl/split.hpp"
//#include "openMVG/image/image.hpp" // swine
#include "openMVG/image/image_container.hpp" // swine
#include "openMVG/image/image_io.hpp" // swine
#include <openMVG/cameras/cameras.hpp>

namespace openMVG {
namespace vsslam {

using namespace openMVG::cameras;
using namespace openMVG::image;

struct CameraParameters
{
  EINTRINSIC camera_model;
  double focal = -1;
  double ppx = -1;
  double ppy = -1;
  size_t img_width = 0;
  size_t img_height = 0;
  std::vector<double> dist;
  bool b_calibrated = true;

  bool readImageSettings(const std::string & sImagePath)
  {
    image::ImageHeader imgHeader;

    if (!image::ReadImageHeader(sImagePath.c_str(), &imgHeader))
    {
      std::cerr << "\nError reading image header file" << std::endl;
      return false;
    }
    img_width = imgHeader.width;
    img_height = imgHeader.height;
    // If principal point is not set we set the center of image
    if (ppx == -1)
      ppx = img_width/2;
    if (ppy == -1)
      ppy = img_height/2;

    return true;
  }

  /// Check that Kmatrix is a string like "f;0;ppx;0;f;ppy;0;0;1"
  /// With f,ppx,ppy as valid numerical value
  static bool checkIntrinsicStringValidity(const std::string & Kmatrix, double & focal, double & ppx, double & ppy)
  {
    std::vector<std::string> vec_str;
    stl::split(Kmatrix, ';', vec_str);
    if (vec_str.size() != 9)  {
      std::cerr << "\n Missing ';' character" << std::endl;
      return false;
    }
    // Check that all K matrix value are valid numbers
    for (size_t i = 0; i < vec_str.size(); ++i) {
      double readvalue = 0.0;
      std::stringstream ss;
      ss.str(vec_str[i]);
      if (! (ss >> readvalue) )  {
        std::cerr << "\n Used an invalid not a number character" << std::endl;
        return false;
      }
      if (i==0) focal = readvalue;
      if (i==2) ppx = readvalue;
      if (i==5) ppy = readvalue;
    }
    return true;
  }

  bool isValid()
  {
    if (focal !=-1 && ppx != -1 && ppy != -1 && img_width != 0 && img_height != 0)
    {
      return true;
    }
    return false;
  }
};

class Camera: public std::enable_shared_from_this<Camera>
{
private:
  IndexT id_;
  image::Image<unsigned char> img_mask_; // Mask for processing images
  image::Image<unsigned char> * ptr_img_mask_ = nullptr; // Mask for processing images
public:
  // Intrinsics
  std::shared_ptr<IntrinsicBase> ptr_intrinsic_;  // original intrinsics base
  std::shared_ptr<IntrinsicBase> ptr_intrinsic_undist_;  // intrinisc base without distortion
  IntrinsicBase * ptr_intrinsic_valid_ = nullptr; // pointer to valid intrinsic base (if we have calibrated camera we have undist base as well)
  bool b_calibrated_ = false;  // Flag if camera is calibrated

  float f_img_borders[4]; // min/max of image borders

public:
  Camera(const IndexT & id, const bool & b_calibrated ) : id_(id), b_calibrated_(b_calibrated), ptr_img_mask_(nullptr){};

  const IndexT & getCamId() const
  {
    return id_;
  }

  image::Image<unsigned char> * getMaskImagePtr()
  {
    return ptr_img_mask_;
  }

  void setMaskImage(image::Image<unsigned char> & mask)
  {
    img_mask_ = mask;
    ptr_img_mask_ = &img_mask_;
  }

  const bool & isCalibrated() const
  {
    return b_calibrated_;
  }
  bool isPointInImage(const Vec2 & pt) const
  { if (pt(0) < f_img_borders[0] || pt(0) > f_img_borders[2] || pt(1) < f_img_borders[1] || pt(1) > f_img_borders[3])
    {
      return false;
    }
    return true;
  }
  void computeImageBorders()
  {
    if (b_calibrated_ && ptr_intrinsic_->have_disto())
    {
      // If camera is calibrated and original intrinsic have distortion we compute the new borders by undistorting control points

      // Control points:
      // 0 -- 4 -- 1
      // |         |
      // 7         5
      // |         |
      // 3 -- 6 -- 2
      const size_t w = ptr_intrinsic_->w();
      const size_t h = ptr_intrinsic_->h();
      Mat2X pt2D_cp(2, 8);
      pt2D_cp << 0,w,w,0,w/2,w,w/2,0,     0, 0, h, h, 0, h/2, h, h/2;
      ptr_intrinsic_undist_->get_ud_pixel(pt2D_cp);
      // TODO: finish this
      std::cout << "Compute camera borders - DIST: NOT COMPLETED\n";
    }
    else
    {
      // Use image corners as limits as we will always do all the processing on distorted coordinates
      f_img_borders[0] = 0;
      f_img_borders[1] = 0;
      f_img_borders[2] = ptr_intrinsic_valid_->w();
      f_img_borders[3] = ptr_intrinsic_valid_->h();
    }
  }

  void printParameters()
  {
    std::cout << "VSSLAM [Camera] Camera: " << id_ << "\n"
              << "\tImage size: " << ptr_intrinsic_valid_->w() << ", " << ptr_intrinsic_valid_->h() << "\n"
              << "\tIntrinsics: (f/ppx/ppy/dist): ";
    for (auto & param :  ptr_intrinsic_valid_->getParams())
    {
                std::cout << param << " ";
    }
    std::cout << "\n"
              << "\tCalibrated: " << b_calibrated_ << "\n"
              << "\tImage boundaries: " << f_img_borders[0] << ", " << f_img_borders[1] << ", " << f_img_borders[2] << ", " << f_img_borders[3] << "\n";
    std::cout << "\tImage mask: " << (ptr_img_mask_==nullptr?"NO":"YES") << "\n";
  }

};

}
}
