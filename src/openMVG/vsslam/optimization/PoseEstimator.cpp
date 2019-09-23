// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include <openMVG/vsslam/optimization/PoseEstimator.hpp>

using namespace openMVG::robust;
using namespace openMVG::geometry;

namespace openMVG {
namespace vsslam {


struct ResectionSquaredResidualError {
  // Compute the residual of the projection distance(pt2D, Project(P,pt3D))
  // Return the squared error
  static double Error(const Mat34 & P, const Vec2 & pt2D, const Vec3 & pt3D){
    return (Project(P, pt3D) - pt2D).squaredNorm();
  }
};

double PoseEstimator::computeCosParallaxBetweenRays(const Vec3 & ray_1, const Vec3 & ray_2)
{
  const double mag = ray_1.norm() * ray_2.norm();
  const double dotAngle = ray_1.dot( ray_2 );
  return clamp( dotAngle / mag, -1.0 + 1.e-8, 1.0 - 1.e-8 );

}

bool PoseEstimator::computeH
(
  const Mat & x1,
  const Mat & x2,
  const std::pair<size_t,size_t> & size_img_1,
  const std::pair<size_t,size_t> & size_img_2,
  const double & f_max_thresh,
  Mat3 & H_21,  //x2 = H21 * x1
  std::pair<double, double> & ACRansacInfo,
  std::vector<uint32_t> & vec_inliers
)
{
  //-- Homography robust estimation
  using KernelType =
    robust::ACKernelAdaptor<
      openMVG::homography::kernel::FourPointSolver,
      openMVG::homography::kernel::AsymmetricError,
      UnnormalizerI,
      Mat3>;

  KernelType kernel(
    x1, size_img_1.first, size_img_1.second,
    x2, size_img_2.first, size_img_2.second,
    false); // configure as point to point error model.

  // ACRansacOut.first (threshold) ACRansacOut.second(NFA score)
  // Precision is squared distance to projection
  ACRansacInfo = ACRANSAC(kernel, vec_inliers, ACRANSAC_ITER, &H_21, f_max_thresh*f_max_thresh, false);

  if (vec_inliers.size() > KernelType::MINIMUM_SAMPLES *2.5)
  {
    return true;
  }
  vec_inliers.clear();
  return false;
}


bool PoseEstimator::computeE
(
  const Mat3 & K1,
  const Mat3 & K2,
  const Mat & x1,
  const Mat & x2,
  const std::pair<size_t,size_t> & size_img_1,
  const std::pair<size_t,size_t> & size_img_2,
  const double & f_max_thresh,
  Mat3 & E_21,
  std::pair<double, double> & ACRansacInfo,
  std::vector<uint32_t> & vec_inliers
)
{
  // Use the 5 point solver to estimate E
  using SolverType = openMVG::essential::kernel::FivePointKernel;
  // Define the AContrario adaptor
  using KernelType =
    ACKernelAdaptorEssential<
      SolverType,
      openMVG::fundamental::kernel::EpipolarDistanceError,
      Mat3>;

  KernelType kernel(
    x1, size_img_1.first, size_img_1.second,
    x2, size_img_2.first, size_img_2.second,
    K1, K2);

  // Robustly estimation of the Essential matrix and it's precision
  // ACRansacOut.first (threshold) ACRansacOut.second(NFA score)
  // Precision is squared distance to epipolar line
  ACRansacInfo = ACRANSAC(kernel, vec_inliers, ACRANSAC_ITER, &E_21, f_max_thresh*f_max_thresh, false);

  if (vec_inliers.size() > SolverType::MINIMUM_SAMPLES *2.5)
  {
    return true;
  }
  vec_inliers.clear();
  return false;
}


bool PoseEstimator::estimateRobustRelativePose_HE_Pinhole
(
    const Frame * frame_1,
    const Frame * frame_2,
    const openMVG::matching::IndMatches & vec_putative_matches_1_2_idx,
    Mat4 & T,
    std::vector<uint32_t> & vec_inliers,
    double & f_model_thresh,
    VSSLAM_Parameters * param
)
{
  if(param->verbose_level>1)
    std::cout<<"Pose: [Estimator] Try estimating H and E\n";

  // ----------------------------
  // Get camera info
  // ----------------------------
  const Pinhole_Intrinsic * cam_intrinsic_1 = dynamic_cast<const Pinhole_Intrinsic*>(frame_1->getCameraIntrinsics());
  const Pinhole_Intrinsic * cam_intrinsic_2 = dynamic_cast<const Pinhole_Intrinsic*>(frame_2->getCameraIntrinsics());


  // Try to estimate the H and F/E from mathces
  const size_t n_putative_matches = vec_putative_matches_1_2_idx.size();
  Mat2X pt2D_frame1(2, n_putative_matches);
  Mat2X pt2D_frame2(2, n_putative_matches);

  // ----------------------------
  // Copy data of matches
  // ----------------------------
  #ifdef OPENMVG_USE_OPENMP
  #pragma omp parallel for schedule(dynamic)
  #endif
  for (int m_i = 0; m_i < n_putative_matches; ++m_i)
  {
    matching::IndMatches::const_iterator m_iter = vec_putative_matches_1_2_idx.begin();
    std::advance(m_iter,m_i);
    pt2D_frame1.col(m_i) = frame_1->getFeaturePosition(m_iter->i_).cast<double>();
    pt2D_frame2.col(m_i) = frame_2->getFeaturePosition(m_iter->j_).cast<double>();
  }

  const std::pair<size_t,size_t> size_img_1(cam_intrinsic_1->w(), cam_intrinsic_1->h());
  const std::pair<size_t,size_t> size_img_2(cam_intrinsic_2->w(), cam_intrinsic_2->h());

  // ----------------------------
  // Estimate H: 2_H_1
  // ----------------------------
  Mat3 H_21;
  std::vector<uint32_t> vec_inliers_H;
  std::pair<double, double> infoACR_H; // first - threshold;  second - NFA score

  bool b_valid_H = computeH(pt2D_frame1,pt2D_frame2, size_img_1, size_img_2, param->init_track_max_model_thresh_px, H_21, infoACR_H, vec_inliers_H);

  // Check if enough inliers
  if (b_valid_H && vec_inliers_H.size() < param->init_track_min_matches)
  {
    b_valid_H = false;
  }

  if(param->verbose_level>1)
    std::cout<<"Pose: [Compute H] " << b_valid_H << " thresh: " << infoACR_H.first<<" NFA: "<<infoACR_H.second <<" inliers: "<<vec_inliers_H.size() <<"\n";


  // ----------------------------
  // Estimate  E: 2_E_1
  // ----------------------------
  const Mat & K_cam_1 = cam_intrinsic_1->K();
  const Mat & K_cam_2 = cam_intrinsic_2->K();

  Mat3 E_21;
  std::vector<uint32_t> vec_inliers_E;
  std::pair<double, double> infoACR_E; // first - threshold;  second - NFA score

  bool b_valid_E = computeE(K_cam_1, K_cam_2, pt2D_frame1, pt2D_frame2, size_img_1, size_img_2, param->init_track_max_model_thresh_px, E_21, infoACR_E, vec_inliers_E);
  // Check if enough inliers
  if (b_valid_E && vec_inliers_E.size() < param->init_track_min_matches)
  {
    b_valid_E = false;
  }

  if(param->verbose_level>1)
    std::cout<<"Pose: [Compute E] " << b_valid_E << " thresh: " << infoACR_E.first<<" NFA: "<<infoACR_E.second <<" inliers: "<<vec_inliers_E.size() <<"\n";

  // Check if either of models is ok
  if (!b_valid_H && !b_valid_E)
  {
    // Neither models were successful! - skip this frame
    if(param->verbose_level>1)
      std::cout<<"Pose: Unsuccessful pose estimation! No model available\n";
    return false;
  }


  // ----------------------------
  // Estimate pose from model
  // ----------------------------
  // Decide which model to use based on NFA (smaller is better)
  bool b_use_H = (infoACR_H.second < infoACR_E.second);
  // for now!!
  b_use_H = false;

  if (b_use_H)
  {
    std::cout<<"Pose: Model H selected! - NOT IMPLEMENTED\n";
    return false;
  }
  else
  {
    if(param->verbose_level>1)
      std::cout<<"Pose: Model E selected!\n";

    // Try to get the pose from Essential matrix
    Mat3 R_11 = Mat3::Identity(); Vec3 t_11 = Vec3::Zero(); double s_11 = 1.0;
    Mat3 R_21; Vec3 t_21; double s_21;


    // Estimate rotation and translation from essential matrix
    if (sfm::estimate_Rt_fromE(
      K_cam_1.inverse() * pt2D_frame1.colwise().homogeneous(),
      K_cam_2.inverse() * pt2D_frame2.colwise().homogeneous(),
      E_21,
      vec_inliers_E, &R_21, &t_21))
    {
      for (auto it_inlier = vec_inliers_E.begin(); it_inlier != vec_inliers_E.end();)
      {
        const size_t k = *it_inlier;
        const Vec2
          & x1_ = pt2D_frame1.col(k),
          & x2_ = pt2D_frame2.col(k);

        // Compute ray angle between points
        const Vec3 ray_1 = Vec3(R_11.transpose() * Vec3( K_cam_1.inverse() * Vec3( x1_( 0 ), x1_( 1 ), 1.0 ) )).normalized();
        const Vec3 ray_2 = Vec3(R_21.transpose() * Vec3( K_cam_2.inverse() * Vec3( x2_( 0 ), x2_( 1 ), 1.0 ) )).normalized();

        const double cosParallax = computeCosParallaxBetweenRays(ray_1,ray_2);

        // If angle is smaller than threshold delete it
        if (cosParallax > param->init_track_min_cos_parallax_pt)
        {
          //std::cout << "Inlier deleted (cos parallax): "<<cosParallax << " :: " << param->init_track_min_cos_parallax_pt << "\n";
          it_inlier = vec_inliers_E.erase(it_inlier);
        }
        else
        {
          ++it_inlier;
        }
      }

      if (vec_inliers_E.size() < param->init_track_min_matches)
      {
        if(param->verbose_level>1)
          std::cout<<"Pose: Unsuccessful pose estimation! Issuficuent # of inliers: " << vec_inliers_E.size() << "\n";

        return false;
      }


      vec_inliers = vec_inliers_E;
      f_model_thresh = infoACR_E.first;
      s_21 = 1.0;
      T = Mat4::Identity();
      T.block(0,0,3,3) = 1/s_21 * R_21.transpose();
      T.block(0,3,3,1) = - (1/s_21 * R_21.transpose())*t_21;

      if(param->verbose_level>1)
      {
        std::cout << "Relative pose init:\n"
                  << "R:\n" << R_21 << "\n"
                  << "t: " << t_21.transpose() << "\n"
                  << "s: " << s_21 << "\n\n";
      }

      return true;
    }
    else
    {
      if(param->verbose_level>1)
        std::cout<<"Pose: Unsuccessful pose estimation! Failed extracting Rts from E\n";

      return false;
    }
    return true;
  }
}


bool PoseEstimator::estimateRobustPose_Pinhole
(
  const Frame * frame,
  Hash_Map<MapLandmark* ,IndexT> & matches_landmarks_frame,
  Mat4 & T,
  std::vector<uint32_t> & vec_inliers,
  double & f_model_thresh,
  VSSLAM_Parameters * param
)
{
  const size_t n_putative_matches = matches_landmarks_frame.size();

  Mat pt2D_frame(2, n_putative_matches);
  Mat pt3D_frame(3, n_putative_matches);

  // ----------------------------
  // Copy data of matches
  // ----------------------------
  #ifdef OPENMVG_USE_OPENMP
  #pragma omp parallel for schedule(dynamic)
  #endif
  for (int map_landmark_i = 0; map_landmark_i < n_putative_matches; ++map_landmark_i)
  {
    Hash_Map<MapLandmark*,IndexT>::iterator map_landmark_it = matches_landmarks_frame.begin();
    std::advance(map_landmark_it,map_landmark_i);
    MapLandmark * map_landmark = map_landmark_it->first;

    // Project landmark to frame
    Vec2 pt_2D_frame_2_projected;
    frame->getProjectedPoint(map_landmark,pt_2D_frame_2_projected);


    pt2D_frame.col(map_landmark_i) = pt_2D_frame_2_projected.cast<double>();
    pt3D_frame.col(map_landmark_i) = (map_landmark->X_).cast<double>();
  }


/*
  // ----------------------------
  // EPnP
  // ----------------------------

  // Since K calibration matrix is known, compute only [R|t]
  using KernelType =
    openMVG::robust::ACKernelAdaptorResection_K<
    openMVG::euclidean_resection::kernel::EpnpSolver,
      ResectionSquaredResidualError,
      Mat34>;

  KernelType kernel(pt2D_frame, pt3D_frame, frame->getK());
  Mat34 P;
  const std::pair<double,double> ACRansacOut =
  openMVG::robust::ACRANSAC(kernel, vec_inliers,ACRANSAC_ITER, &P, param->relocalization_max_model_thresh_px*param->relocalization_max_model_thresh_px, true);

  // Update the upper bound precision of the model found by AC-RANSAC
  f_model_thresh = ACRansacOut.first;
  const bool bResection = (vec_inliers.size() > 2.5 * openMVG::euclidean_resection::kernel::EpnpSolver::MINIMUM_SAMPLES);
*/

  // ----------------------------
  // P3P
  // ----------------------------

  // Since K calibration matrix is known, compute only [R|t]

  using KernelType =
        openMVG::robust::ACKernelAdaptorResection_K<
        openMVG::euclidean_resection::P3PSolver,
          ResectionSquaredResidualError,
          Mat34>;

  KernelType kernel(pt2D_frame, pt3D_frame, frame->getK());
  Mat34 P;
  // Robust estimation of the Projection matrix and it's precision
  // Precision is squared distance to epipolar line
  const std::pair<double,double> ACRansacOut =
    openMVG::robust::ACRANSAC(kernel, vec_inliers,ACRANSAC_ITER, &P, param->relocalization_max_model_thresh_px*param->relocalization_max_model_thresh_px, true);

  // Update the upper bound precision of the model found by AC-RANSAC
  f_model_thresh = ACRansacOut.first*ACRansacOut.first;

  if(param->verbose_level>1)
    std::cout<<"P3P inliers: "<<vec_inliers.size()<<"\n";

  // Test if the mode support some points (more than those required for estimation)
  const bool bResection = (vec_inliers.size() > 2.5 * openMVG::euclidean_resection::P3PSolver::MINIMUM_SAMPLES);

  if (bResection)
  {
    if(param->verbose_level>1)
      std::cout<<"Pose: Successful P3P estimation! # inliers: "<<vec_inliers.size()<<"\n";

    Mat3 K_t;
    Mat3 R;
    Vec3 t;
    KRt_From_P(P, &K_t, &R, &t);

    T = Mat4::Identity();
    T.block(0,0,3,3) = R.transpose();
    T.block(0,3,3,1) = - R.transpose()*t;

    return true;
  }

  if(param->verbose_level>1)
    std::cout<<"Pose: Failed EPnP estimation! # inliers: "<<vec_inliers.size()<<"\n";
    
  return false;
}

// Get representation of a point (represented in frame_reference) in frame_new_reference
bool PoseEstimator::getRelativePointPosition
(
  const Vec3 & pt_3D_frame,
  const Frame * frame_reference,
  Vec3 & pt_3D_frame_new,
  const Frame * frame_new_reference
)
{
  if (frame_reference == frame_new_reference)
  {
    pt_3D_frame_new = pt_3D_frame;
    return true;
  }

  // Point is in global reference frame
  if (frame_reference == nullptr)
  {
    // p_c = inv(w_T_c) * p_w
    pt_3D_frame_new = Vec4(frame_new_reference->getTransformationMatrixInverse() * pt_3D_frame.homogeneous()).hnormalized();
    return true;
  }

  std::cout<<"Relative points ARE NOT IMPLEMENTED...YET!\n";
  return false;
}

bool PoseEstimator::computeFundamentalMatrix
(
  const Frame * frame_1,
  const Frame * frame_2,
  Mat3 & F_21
)
{
  // Transformation between frames (from frame_1 to frame_2)
  Mat4 T_12;
  if (!getRelativeCameraTransformation(frame_2,frame_1,T_12))
  {
    // Shouldnt happen!
    return false;
  }

  // Get Rt from T = [sR t]
  Mat4 T_21 = T_12.inverse(); //  camera transformations are inverse
  const double s = T_21.block(0,0,3,1).norm();
  const Mat3 sR = T_21.block(0,0,3,3);
  const Vec3 t = T_21.block(0,3,3,1);
  Mat3 tx;
  tx<<0,-t(2),t(1), t(2),0,-t(0), -t(1),t(0),0;

  // Compute essential matrix
  const Mat3 & K_inv_frame_1 = frame_1->getKinv();
  const Mat3 & K_inv_frame_2 = frame_2->getKinv();

  F_21 = K_inv_frame_2.transpose() * tx * sR * K_inv_frame_1;
  return true;
}


// Find transformation from frame_reference to frame: (frame)_T_(frame_reference)
bool PoseEstimator::getRelativeCameraTransformation
(
  const Frame * frame,
  const Frame * frame_reference,
  Mat4 & T
)
{
  if (frame == frame_reference)
  {
    T = Mat4::Identity();
    return true;
  }

  // Frame is already expressed in frame_reference
  if (frame and frame->getReferenceFrame() == frame_reference)
  {
    T = frame->getTransformationMatrix();
    return true;
  }

  if (frame->getReferenceFrame() == frame_reference->getReferenceFrame())
  {
    T = frame_reference->getTransformationMatrixInverse() * frame->getTransformationMatrix();
    return true;
  }

  std::cout<<"Relative frames ARE NOT IMPLEMENTED...YET!\n";
  return false;

}
}
}
