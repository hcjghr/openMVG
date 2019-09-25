// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <cstdlib>
#include <iostream>


#include "openMVG/image/image_io.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <openMVG/vsslam/system/Camera.hpp>
#include <openMVG/vsslam/system/slam_system.hpp>
#include <openMVG/vsslam/display/vsslam_display.hpp>

#include <openMVG/vsslam/tracking/Abstract_Tracker.hpp>
#include <openMVG/vsslam/tracking/Tracker_Features.hpp>
#include <openMVG/vsslam/features/Abstract_Feature_Extractor.hpp>
#include <openMVG/vsslam/features/Feat_Extractor_SIFT.hpp>
#include <openMVG/vsslam/features/Feat_Extractor_AKAZE_MLDB.hpp>
#include <openMVG/vsslam/features/Feat_Extractor_AKAZE_MSURF.hpp>
#include <openMVG/vsslam/features/Abstract_Feature_Matcher.hpp>
#include <openMVG/vsslam/features/Feat_Matcher_CascadeHashing.hpp>
#include <openMVG/vsslam/features/Feat_Matcher_Regions.hpp>

#include <openMVG/vsslam/mapping/Cartographer.hpp>

#ifndef SWINE_NOGL
#include "software/VSSLAM/CGlWindow.hpp"  // swine
#endif // !SWINE_NOGL

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::image;
using namespace openMVG::vsslam;

VSSLAM_Display display_data;
VSSLAM_Time_Stats time_data;

int n_dummy_param = 0;

int main(int argc, char **argv)
{
  using namespace std;
  std::cout << "VISUAL S-SLAM -- Tracking demo --" << std::endl;

  CmdLine cmd;

  std::string sImaDirectory = "";
  std::string sOutDir = "";
  std::string sImaMask = "";
  unsigned int uTracker = 0;

  std::string config_file_path="";

  int verbose_level = 0;  // 0 - none; 1 - stats; 2 - all
  bool b_prompt_wait = false;
  bool b_ceres_global_BA = false;

  // Camera data
  std::string sKmatrix;
  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;

  cmd.add( make_option('i', sImaDirectory, "imadir") );
  cmd.add( make_option('o', sOutDir, "out_dir") );
  cmd.add( make_option('m', sImaMask, "imamask") );
  cmd.add( make_option('s', config_file_path, "config_file_path") );
  cmd.add( make_option('t', uTracker, "tracker") );
  cmd.add( make_option('k', sKmatrix, "intrinsics") );
  cmd.add( make_option('c', i_User_camera_model, "camera_model") );
  cmd.add( make_option('v', verbose_level, "verbose_level") );
  cmd.add( make_switch('w', "prompt_wait") );
  cmd.add( make_switch('C', "ceres_global_BA") );

  try {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--imadir image path] \n"
    << "[-o|--out_dir output path] \n"
    << "[-m|--mask image] \n"
    << "[-s|--config_file_path config file] \n"
    << "[-t|--tracker Used tracking interface] \n"
    << "\t 0 (default) description based Tracking -> Fast detector + Dipole descriptor\n"
#if defined HAVE_OPENCV
    << "\t 1 image based Tracking -> use OpenCV Pyramidal KLT Tracking\n"
#endif
    << "-v|--verbose_level] \n"
    << "\t 0: None (default)\n"
    << "\t 1: Stats\n"
    << "\t 2: All\n"
    << "[-k|--intrinsics] Kmatrix: \"f;0;ppx;0;f;ppy;0;0;1\"\n"
    << "[-c|--camera_model] Camera model type:\n"
    << "\t 1: Pinhole\n"
    << "\t 2: Pinhole radial 1\n"
    << "\t 3: Pinhole radial 3 (default)\n"
    << "\t 4: Pinhole brown 2\n"
    << "\t 5: Pinhole with a simple Fish-eye distortion\n"
    << "[-w|--prompt_wait] wait for key after frame\n"
    << "[-C|--ceres_global_BA] Use Ceres as global BA\n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

   std::cout << " You called : " <<std::endl
            << argv[0] << std::endl
            << "--imageDirectory " << sImaDirectory << std::endl
            << "--imageMask " << sImaMask << std::endl
            << "--config_file_path " << config_file_path << std::endl
            << "--intrinsics " << sKmatrix << std::endl
            << "--camera_model " << i_User_camera_model << std::endl;

  if (sImaDirectory.empty() || !stlplus::is_folder(sImaDirectory))
  {
    std::cerr << "\nVSSLAM: [Main] Invalid input directory" << std::endl;
    return EXIT_FAILURE;
  }

  b_prompt_wait = (cmd.used('w')?true:false);
  b_ceres_global_BA = (cmd.used('C')?true:false);

  // ----------------------------------
  // Image management
  // ----------------------------------
  image::Image<unsigned char> currentImage;
  // Load images from folder
  std::vector<std::string> vec_image = stlplus::folder_files(sImaDirectory);
  // clean invalid image file
  {
    std::vector<std::string> vec_image_;
    for (size_t i = 0; i < vec_image.size(); ++i)
    {
      if(vec_image[i].find("mask.png") != std::string::npos
         || vec_image[i].find("_mask.png") != std::string::npos)
      {
        std::cout
            << vec_image[i] << " is a mask image" << "\n";
        continue;
      }

      if (openMVG::image::GetFormat(vec_image[i].c_str()) != openMVG::image::Unknown)
        vec_image_.push_back(vec_image[i]);
    }
    vec_image_.swap(vec_image);
  }
  std::sort(vec_image.begin(), vec_image.end());

  // ----------------------------------
  // SLAM system initialization
  // ----------------------------------
  std::cout<<"\nVSSLAM: [Main] Start\n";

  // initialize params
  std::shared_ptr<VSSLAM_Parameters> params_system = std::make_shared<VSSLAM_Parameters>();
  if(!config_file_path.empty())
  {
    std::cout<<"\nVSSLAM: [Main] Load config file\n";
    params_system->LoadConfigFile(config_file_path);
  }
  SLAM_System slam_system(params_system);

  // Output settings
  if (sOutDir.empty() || !stlplus::is_folder(sOutDir))
  {
    slam_system.disableOutput();
  }
  else
  {
    slam_system.setOutputDirectory(sOutDir);
    slam_system.setIntermediateResultOutput(true);
  }

  slam_system.prepareStatsFile();



  // Create cartographer
  MAP_FRAME_TYPE map_frame_type = MAP_FRAME_TYPE::GLOBAL;
  MAP_LANDMARK_TYPE map_landmark_type = MAP_LANDMARK_TYPE::GLOBAL_EUCLIDEAN;

  MAP_OPTIMIZATION_TYPE global_BA_type;

  if (b_ceres_global_BA)
    global_BA_type = MAP_OPTIMIZATION_TYPE::CERES;
  else
    global_BA_type = MAP_OPTIMIZATION_TYPE::SLAMPP;

  MAP_OPTIMIZATION_TYPE local_BA_type = MAP_OPTIMIZATION_TYPE::CERES;

  {
    std::unique_ptr<Abstract_Tracker> ptr_tracker;
    std::unique_ptr<Abstract_Feature_Extractor> ptr_feat_extractor;
    std::unique_ptr<Abstract_Feature_Matcher> ptr_feat_matcher;

    switch (uTracker)
    {
      case 0:
        ptr_feat_extractor.reset(new Feat_Extractor_SIFT(params_system, features::NORMAL_PRESET));
        ptr_feat_matcher.reset(new Feat_Matcher_CascadeHashing(params_system, ptr_feat_extractor.get()));
        display_data.b_enable_display = 0;

        break;
      case 1:
        //ptr_feat_extractor.reset(new Feat_Extractor_SIFT(params_system, features::HIGH_PRESET));
        ptr_feat_extractor.reset(new Feat_Extractor_SIFT(params_system, params_system->feat_extract_sift_thresh));
        ptr_feat_matcher.reset(new Feat_Matcher_CascadeHashing(params_system, ptr_feat_extractor.get()));
        display_data.b_enable_display = 0;

        break;
      case 2:
        ptr_feat_extractor.reset(new Feat_Extractor_AKAZE_MSURF(params_system, features::NORMAL_PRESET));
        ptr_feat_matcher.reset(new Feat_Matcher_CascadeHashing(params_system, ptr_feat_extractor.get()));
        display_data.b_enable_display = 0;
        break;
      case 3:
        //ptr_feat_extractor.reset(new Feat_Extractor_AKAZE_MSURF(params_system, features::HIGH_PRESET));
        ptr_feat_extractor.reset(new Feat_Extractor_AKAZE_MSURF(params_system, params_system->feat_extract_akaze_thresh));
        ptr_feat_matcher.reset(new Feat_Matcher_CascadeHashing(params_system, ptr_feat_extractor.get()));
        display_data.b_enable_display = 0;
        break;
      default:
        std::cerr << "VSSLAM: [Main] Unknow tracking method" << std::endl;
        return EXIT_FAILURE;
    }


    if (!slam_system.createCartographer(map_frame_type,map_landmark_type,global_BA_type,local_BA_type ))
    {
      std::cerr << "VSSLAM: [Main] Cannot instantiate the cartographer" << std::endl;
      return EXIT_FAILURE;
    }

    ptr_tracker.reset(new Tracker_Features(params_system));
    if (!ptr_tracker)
    {
      std::cerr << "VSSLAM: [Main] Cannot instantiate the tracking interface" << std::endl;
      return EXIT_FAILURE;
    }

    slam_system.setFeatureExtractor(ptr_feat_extractor);
    slam_system.setFeatureMatcher(ptr_feat_matcher);
    slam_system.setTracker(ptr_tracker);

    // Set verbose level
    slam_system.setVerboseLevel(verbose_level);

  }



  // ----------------------------------
  // Create camera
  // ----------------------------------
  IndexT id_cam_0;
  {
    // Check and set camera parameters
    CameraParameters params_cam_0;
    params_cam_0.camera_model = EINTRINSIC(i_User_camera_model);

    if (sKmatrix.empty() ||
      !CameraParameters::checkIntrinsicStringValidity(sKmatrix, params_cam_0.focal, params_cam_0.ppx, params_cam_0.ppy) )
    {
      std::cerr << "\nVSSLAM: [Main] Error: Invalid K matrix input" << std::endl;
      return EXIT_FAILURE;
    }

    if (!params_cam_0.readImageSettings(stlplus::create_filespec( sImaDirectory, vec_image[0] )))
    {
      std::cerr << "\nVSSLAM: [Main] Error: reading image header file" << std::endl;
      return EXIT_FAILURE;
    }

    // Create camera model
    if (params_cam_0.camera_model!=EINTRINSIC::PINHOLE_CAMERA)
    {
      std::cerr << "VSSLAM: [Main] Error: Camera type: " << params_cam_0.camera_model << " NOT implemented" << std::endl;
      return EXIT_FAILURE;
    }

    if (params_cam_0.isValid())
    {
      params_cam_0.b_calibrated = true;
      id_cam_0 = slam_system.createCamera(params_cam_0);
    }
    else
    {
      std::cerr << "VSSLAM: [Main] Error: invalid camera parameters"<< std::endl;
      return EXIT_FAILURE;
    }


    // Update thresholds with camera size
    Camera * cam_0 = slam_system.getCameraPtr(id_cam_0);
    params_system->updateThresholdsWithImageSize(cam_0->getImageWidth(), cam_0->getImageHeight());

    
  }

std::cout << params_system->init_track_min_cos_parallax_pt << "\n";
  

  // ----------------------------------
  // Load mask images
  // ----------------------------------
  {
    image::Image<unsigned char> mask_cam_0;
    if (!sImaMask.empty() && stlplus::file_exists(sImaMask))
    {
      if (image::GetFormat(sImaMask.c_str()) == image::Unknown)
        std::cout << "VSSLAM: [Main] Mask image path is invalid! Not using mask image" << std::endl;
      else
      {
        if (!(image::ReadImage( sImaMask.c_str(), &mask_cam_0)))
        {
          std::cout << "VSSLAM: [Main] Mask image is invalid! Not using mask image" << std::endl;
        }
        else
        {
          slam_system.addMaskImageToCamera(id_cam_0,mask_cam_0);
          std::cout << "VSSLAM: [Main] Camera: " << id_cam_0 << " using mask image: "<<sImaMask<<"\n" << std::endl;
        }
      }
    }
  }
  slam_system.printCameraParameters(id_cam_0);

  if(!slam_system.isReady())
  {
    std::cerr << "VSSLAM: [Main] Error: SLAM not correctly initialized\n";
    return EXIT_FAILURE;
  }

#ifndef SWINE_NOGL

  // ----------------------------------
  // Graphics
  // ----------------------------------
  if ( !glfwInit() )
  {
    return EXIT_FAILURE;
  }

  CGlWindow window;
  GLuint text2D;
#endif // !SWINE_NOGL




  // ----------------------------------
  // Frame-by-Frame processing
  // ----------------------------------
  IndexT id_frame = 0;
  IndexT id_cam = 0;
  double timestamp_frame = 0;

  // Traverse image folder
  for (std::vector<std::string>::const_iterator iterFile = vec_image.begin();
      iterFile != vec_image.end(); ++iterFile, ++id_frame, ++timestamp_frame)
  {
std::cout << "NEXT FRAME" << std::endl;
    const std::string sImageFilename = stlplus::create_filespec( sImaDirectory, *iterFile );
    if (openMVG::image::ReadImage( sImageFilename.c_str(), &currentImage))
    {
      slam_system.nextFrame(currentImage, id_frame, id_cam,timestamp_frame);

     
        std::cout << "Display 00" << std::endl;
#ifndef SWINE_NOGL
        std::cout << "Display 0" << std::endl;
      if (window._height < 0)
      {
        // no window created yet, initialize it with the first frame

        const double aspect_ratio = currentImage.Width()/(double)currentImage.Height();
        window.Init(1280, 1280/aspect_ratio, "VisualOdometry--TrackingViewer");
        glGenTextures(1,&text2D);             //allocate the memory for texture
        glBindTexture(GL_TEXTURE_2D, text2D); //Binding the texture
        glEnable(GL_TEXTURE_2D);              //Enable texture
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
          currentImage.Width(), currentImage.Height(), 0,
          GL_LUMINANCE, GL_UNSIGNED_BYTE, currentImage.data());
      }

      if (display_data.b_enable_display)
      {
        // Display steps
        display_data.displaySteps(window,text2D, currentImage,slam_system.getCurrentFramePtr(),2);
        display_data.resetSteps();
      }
      else
      {
        std::cout << "Display 1" << std::endl;
        // Set title
        std::stringstream ss;
        ss << "Frame: " << " " << slam_system.getCurrentFramePtr()->getFrameId() << " # global frames/landmarks: " << time_data.global_frames << "/" << time_data.global_landmarks;
        window.setTitle(ss.str());

        std::cout << "Display 2" << std::endl;
        display_data.displayImage(window,text2D, currentImage);
        std::cout << "Display 3" << std::endl;
        // Detected features yellow
        display_data.displayDetectedFeatures(slam_system.getCurrentFramePtr());

        std::cout << "Display 4" << std::endl;
        display_data.displayLocalMap(slam_system.getCurrentFramePtr());

        std::cout << "Display 5" << std::endl;
        if (slam_system.isMapInitialized())    
          display_data.displayHistoryTracks(slam_system.getCurrentFramePtr());

        std::cout << "Display 6" << std::endl;
        display_data.displayFeaturesByAssociation(slam_system.getCurrentFramePtr());
        std::cout << "Display 7" << std::endl;
        display_data.displayFrameActivityIndicator(window,slam_system.getCurrentFramePtr());
        std::cout << "Display 8" << std::endl;

      }

      glFinish();
      glFlush();
      window.Swap(); // Swap openGL buffer
#endif // !SWINE_NOGL

      if (b_prompt_wait)
      {
        std::cout<<"VSSLAM: [Main] Press ENTER to continue....."<<std::endl<<std::endl;
        std::cin.ignore(1);
      }
    }
  }





  return 0;
}
