// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/cameras.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/geodesy/geodesy.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/types.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <utility>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;

/// Check that Kmatrix is a string like "f;0;ppx;0;f;ppy;0;0;1"
/// With f,ppx,ppy as valid numerical value
bool checkIntrinsicStringValidity(const std::string & Kmatrix, double & focal, double & ppx, double & ppy)
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

std::pair<bool, Vec3> checkGPS
(
  const std::string & filename,
  const int & GPS_to_XYZ_method = 0
)
{
  std::pair<bool, Vec3> val(false, Vec3::Zero());
  std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
  if (exifReader)
  {
    // Try to parse EXIF metada & check existence of EXIF data
    if ( exifReader->open( filename ) && exifReader->doesHaveExifInfo() )
    {
      // Check existence of GPS coordinates
      double latitude, longitude, altitude;
      if ( exifReader->GPSLatitude( &latitude ) &&
           exifReader->GPSLongitude( &longitude ) &&
           exifReader->GPSAltitude( &altitude ) )
      {
        // Add ECEF or UTM XYZ position to the GPS position array
        val.first = true;
        switch (GPS_to_XYZ_method)
        {
          case 1:
            val.second = lla_to_utm( latitude, longitude, altitude );
            break;
          case 0:
          default:
            val.second = lla_to_ecef( latitude, longitude, altitude );
            break;
        }
      }
    }
  }
  return val;
}


/// Check string of prior weights
std::pair<bool, Vec3> checkPriorWeightsString
(
  const std::string &sWeights
)
{
  std::pair<bool, Vec3> val(true, Vec3::Zero());
  std::vector<std::string> vec_str;
  stl::split(sWeights, ';', vec_str);
  if (vec_str.size() != 3)
  {
    std::cerr << "\n Missing ';' character in prior weights" << std::endl;
    val.first = false;
  }
  // Check that all weight values are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i)
  {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (! (ss >> readvalue) )  {
      std::cerr << "\n Used an invalid not a number character in local frame origin" << std::endl;
      val.first = false;
    }
    val.second[i] = readvalue;
  }
  return val;
}

// Split string in to fields
std::vector<std::string> parseDelimitedString(const std::string& str, const std::string& delim)
{
	std::vector<std::string> tokens;
	size_t prev = 0, pos = 0;
	do
	{
		pos = str.find(delim, prev);
		if (pos == std::string::npos) pos = str.length();
		std::string token = str.substr(prev, pos - prev);
		if (!token.empty()) tokens.push_back(token);
		prev = pos + delim.length();
	} while (pos < str.length() && prev < str.length());
	return tokens;
}

/// Read in landmarks.txt file
bool readLandmarksTextFile
(
	const std::string& sFilename,
	SfM_Data&          sfm_data
)
{
	// Open file
	std::ifstream theFile;
	try
	{
		theFile.open(sFilename.c_str());
	}
	catch ( ... )
	{
		std::cerr << "Unable to open landmark file for reading: " << sFilename << std::endl;
		return (false);
	}

	// Read in each landmark
	std::string sLine;
	std::string landmark_name;
	double x, y, z;
	int num_control_points;
	while (std::getline( theFile, sLine))
	{
		// Skip empty
		if (sLine.empty())
		{
			continue;
		}

		// Parse string of form: landmark_name,x,y,z,#control_points_to_follow
		auto tokens = parseDelimitedString(sLine, ",");
		if (tokens.size() < 5)
		{
			std::cerr << "Unrecognized format in landmarks file for line: " << sLine << std::endl;
			continue;
		}
		landmark_name = tokens[0];
		x = ::atof(tokens[1].c_str());
		y = ::atof(tokens[2].c_str());
		z = ::atof(tokens[3].c_str());
		num_control_points = ::atoi(tokens[4].c_str());
		if ((num_control_points < 0) || (num_control_points > sfm_data.GetViews().size()))
		{
			std::cerr << "Unexpected control point count of " << num_control_points << " for landmark " << landmark_name << std::endl;
			continue;
		}

		// Start landmark
		Landmark landmark;
		landmark.X = Vec3(x,y,z);

		// Read in each control point
		for (int i = 0; i < num_control_points; ++i)
		{
			// Read in the line string
			if (!std::getline(theFile, sLine))
			{
				std::cerr << "Unexpected end of file reading control points for landmark " << landmark_name << std::endl;
				break;
			}
			// Read in the pixel coordinates and image name
			auto tokens = parseDelimitedString(sLine, ",");
			if ( tokens.size() < 3)
			{
				std::cerr << "Unrecognized format in control point " << i << " for landmark " << landmark_name << std::endl;
				break;
			}
			std::string image_name = tokens[0];
			double pix_x = ::atof(tokens[1].c_str());
			double pix_y = ::atof(tokens[2].c_str());

			// Determine view ID from image filename. Should be only point on this view for the landmark.
			IndexT view_id = 0;
			bool view_id_valid = false;
			for (const auto& view : sfm_data.GetViews())
			{
				// Extract filename part of entire path
				const auto& view_full_filename = view.second->s_Img_path;
				auto last_slash_pos = view_full_filename.find_last_of("\\/");
				std::string view_base_filename;
				if (view_full_filename.npos == last_slash_pos)
				{
					view_base_filename = view_full_filename;
				}
				else
				{
					view_base_filename = view_full_filename.substr(last_slash_pos + 1);
				}
				
				// Remove extension
				auto last_period_pos = view_base_filename.find_last_of('.');
				if (view_base_filename.npos != last_period_pos)
				{
					view_base_filename = view_base_filename.substr(0, last_period_pos);
				}

				// Compare the strings
				if (0 == stricmp(view_base_filename.c_str(), image_name.c_str()))
				{
					view_id = view.first;
					view_id_valid = true;
					break;
				}
			}
			if (!view_id_valid)
			{
				std::cerr << "No view found associated with image filename " << image_name << " for control point " << i
					<< " for landmark " << landmark_name << ", ignoring." << std::endl;
				continue;
			}

			// Make sure we don't already have a control point from this image
			if (landmark.obs.end() != landmark.obs.find(view_id))
			{
				std::cerr << "Duplicate control point entries for view " << view_id << " (" << image_name << "), ignoring: " << sLine << std::endl;
				continue;
			}

			// Add control point to landmark
			Observation obs;
			obs.x = Vec2(pix_x, pix_y);
			obs.id_feat = view_id;
			landmark.obs[view_id] = obs;
		}		

		// Add landmark
		if (!landmark.obs.empty())
		{
			IndexT landmark_unique_id = (IndexT)sfm_data.control_points.size(); // unique ID just from current landmark count
			sfm_data.control_points[ landmark_unique_id ] = landmark;

			// Log what we added
			std::cout << "Added landmark " << landmark_unique_id << " at location (" << landmark.X.x() << ", "
				<< landmark.X.y() << ", " << landmark.X.z() << ") with the following " << landmark.obs.size() << " control points:" << std::endl;
			for (const auto& obs : landmark.obs) {
				std::cout << "--View " << obs.second.id_feat << "@ pixel coordinates (" << obs.second.x.x() << ", " << obs.second.x.y() << ")" << std::endl;
			}
		}

	}

	return (true);
}

//
// Create the description of an input image dataset for OpenMVG toolsuite
// - Export a SfM_Data file with View & Intrinsic data
//
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sImageDir,
    sfileDatabase = "",
    sOutputDir = "",
    sKmatrix;

  std::string sPriorWeights;
  std::pair<bool, Vec3> prior_w_info(false, Vec3(1.0,1.0,1.0));

  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;

  bool b_Group_camera_model = true;

  bool b_EXIF_Extended_Lookup = false;

  int i_GPS_XYZ_method = 0;

  double focal_pixels = -1.0;

  std::string sLandmarksFilename;

  cmd.add( make_option('i', sImageDir, "imageDirectory") );
  cmd.add( make_option('d', sfileDatabase, "sensorWidthDatabase") );
  cmd.add( make_option('o', sOutputDir, "outputDirectory") );
  cmd.add( make_option('f', focal_pixels, "focal") );
  cmd.add( make_option('k', sKmatrix, "intrinsics") );
  cmd.add( make_option('c', i_User_camera_model, "camera_model") );
  cmd.add( make_option('g', b_Group_camera_model, "group_camera_model") );
  cmd.add( make_switch('P', "use_pose_prior") );
  cmd.add( make_option('W', sPriorWeights, "prior_weights"));
  cmd.add( make_option('m', i_GPS_XYZ_method, "gps_to_xyz_method") );
  cmd.add( make_option('x', b_EXIF_Extended_Lookup, "exif_extended_lookup") );
  cmd.add(make_option('l', sLandmarksFilename, "landmarksFilename"));

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch (const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--imageDirectory]\n"
      << "[-d|--sensorWidthDatabase]\n"
      << "[-o|--outputDirectory]\n"
      << "[-f|--focal] (pixels)\n"
      << "[-k|--intrinsics] Kmatrix: \"f;0;ppx;0;f;ppy;0;0;1\"\n"
      << "[-c|--camera_model] Camera model type:\n"
      << "\t 1: Pinhole\n"
      << "\t 2: Pinhole radial 1\n"
      << "\t 3: Pinhole radial 3 (default)\n"
      << "\t 4: Pinhole brown 2\n"
      << "\t 5: Pinhole with a simple Fish-eye distortion\n"
      << "[-g|--group_camera_model]\n"
      << "\t 0-> each view have it's own camera intrinsic parameters,\n"
      << "\t 1-> (default) view can share some camera intrinsic parameters\n"
      << "\n"
      << "[-P|--use_pose_prior] Use pose prior if GPS EXIF pose is available"
      << "[-W|--prior_weigths] \"x;y;z;\" of weights for each dimension of the prior (default: 1.0)\n"
      << "[-m|--gps_to_xyz_method] XYZ Coordinate system:\n"
      << "\t 0: ECEF (default)\n"
      << "\t 1: UTM\n"
      << "[-x|--exif_extended_lookup] Allow partial map on camera model name in EXIF information"
	  << "[-l|--landmarksFilename] Landmarks Filename\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  std::cout << " You called : " << std::endl
	  << argv[0] << std::endl
	  << "--imageDirectory " << sImageDir << std::endl
	  << "--sensorWidthDatabase " << sfileDatabase << std::endl
	  << "--outputDirectory " << sOutputDir << std::endl
	  << "--focal " << focal_pixels << std::endl
	  << "--intrinsics " << sKmatrix << std::endl
	  << "--camera_model " << i_User_camera_model << std::endl
	  << "--group_camera_model " << b_Group_camera_model << std::endl
	  << "--exif_extended_lookup " << b_EXIF_Extended_Lookup << std::endl
	  << "--landmarksFilename " << sLandmarksFilename << std::endl;

  // Expected properties for each image
  double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1;

  const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);

  if ( !stlplus::folder_exists( sImageDir ) )
  {
    std::cerr << "\nThe input directory doesn't exist" << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutputDir.empty())
  {
    std::cerr << "\nInvalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if ( !stlplus::folder_exists( sOutputDir ) )
  {
    if ( !stlplus::folder_create( sOutputDir ))
    {
      std::cerr << "\nCannot create output directory" << std::endl;
      return EXIT_FAILURE;
    }
  }

  if (sKmatrix.size() > 0 &&
    !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy) )
  {
    std::cerr << "\nInvalid K matrix input" << std::endl;
    return EXIT_FAILURE;
  }

  if (sKmatrix.size() > 0 && focal_pixels != -1.0)
  {
    std::cerr << "\nCannot combine -f and -k options" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<Datasheet> vec_database;
  if (!sfileDatabase.empty())
  {
    if ( !parseDatabase( sfileDatabase, vec_database ) )
    {
      std::cerr
       << "\nInvalid input database: " << sfileDatabase
       << ", please specify a valid file." << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Check if prior weights are given
  if (cmd.used('P') && !sPriorWeights.empty())
  {
    prior_w_info = checkPriorWeightsString(sPriorWeights);
  }
  else if (cmd.used('P'))
  {
    prior_w_info.first = true;
  }

  std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );
  std::sort(vec_image.begin(), vec_image.end());

  // Configure an empty scene with Views and their corresponding cameras
  SfM_Data sfm_data;
  sfm_data.s_root_path = sImageDir; // Setup main image root_path
  Views & views = sfm_data.views;
  Intrinsics & intrinsics = sfm_data.intrinsics;
  Landmarks& landmarks = sfm_data.control_points;

  C_Progress_display my_progress_bar( vec_image.size(),
      std::cout, "\n- Image listing -\n" );
  std::ostringstream error_report_stream;
  double last_computed_focal = -1.0;
  for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
    iter_image != vec_image.end();
    ++iter_image, ++my_progress_bar )
  {
    // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
    width = height = ppx = ppy = focal = -1.0;

    const std::string sImageFilename = stlplus::create_filespec( sImageDir, *iter_image );
    const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

    // Test if the image format is supported:
    if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
    {
      error_report_stream
          << sImFilenamePart << ": Unkown image file format." << "\n";
      continue; // image cannot be opened
    }

    if (sImFilenamePart.find("mask.png") != std::string::npos
       || sImFilenamePart.find("_mask.png") != std::string::npos)
    {
      error_report_stream
          << sImFilenamePart << " is a mask image" << "\n";
      continue;
    }

    ImageHeader imgHeader;
    if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
      continue; // image cannot be read

    width = imgHeader.width;
    height = imgHeader.height;
    ppx = width / 2.0;
    ppy = height / 2.0;

    std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
    exifReader->open( sImageFilename );

    const bool bHaveValidExifMetadata =
      exifReader->doesHaveExifInfo()
      && !exifReader->getModel().empty();

    // Consider the case where the focal is provided manually
    if ( !bHaveValidExifMetadata || focal_pixels != -1)
    {
      if (sKmatrix.size() > 0) // Known user calibration K matrix
      {
        if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
          focal = -1.0;
      }
      else // User provided focal length value
        if (focal_pixels != -1 )
          focal = focal_pixels;
    }
    else // If image contains meta data
    {
      const std::string sCamModel = exifReader->getModel();

      // Handle case where focal length is equal to 0
      if (exifReader->getFocal() == 0.0f)
      {
        error_report_stream
          << stlplus::basename_part(sImageFilename) << ": Focal length is missing." << "\n";
        focal = -1.0;
      }
      else
      // Create the image entry in the list file
      {
        Datasheet datasheet;
        if ( getInfo( sCamModel, vec_database, datasheet, b_EXIF_Extended_Lookup))
        {
          // The camera model was found in the database so we can compute it's approximated focal length
          const double ccdw = datasheet.sensorSize_;
          focal = std::max ( width, height ) * exifReader->getFocal() / ccdw;

		  // Log what we computed if different than last
		  if (focal != last_computed_focal)
		  {
			  std::cout << "Computed new focal length pixels = " << focal << " for image <" << sImageFilename
				  << "> of size " << width << "x" << height << ", focal = " << exifReader->getFocal() << "mm, ccdw ="
				  << ccdw << "mm" << std::endl;
			  last_computed_focal = focal;
		  }
        }
        else
        {
          error_report_stream
            << stlplus::basename_part(sImageFilename)
            << "\" model \"" << sCamModel << "\" doesn't exist in the database" << "\n"
            << "Please consider add your camera model and sensor width in the database." << "\n";
        }
      }
    }

    // Build intrinsic parameter related to the view
    std::shared_ptr<IntrinsicBase> intrinsic;

    if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
    {
      // Create the desired camera type
      switch (e_User_camera_model)
      {
        case PINHOLE_CAMERA:
          intrinsic = std::make_shared<Pinhole_Intrinsic>
            (width, height, focal, ppx, ppy);
        break;
        case PINHOLE_CAMERA_RADIAL1:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
            (width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
        break;
        case PINHOLE_CAMERA_RADIAL3:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
        break;
        case PINHOLE_CAMERA_BROWN:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
        break;
        case PINHOLE_CAMERA_FISHEYE:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>
            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
        break;
        default:
          std::cerr << "Error: unknown camera model: " << (int) e_User_camera_model << std::endl;
          return EXIT_FAILURE;
      }
    }

    // Build the view corresponding to the image
    const std::pair<bool, Vec3> gps_info = checkGPS(sImageFilename, i_GPS_XYZ_method);
    if (gps_info.first && cmd.used('P'))
    {
      ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);

      // Add intrinsic related to the image (if any)
      if (intrinsic == nullptr)
      {
        //Since the view have invalid intrinsic data
        // (export the view, with an invalid intrinsic field value)
        v.id_intrinsic = UndefinedIndexT;
      }
      else
      {
        // Add the defined intrinsic to the sfm_container
        intrinsics[v.id_intrinsic] = intrinsic;
      }

      v.b_use_pose_center_ = true;
      v.pose_center_ = gps_info.second;
      // prior weights
      if (prior_w_info.first == true)
      {
        v.center_weight_ = prior_w_info.second;
      }

      // Add the view to the sfm_container
      views[v.id_view] = std::make_shared<ViewPriors>(v);
    }
    else
    {
      View v(*iter_image, views.size(), views.size(), views.size(), width, height);

      // Add intrinsic related to the image (if any)
      if (intrinsic == nullptr)
      {
        //Since the view have invalid intrinsic data
        // (export the view, with an invalid intrinsic field value)
        v.id_intrinsic = UndefinedIndexT;
      }
      else
      {
        // Add the defined intrinsic to the sfm_container
        intrinsics[v.id_intrinsic] = intrinsic;
      }

      // Add the view to the sfm_container
      views[v.id_view] = std::make_shared<View>(v);
    }
  }

  // Read in the landmarks
  if (!sLandmarksFilename.empty())
  {
	  if (!readLandmarksTextFile(sLandmarksFilename, sfm_data))
	  {
		  error_report_stream << "Error reading landmarks from file: " << sLandmarksFilename << std::endl;
	  }
  }

  // Display saved warning & error messages if any.
  if (!error_report_stream.str().empty())
  {
    std::cerr
      << "\nWarning & Error messages:" << std::endl
      << error_report_stream.str() << std::endl;
  }

  // Group camera that share common properties if desired (leads to more faster & stable BA).
  if (b_Group_camera_model)
  {
    GroupSharedIntrinsics(sfm_data);
  }

  // Store SfM_Data views & intrinsic data
  if (!Save(
    sfm_data,
    stlplus::create_filespec( sOutputDir, "sfm_data.json" ).c_str(),
    ESfM_Data(VIEWS|INTRINSICS|CONTROL_POINTS)))
  {
    return EXIT_FAILURE;
  }

  std::cout << std::endl
    << "SfMInit_ImageListing report:\n"
    << "listed #File(s): " << vec_image.size() << "\n"
    << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
    << "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size() << std::endl;

  return EXIT_SUCCESS;
}
