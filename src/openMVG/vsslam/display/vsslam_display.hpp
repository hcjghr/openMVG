// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Klemen Istenic

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>
#include <stdlib.h>
#include <unistd.h>
#include "openMVG/types.hpp"
#include "openMVG/matching/indMatchDecoratorXY.hpp"

#include <openMVG/vsslam/vsslam_data.hpp>
#include <openMVG/vsslam/system/Frame.hpp>
#include <openMVG/vsslam/mapping/MapLandmark.hpp>
#include <openMVG/vsslam/optimization/PoseEstimator.hpp>

#ifndef SWINE_NOGL
#include "software/VSSLAM/CGlWindow.hpp" // swine
#endif // !SWINE_NOGL

namespace openMVG {
namespace vsslam {

/// Define struct for all parameters

struct VSSLAM_Display
{
  int verbose_level = 0;

  bool b_enable_display = false;

  std::vector<size_t> display_iter;
  std::vector<std::string> display_text;
  std::vector<std::vector<Vec2> > display_pt2d_A;
  std::vector<std::vector<Vec2> > display_pt2d_B;
  std::vector<std::vector<Vec2> > display_pt2d_C;
  std::vector<std::vector<float> > display_size_A;
  std::vector<std::vector<float> > display_size_B;
  std::vector<std::vector<float> > display_size_C;

  size_t n_steps_in_bundle = 0;

  std::vector<MapLandmark *> display_local_map;




  void DrawCircle(float cx, float cy, float r)
  {
#ifndef SWINE_NOGL
      int num_segments = 20;
      glBegin(GL_LINE_LOOP);
      for(int ii = 0; ii < num_segments; ii++)
      {
          float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle

          float x = r * cosf(theta);//calculate the x component
          float y = r * sinf(theta);//calculate the y component

          glVertex2f(x + cx, y + cy);//output vertex

      }
      glEnd(); // swine
#endif // !SWINE_NOGL
  }

  void finishDisplayBundle()
  {
    display_iter.push_back(n_steps_in_bundle);
    n_steps_in_bundle = 0;
  }
  void addDisplayStep(std::string text, Frame * frame_A, Frame * frame_B, matching::IndMatches & vec_putative_matches_A_B_idx)
  {
    std::vector<Vec2> d_A; std::vector<Vec2> d_B;std::vector<Vec2> d_C; std::vector<float> d_sA;std::vector<float> d_sB;std::vector<float> d_sC;
    for (auto mp : vec_putative_matches_A_B_idx)
    {
      d_A.push_back(frame_A->getFeaturePosition(mp.i_));
      d_B.push_back(frame_B->getFeaturePosition(mp.j_));
      d_sA.push_back(frame_A->getFeatureScale(mp.i_));
      d_sB.push_back(frame_B->getFeatureScale(mp.j_));
    }
    display_pt2d_A.push_back(d_A);
    display_pt2d_B.push_back(d_B);
    display_pt2d_C.push_back(d_C);
    display_size_A.push_back(d_sA);
    display_size_B.push_back(d_sB);
    display_size_C.push_back(d_sC);
    display_text.push_back(text);
    n_steps_in_bundle++;
  }
  void addDisplayStep(std::string text, Frame * frame_A, Frame * frame_B, Hash_Map<IndexT,IndexT> & vec_putative_matches_A_B_idx)
    {
      std::vector<Vec2> d_A; std::vector<Vec2> d_B;std::vector<Vec2> d_C; std::vector<float> d_sA;std::vector<float> d_sB;std::vector<float> d_sC;
      for (auto mp : vec_putative_matches_A_B_idx)
      {
        d_A.push_back(frame_A->getFeaturePosition(mp.first));
        d_B.push_back(frame_B->getFeaturePosition(mp.second));
        d_sA.push_back(frame_A->getFeatureScale(mp.first));
        d_sB.push_back(frame_B->getFeatureScale(mp.second));
      }
      display_pt2d_A.push_back(d_A);
      display_pt2d_B.push_back(d_B);
      display_pt2d_C.push_back(d_C);
      display_size_A.push_back(d_sA);
      display_size_B.push_back(d_sB);
      display_size_C.push_back(d_sC);
      display_text.push_back(text);
      n_steps_in_bundle++;
    }
  void addDisplayStep(std::string text, Frame * frame_A, NewMapLandmarks & vec_new_landmarks)
  {
    std::vector<Vec2> d_A; std::vector<Vec2> d_B;std::vector<Vec2> d_C; std::vector<float> d_sA;std::vector<float> d_sB;std::vector<float> d_sC;
    for ( size_t k_i = 0; k_i < vec_new_landmarks.size(); ++k_i)
    {
      std::unique_ptr<MapLandmark> & landmark = vec_new_landmarks[k_i];

      // Get projection of 3D point on current frame
      Vec2 pt_2D_frame_projected;
      if (!frame_A->getProjectedPoint(landmark.get(),pt_2D_frame_projected))
        continue;


      LandmarkObservations l_obs = landmark->getObservations();
      for (auto m_o_pair : l_obs)
      {
        MapObservation & m_o = m_o_pair.second;

        d_B.push_back(m_o.frame_ptr->getFeaturePosition(m_o.feat_id));
        d_C.push_back(pt_2D_frame_projected);
        d_sC.push_back(4);
      }
    }
    display_pt2d_A.push_back(d_A);
    display_pt2d_B.push_back(d_B);
    display_pt2d_C.push_back(d_C);
    display_size_A.push_back(d_sA);
    display_size_B.push_back(d_sB);
    display_size_C.push_back(d_sC);
    display_text.push_back(text);
    n_steps_in_bundle++;
  }


  void addDisplayStep(std::string text, Frame * frame_B, Hash_Map<MapLandmark *,IndexT> & map_matches, float area_size)
  {
    std::vector<Vec2> d_A; std::vector<Vec2> d_B;std::vector<Vec2> d_C; std::vector<float> d_sA;std::vector<float> d_sB;std::vector<float> d_sC;

    for (auto  match_it : map_matches)
    {
      MapLandmark * landmark_A = match_it.first;

      if (!landmark_A)
        continue;

      // Project to current frame
      Vec2 pt_2D_frame_B_projected;
      if (!frame_B->getProjectedPoint(landmark_A,pt_2D_frame_B_projected))
        continue;

      // Projection of point in prev frame
      d_C.push_back(pt_2D_frame_B_projected);
      d_sC.push_back(area_size);

      d_B.push_back(frame_B->getFeaturePosition(match_it.second));

    }
    display_pt2d_A.push_back(d_A);
    display_pt2d_B.push_back(d_B);
    display_pt2d_C.push_back(d_C);
    display_size_A.push_back(d_sA);
    display_size_B.push_back(d_sB);
    display_size_C.push_back(d_sC);
    display_text.push_back(text);
    n_steps_in_bundle++;
  }

  void addDisplayStep(std::string text, Frame * frame_A, Frame * frame_B, Hash_Map<MapLandmark *,IndexT> & map_matches, float area_size)
  {
    std::vector<Vec2> d_A; std::vector<Vec2> d_B;std::vector<Vec2> d_C; std::vector<float> d_sA;std::vector<float> d_sB;std::vector<float> d_sC;

    for (IndexT a_i = 0; a_i < frame_A->getNumberOfFeatures(); ++a_i)
    {
      MapLandmark * landmark_A = frame_A->getLandmark(a_i);

      if (!landmark_A)
        continue;

      // Project to current frame
      Vec2 pt_2D_frame_B_projected;
      if (!frame_B->getProjectedPoint(landmark_A,pt_2D_frame_B_projected))
        continue;

      // Projection of point in prev frame
      d_C.push_back(pt_2D_frame_B_projected);
      d_sC.push_back(area_size);

      // we matched with feature in current frame
      if (map_matches.find(landmark_A)!=map_matches.end())
      {
        d_B.push_back(frame_B->getFeaturePosition(map_matches.find(landmark_A)->second));
      }
      else
      {
        d_B.push_back(Vec2(-1,-1));
      }
    }
    display_pt2d_A.push_back(d_A);
    display_pt2d_B.push_back(d_B);
    display_pt2d_C.push_back(d_C);
    display_size_A.push_back(d_sA);
    display_size_B.push_back(d_sB);
    display_size_C.push_back(d_sC);
    display_text.push_back(text);
    n_steps_in_bundle++;
  }

  void addDisplayStep(std::string text, Frame * frame_B, Hash_Map<MapLandmark *,IndexT> & map_matches_old, Hash_Map<MapLandmark *,IndexT> & map_matches, float area_size)
  {
    std::vector<Vec2> d_A; std::vector<Vec2> d_B;std::vector<Vec2> d_C; std::vector<float> d_sA;std::vector<float> d_sB;std::vector<float> d_sC;

    for (auto match : map_matches_old)
    {
      MapLandmark * landmark_A = match.first;

      // Project to current frame
      Vec2 pt_2D_frame_B_projected;
      if (!frame_B->getProjectedPoint(landmark_A,pt_2D_frame_B_projected))
        continue;

      // Projection of point in prev frame
      d_C.push_back(pt_2D_frame_B_projected);
      d_sC.push_back(area_size);

      // we match survived outlier rejection
      if (map_matches.find(landmark_A)!=map_matches.end())
      {
        d_B.push_back(frame_B->getFeaturePosition(map_matches.find(landmark_A)->second));
        d_A.push_back(Vec2(-1,-1));
      }
      else
      {
        d_B.push_back(Vec2(-1,-1));
        d_A.push_back(frame_B->getFeaturePosition(map_matches.find(landmark_A)->second));
      }
    }

    display_pt2d_A.push_back(d_A);
    display_pt2d_B.push_back(d_B);
    display_pt2d_C.push_back(d_C);
    display_size_A.push_back(d_sA);
    display_size_B.push_back(d_sB);
    display_size_C.push_back(d_sC);
    display_text.push_back(text);
    n_steps_in_bundle++;
  }

#ifndef SWINE_NOGL
  void displayImage(CGlWindow & window, GLuint & text2D,
      image::Image<unsigned char> &currentImage)
  {
    glBindTexture(GL_TEXTURE_2D, text2D); //Binding the texture
    glEnable(GL_TEXTURE_2D);              //Enable texture
    //-- Update the openGL texture with the current frame pixel values
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
     currentImage.Width(), currentImage.Height(),
    GL_LUMINANCE, GL_UNSIGNED_BYTE,
    currentImage.data());

    //-- Draw the current image
    window.SetOrtho(currentImage.Width(), currentImage.Height());
    window.DrawFullScreenTexQuad(currentImage.Width(), currentImage.Height());
    glDisable(GL_TEXTURE_2D);

    // Clear the depth buffer so the drawn image become the background
    glClear(GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
  }

  void displayDetectedFeatures(Frame * frame)
  {
    glPointSize(2.0f);
    glColor3f(1.f, 1.f, 0.f);
    glBegin(GL_POINTS);

    for (size_t feat_i = 0; feat_i<frame->getNumberOfFeatures(); ++feat_i)
    {
      Vec2 pt = frame->getFeaturePosition(feat_i);
      glVertex2f(pt(0),pt(1));
    }
    glEnd();  
  }


  void displayFrameActivityIndicator(CGlWindow & window, Frame * frame)
  {
    if (verbose_level>1)
      std::cout<<"Activity frame: "<<frame->getFrameId()<<":: Global: "<<frame->isActive()<<"\n";

    // Green circle if active and red if inactive
    if (frame->isActive())
    {
      glColor3f(0.0,1.0,0.0);
    }
    else
    {
      glColor3f(0.0,0.0,1.0);
    }

    // Draw point
    glPointSize(25.0f);
    glBegin(GL_POINTS);
    glVertex2f(50,50);
    glEnd();
  }


  void displayFeaturesByAssociation(Frame * frame)
  {
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (IndexT feat_i = 0; feat_i<frame->getNumberOfFeatures(); ++feat_i)
    {
      MapLandmark * map_landmark = frame->getLandmark(feat_i);
      if (!map_landmark)
        continue;


      Vec3 pt_color;
      // check if frame is in global frame
      switch (map_landmark->association_type_)
      {
      case 0: // Unknown - white
        pt_color = Vec3(0.0,0.0,0.0);
        break;
      case 1: // Initialization - red
        pt_color = Vec3(1.0,0.0,0.0);
        break;
      case 2: // Motion - green
        pt_color = Vec3(0.0,0.6,0.0);
        break;
      case 3: // Reference - yellow
        pt_color = Vec3(1.0,1.0,0.0);
        break;
      case 5: // Local Map - violet
        pt_color = Vec3(0.5,0.0,1.0);
        break;
      case 6: // New triangulations - 3 or more observations -  blue
        if (map_landmark->isActive())
          pt_color = Vec3(0.0,0.0,1.0);
        else  // New triangulations - 2 observations - red
          pt_color = Vec3(1.0,0.0,0.0);
        break;
      }
      // Draw point
      glColor3f(pt_color(0),pt_color(1),pt_color(2));
      Vec2 pt = frame->getFeaturePosition(feat_i);
      glVertex2f(pt(0),pt(1));
    }
    glEnd();
  }

  void displayHistoryTracks(Frame * frame)
  {
    if (verbose_level>1)
      std::cout<<"History tracks: "<<frame->getFrameId()<<":: "<<frame->isActive()<<"\n";
    
    for (IndexT feat_i = 0; feat_i<frame->getNumberOfFeatures(); ++feat_i)
    {
      MapLandmark * map_landmark = frame->getLandmark(feat_i);
      if (!map_landmark)
        continue;

      // Plot historical line of the landmark
      glPointSize(2.0f);
      glColor3f(1.f, 0.f, 0.f);
      glBegin(GL_LINE_STRIP);

      for (auto & obs_it : map_landmark->getObservations())
      {
          Vec2 pt = obs_it.second.frame_ptr->getFeaturePosition(obs_it.second.feat_id);
          glVertex2f(pt(0),pt(1));
      }
      
      // Link to current point
      glColor3f(1.f, 1.f, 0.f);
      Vec2 pt = frame->getFeaturePosition(feat_i);
      glVertex2f(pt(0),pt(1));

      glEnd();
    }
  }

  void displayLocalMap(Frame * frame)
  {
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for (IndexT feat_i = 0; feat_i<display_local_map.size(); ++feat_i)
    {
      // Get Landmark data
      MapLandmark * map_landmark = display_local_map[feat_i];
      if (!map_landmark)
        continue;

      // Blue if its local frame
      if (map_landmark->isActive())
      {
        glColor3f(1.f, 0.65f, 0.f);
      }
      else
      {
        glColor3f(0.f, 0.f, 1.f);
      }

      // Project point to frame coordinate system
      Vec2 pt_2D_frame;
      if (frame->getProjectedPoint(map_landmark,pt_2D_frame))
      {
        glVertex2f(pt_2D_frame(0),pt_2D_frame(1));
      }
    }
    glEnd();
  }
  
  void displaySteps(CGlWindow & window, GLuint & text2D,
      image::Image<unsigned char> &currentImage, Frame * frame, unsigned int sleep_time = 3)
  {

    Vec2 ptA ,ptB,ptC = Vec2(-1,-1);
    float ptSA,ptSB,ptSC = -1;
    size_t max_steps = std::max<size_t>(display_pt2d_A.size(),std::max<size_t>(display_pt2d_B.size(),display_pt2d_C.size()));
    
    for (size_t step_i = 0; step_i < max_steps; step_i++)
    {
      std::cout<<display_text[step_i]<<"\n";

      displayImage(window,text2D,currentImage);
      displayDetectedFeatures(frame);

      size_t max_step = std::max<size_t>(display_pt2d_A[step_i].size(),std::max<size_t>(display_pt2d_B[step_i].size(),display_pt2d_C[step_i].size()));

      for (size_t i=0; i<max_step; ++i)
      {
        if (!display_pt2d_A[step_i].empty())
          ptA = display_pt2d_A[step_i][i];
        else
          ptA = Vec2(-1,-1);
        if (!display_pt2d_B[step_i].empty())
          ptB = display_pt2d_B[step_i][i];
        else
          ptB = Vec2(-1,-1);
        if (!display_pt2d_C[step_i].empty())
          ptC = display_pt2d_C[step_i][i];
        else
          ptC = Vec2(-1,-1);

        if (!display_size_A[step_i].empty())
          ptSA = display_size_A[step_i][i];
        else
          ptSA = -1;
        if (!display_size_B[step_i].empty())
          ptSB = display_size_B[step_i][i];
        else
          ptSB = -1;
        if (!display_size_C[step_i].empty())
          ptSC = display_size_C[step_i][i];
        else
          ptSC = -1;

        glPointSize(6.0f);

        // Projected point to current frame  - RED/GREEN
        if (ptC(0) != -1 && ptC(1) != -1)
        {
          if (ptB(0) != -1 && ptB(1) != -1)
          {
            glColor3f(0.f, 1.f, 0.f); //GREEN
          }
          else
          {
            glColor3f(1.f, 0.f, 0.f); //RED
          }

          glBegin(GL_POINTS);
          glVertex2f(ptC(0),ptC(1));
          glEnd();
          if (ptSC!=-1)
            DrawCircle(ptC(0),ptC(1),ptSC);
        }

        glPointSize(4.0f);

        // feature matched in prev frame  - RED
        if (ptA(0) != -1 && ptA(1) != -1)
        {
          glColor3f(1.f, 0.f, 0.f);
          glBegin(GL_POINTS);
          glVertex2f(ptA(0),ptA(1));
          glEnd();
          if (ptSA!=-1)
            DrawCircle(ptA(0),ptA(1),ptSA);
        }

        // feature matched to current frame  - BLUE
        if (ptB(0) != -1 && ptB(1) != -1)
        {
          glColor3f(0.f, 0.f, 1.f);
          glBegin(GL_POINTS);
          glVertex2f(ptB(0),ptB(1));
          glEnd();
          if (ptSB!=-1)
            DrawCircle(ptB(0),ptB(1),ptSB);
        }


        // Line from point in prev to predicted - WHITE
        if (ptA(0) != -1 &&  ptA(1) != -1 && ptB(0) != -1 && ptB(1) != -1)
        {
          glColor3f(1.f, 1.f, 1.f);
          glBegin(GL_LINE_STRIP);
          glVertex2f(ptA(0),ptA(1));
          glVertex2f(ptB(0),ptB(1));
          glEnd();
        }

        // Line from point in prev to new triangulated
        if (ptC(0) != -1 && ptC(1) != -1 && ptB(0) != -1 && ptB(1) != -1)
        {
          glColor3f(1.f, 1.f, 1.f);
          glBegin(GL_LINE_STRIP);
          glVertex2f(ptB(0),ptB(1));
          glVertex2f(ptC(0),ptC(1));
          glEnd();
        }
        else if (ptC(0) != -1 && ptC(1) != -1 && ptA(0) != -1 && ptA(1) != -1)
        {
          glColor3f(1.f, 1.f, 1.f);
          glBegin(GL_LINE_STRIP);
          glVertex2f(ptA(0),ptA(1));
          glVertex2f(ptC(0),ptC(1));
          glEnd();
        }
      }
    }
  }
#endif // !SWINE_NOGL

  void resetSteps()
  {
    display_pt2d_A.clear();
    display_pt2d_B.clear();
    display_pt2d_C.clear();
    display_size_A.clear();
    display_size_B.clear();
    display_size_C.clear();
    display_text.clear();
    n_steps_in_bundle=0;

  }


};


}
}
