/* This file is part of the SceneLib2 Project.
 * http://hanmekim.blogspot.com/2012/10/scenelib2-monoslam-open-source-library.html
 * https://github.com/hanmekim/SceneLib2
 *
 * Copyright (c) 2012 Hanme Kim (hanme.kim@gmail.com)
 *
 * SceneLib2 is an open-source C++ library for SLAM originally designed and
 * implemented by Andrew Davison and colleagues at the University of Oxford.
 *
 * I reimplemented his version with the following objectives;
 *  1. Understand his MonoSLAM algorithm in code level.
 *  2. Replace older libraries (i.e. VW34, GLOW, VNL, Pthread) with newer ones
 *     (Pangolin, Eigen3, Boost).
 *  3. Support USB camera instead of IEEE1394.
 *  4. Make it more portable and convenient by using CMake and git repository.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "graphictool.h"

#include "monoslam.h"
#include "support/math_util.h"

namespace SceneLib2 {

GraphicTool::GraphicTool(MonoSLAM *monoslam)
  : kQR0_(0.0, 0.0, 1.0, 0.0), kMoveClippingPlaneFactor_(0.999999),
    kSemiInfiniteLineLength_(10.0), kCovariancesNumberOfSigma_(3),
    kDrawNOverlappingEllipses_(10)
{
  monoslam_ptr_ = monoslam;

  xmin_ = xmax_ = ymin_ = ymax_ = zmin_ = zmax_ = 0.0;

  texName_ = 0;
  texWidth_ = 0;
  texHeight_ = 0;

  bInitialised = false;
}

GraphicTool::~GraphicTool()
{
  gluDeleteQuadric(sphere_quad_);
  gluDeleteQuadric(cylinder_quad_);
  gluDeleteQuadric(circle_quad_);
}

void GraphicTool::Init()
{
  // Set up light and model surface properties
  GLfloat mat_ambient[] = {0.5, 0.5, 0.5, 1.0};
  GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat mat_shininess[] = {50.0};
  GLfloat light_position[] = {1.0, 100.0, 10.0, 0.0};
  GLfloat model_ambient[] = {0.5, 0.5, 0.5, 1.0};

  glClearColor(0.0, 0.0, 0.0, 0.0);

  // The material that the scene objects are made out of
  glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

  // The light model
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, model_ambient);

  // Activate some GL stuff
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glEnable(GL_COLOR_MATERIAL);

  // Allocate sphere object which we will hold onto until the end
  sphere_quad_ = gluNewQuadric();
  gluQuadricDrawStyle(sphere_quad_, GLU_FILL);
  gluQuadricNormals(sphere_quad_, GLU_SMOOTH);

  // Allocate cylinder object which we will hold onto until the end
  cylinder_quad_ = gluNewQuadric();
  gluQuadricDrawStyle(cylinder_quad_, GLU_FILL);
  gluQuadricNormals(cylinder_quad_, GLU_SMOOTH);

  // Allocate circle object which we will hold onto until the end
  circle_quad_ = gluNewQuadric();
  gluQuadricDrawStyle(circle_quad_, GLU_FILL);
  gluQuadricNormals(circle_quad_, GLU_NONE);

  selection_mode_ = false;
  bInitialised = true;
}

void GraphicTool::Draw3dScene(const bool &chk_display_trajectory,
                              const bool &chk_display_3d_features,
                              const bool &chk_display_3d_uncertainties)
{
  if(!bInitialised)
    Init();

  // save the current settings for selection
  chk_display_trajectory_ = chk_display_trajectory;
  chk_display_3d_features_ = chk_display_3d_features;
  chk_display_3d_uncertainties_ = chk_display_3d_uncertainties;

  // need to reset these to decrease axes accordingly
  xmin_ = xmax_ = ymin_ = ymax_ = zmin_ = zmax_ = 0.0;

  glPushAttrib(GL_ALL_ATTRIB_BITS);

  monoslam_ptr_->motion_model_->func_xp(monoslam_ptr_->xv_);
  monoslam_ptr_->motion_model_->func_r(monoslam_ptr_->motion_model_->xpRES_);
  monoslam_ptr_->motion_model_->func_q(monoslam_ptr_->motion_model_->xpRES_);

  Eigen::Quaterniond qWO = monoslam_ptr_->motion_model_->qRES_ * kQR0_;
  Eigen::Vector3d r_local = monoslam_ptr_->motion_model_->rRES_;

  DrawCamera(r_local, qWO);
  UpdateDrawingDimension(r_local);

  if (chk_display_trajectory) {
    DrawCameraTrajectory(monoslam_ptr_->trajectory_store_);
  }

  if (chk_display_3d_features || chk_display_3d_uncertainties) {
    for (vector<Feature *>::const_iterator it = monoslam_ptr_->feature_list_.begin();
         it != monoslam_ptr_->feature_list_.end(); ++it) {

      (*it)->feature_model_->func_yigraphics_and_Pyiyigraphics((*it)->y_, (*it)->Pyy_);

      SetFeatureColour((*it)->selected_flag_, (*it)->successful_measurement_flag_, monoslam_ptr_->marked_feature_label_ == int((*it)->label_));

      if (chk_display_3d_features) {
        if ((*it)->fully_initialised_flag_) {
          DrawPoint((*it)->feature_model_->yigraphicsRES_, (*it)->label_ + 1);
          UpdateDrawingDimension((*it)->feature_model_->yigraphicsRES_);
        }
        else  {
          DrawEstimatedSemiInfiniteLine((*it)->feature_model_->yigraphicsRES_, kSemiInfiniteLineLength_, (*it)->label_ + 1);
        }
      }

      if (chk_display_3d_uncertainties) {
        if ((*it)->fully_initialised_flag_) {
          DrawCovariance((*it)->feature_model_->yigraphicsRES_, (*it)->feature_model_->PyiyigraphicsRES_, kCovariancesNumberOfSigma_, (*it)->label_ + 1);
        }
      }
    }
  }

  if(chk_display_3d_features) {
    DrawAxes();
  }

  glPopAttrib();
}

void GraphicTool::DrawAR(cv::Mat frame,
                         const bool &chk_rectify_image_display,
                         const bool &chk_display_trajectory,
                         const bool &chk_display_3d_features,
                         const bool &chk_display_3d_uncertainties,
                         const bool &chk_display_2d_descriptors,
                         const bool &chk_display_2d_search_regions,
                         const bool &chk_display_initialisation)
{
  if(!bInitialised)
    Init();

  // save the current settings for selection
  frame_ = &frame;
  chk_rectify_image_display_ = chk_rectify_image_display;
  chk_display_trajectory_ = chk_display_trajectory;
  chk_display_3d_features_ = chk_display_3d_features;
  chk_display_3d_uncertainties_ = chk_display_3d_uncertainties;
  chk_display_2d_descriptors_ = chk_display_2d_descriptors;
  chk_display_2d_search_regions_ = chk_display_2d_search_regions;
  chk_display_initialisation_ = chk_display_initialisation;

  glPushAttrib(GL_ALL_ATTRIB_BITS);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  SetProjectionMatrix();

  if (chk_rectify_image_display) {
    DrawRectifiedAR(frame,
                    chk_display_trajectory,
                    chk_display_3d_features,
                    chk_display_3d_uncertainties);
  }
  else {
    DrawRawAR(frame,
              chk_display_2d_descriptors,
              chk_display_2d_search_regions,
              chk_display_initialisation);
  }

  glPopAttrib();
}

void GraphicTool::DrawRectifiedAR(cv::Mat frame,
                                  const bool &chk_display_trajectory,
                                  const bool &chk_display_3d_features,
                                  const bool &chk_display_3d_uncertainties)
{
  Eigen::Vector3d     r_local = monoslam_ptr_->motion_model_->rRES_;
  Eigen::Quaterniond  qWR = monoslam_ptr_->motion_model_->qRES_;

  // need to reset these to decrease axes accordingly
  xmin_ = xmax_ = ymin_ = ymax_ = zmin_ = zmax_ = 0.0;

  if (chk_display_trajectory) {
    DrawCameraTrajectory(monoslam_ptr_->trajectory_store_);
  }

  if (chk_display_3d_features || chk_display_3d_uncertainties) {

    glPushMatrix();
    glTranslatef(r_local(0), r_local(1), r_local(2));
    RotateDrawingPosition(qWR);

    for (vector<Feature *>::const_iterator it = monoslam_ptr_->feature_list_.begin();
         it != monoslam_ptr_->feature_list_.end(); ++it) {
      SetFeatureColour((*it)->selected_flag_, (*it)->successful_measurement_flag_, monoslam_ptr_->marked_feature_label_ == (*it)->label_);

      // Form state and covariance for drawing
      (*it)->feature_model_->func_zeroedyigraphics_and_Pzeroedyigraphics(
            (*it)->y_, monoslam_ptr_->xv_, monoslam_ptr_->Pxx_, (*it)->Pxy_, (*it)->Pyy_);

      if (chk_display_3d_features) {
        if((*it)->fully_initialised_flag_) {
          DrawPoint((*it)->feature_model_->zeroedyigraphicsRES_, (*it)->label_ + 1);
          UpdateDrawingDimension((*it)->feature_model_->zeroedyigraphicsRES_);
        }
        else {
          DrawEstimatedSemiInfiniteLine((*it)->feature_model_->zeroedyigraphicsRES_, kSemiInfiniteLineLength_, (*it)->label_ + 1);
        }
      }

      if(chk_display_3d_uncertainties) {
        if((*it)->fully_initialised_flag_) {
          // The covariance we draw is relative to the robot
          DrawCovariance((*it)->feature_model_->zeroedyigraphicsRES_, (*it)->feature_model_->PzeroedyigraphicsRES_,
                         kCovariancesNumberOfSigma_, (*it)->label_ + 1);
        }
        // Note that we don't yet draw uncertainty on lines
      }
    }

    glPopMatrix();
  }

  if (chk_display_3d_features) {
    // Move to origin
    glLoadIdentity();
    DrawAxes();
  }

  if(!frame.empty()) {
    DrawFrame(frame, 100, true);
  }
}

void GraphicTool::DrawRawAR(cv::Mat frame,
                            const bool &chk_display_2d_descriptors,
                            const bool &chk_display_2d_search_regions,
                            const bool &chk_display_initialisation)
{
  monoslam_ptr_->motion_model_->func_xp(monoslam_ptr_->xv_);
  monoslam_ptr_->motion_model_->func_dxp_by_dxv(monoslam_ptr_->xv_);

  if (chk_display_2d_descriptors || chk_display_2d_search_regions) {
    for (vector<Feature *>::const_iterator it = monoslam_ptr_->feature_list_.begin();
         it != monoslam_ptr_->feature_list_.end(); ++it) {

      SetFeatureColour((*it)->selected_flag_, (*it)->successful_measurement_flag_, monoslam_ptr_->marked_feature_label_ == int((*it)->label_));

      if ((*it)->fully_initialised_flag_) {
        monoslam_ptr_->full_feature_model_->func_hi_and_dhi_by_dxp_and_dhi_by_dyi(
              (*it)->y_, monoslam_ptr_->motion_model_->xpRES_);
        monoslam_ptr_->full_feature_model_->func_zeroedyigraphics_and_Pzeroedyigraphics(
              (*it)->y_, monoslam_ptr_->xv_, monoslam_ptr_->Pxx_, (*it)->Pxy_, (*it)->Pyy_);

        // Only draw 2D stuff if z>0 (or we get some nasty loop-around
        // drawing of features behind the camera)
        if (monoslam_ptr_->full_feature_model_->zeroedyigraphicsRES_(2) > 0) {
          // Draw search regions
          if (chk_display_2d_search_regions) {
            if ((*it)->selected_flag_) {
              if (selection_mode_)
                glLoadName(GLuint((*it)->label_ + 1));

              Draw2DCovariance((*it)->h_(0), (*it)->h_(1), (*it)->S_, kCovariancesNumberOfSigma_, 1.0);

              glLoadName(0);
            }
          }

          // Draw image patches
          if (chk_display_2d_descriptors) {
            // Where to draw the patches?
            double draw_patch_x = -1, draw_patch_y = -1;

            if((*it)->selected_flag_ && (*it)->successful_measurement_flag_) {
              // If we just successfully matched this feature,
              // then draw match position
              draw_patch_x = (*it)->z_(0);
              draw_patch_y = (*it)->z_(1);
            }
            else {
              // Otherwise current estimated position after update
              draw_patch_x = monoslam_ptr_->full_feature_model_->hiRES_(0);
              draw_patch_y = monoslam_ptr_->full_feature_model_->hiRES_(1);
            }

            DrawDescriptors(draw_patch_x, draw_patch_y, (*it)->patch_, (*it)->label_ + 1);
          }
        }
      }
      else {
        if (chk_display_2d_search_regions) {
          Draw2DPartiallyInitialisedLineEllipses((*it));
        }
      }
    }
  }

  if (chk_display_initialisation) {
    Draw2DInitialisationBoxes(monoslam_ptr_->location_selected_flag_,
                              monoslam_ptr_->init_feature_search_region_defined_flag_,
                              monoslam_ptr_->uu_,
                              monoslam_ptr_->vv_,
                              monoslam_ptr_->init_feature_search_ustart_,
                              monoslam_ptr_->init_feature_search_vstart_,
                              monoslam_ptr_->init_feature_search_ufinish_,
                              monoslam_ptr_->init_feature_search_vfinish_,
                              monoslam_ptr_->kBoxSize_);
  }

  if(!frame.empty()) {
    DrawFrame(frame, 100, false);
  }
}

void GraphicTool::DrawCamera(const Eigen::Vector3d &r_w, const Eigen::Quaterniond &q_wr)
{
  // All distances in metres
  static const float  height      = 0.054;
  static const float  width       = 0.058;
  static const float  depth       = 0.024;
  static const float  curve       = 0.003;
  static const float  lensposx    = 0.032;
  static const float  lensposy    = 0.034;
  static const float  lensradius  = 0.012;
  static const float  lensheight  = 0.005;

  //Eigen::Quaterniond  qw0 = q_wr * kQR0_;
  Eigen::Quaterniond  qw0 = q_wr;

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);
  glShadeModel(GL_SMOOTH);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  glTranslatef(r_w(0), r_w(1), r_w(2));

  RotateDrawingPosition(qw0);

  glPushMatrix();

  glColor4f(0.59f, 0.59f, 0.59f, 0.0f);

  glTranslatef(0, 0, -depth/2);
  glRotatef(180, 0, 1, 0);

  // Draw the lens area
  DrawCylinder(lensradius, lensheight, false);
  DrawCone(0.5*lensradius, lensradius, lensheight);
  glColor4f(0.1f, 0.1f, 0.1f, 0.0f);
  DrawCone(0, 0.55*lensradius, 0.1*lensheight);

  // Go back to original position
  glPopMatrix();

  // Move to bottom left
  glTranslatef(-lensposx, -lensposy, -depth/2);
  glColor4f(0.59f, 0.59f, 0.59f, 0.0f);

  // Top bevel
  DrawCurve(width, curve, -1, -1);
  DrawCurveLid(width, curve, depth, -1);
  glTranslatef(0, 0, depth);
  DrawCurve(width, curve, -1, 1);

  glTranslatef(0, height, -depth);
  DrawCurve(width, curve, 1, -1);
  DrawCurveLid(width, curve, depth, 1);
  glTranslatef(0, 0, depth);
  DrawCurve(width, curve, 1, 1);
  glTranslatef(0, -height, -depth);

  // Draw front, back and sides
  glBegin(GL_TRIANGLE_STRIP);
  glNormal3f(0,0,-1);
  glVertex3f(0,0,0);
  glVertex3f(0,height,0);
  glVertex3f(width,0,0);
  glVertex3f(width,height,0);
  glEnd();

  glBegin(GL_TRIANGLE_STRIP);
  glNormal3f(0,0,1);
  glVertex3f(0,0,depth);
  glVertex3f(0,height,depth);
  glVertex3f(width,0,depth);
  glVertex3f(width,height,depth);
  glEnd();

  glBegin(GL_TRIANGLE_STRIP);
  glNormal3f(-1,0,0);
  glVertex3f(0,0,0);
  glVertex3f(0,height,0);
  glVertex3f(0,0,depth);
  glVertex3f(0,height,depth);
  glEnd();

  glBegin(GL_TRIANGLE_STRIP);
  glNormal3f(1,0,0);
  glVertex3f(width,0,0);
  glVertex3f(width,height,0);
  glVertex3f(width,0,depth);
  glVertex3f(width,height,depth);
  glEnd();

  glPopMatrix();
}

// Draw a trajectory.
void GraphicTool::DrawCameraTrajectory(const vector<Eigen::Vector3d> &trajectory_store)
{
  glColor4f(1.0f, 1.0f, 0.0f, 0.0f);

  Eigen::Vector3d r0, r1;

  if (trajectory_store.size() >= 2) {
    vector<Eigen::Vector3d>::const_iterator previous = trajectory_store.begin();

    for (vector<Eigen::Vector3d>::const_iterator it = trajectory_store.begin() + 1;
         it != trajectory_store.end(); ++it) {
      r0 = (*previous);
      r1 = (*it);
      DrawLine(r0, r1);
      ++previous;
    }
  }
}

void GraphicTool::DrawEstimatedSemiInfiniteLine(const Eigen::VectorXd &yigraphics, const double line_length, const int name_to_draw)
{
  // Semi-infinite line representation is end point and normalised
  // direction vector yigraphics = (x, y, z, hhatx, hhaty, hhatz)
  // Since we aren't really going to draw a semi-infinite line,
  // draw it some length
  Eigen::Vector3d y0(yigraphics(0), yigraphics(1), yigraphics(2));
  Eigen::Vector3d hhat(yigraphics(3), yigraphics(4), yigraphics(5));
  Eigen::Vector3d y1 = y0 + hhat * line_length;

  if (selection_mode_)
    glLoadName(GLuint(name_to_draw));

  // Draw line with raw GL to avoid including it in rotation centre
  glDisable(GL_LIGHT0);
  glDisable(GL_LIGHTING);

  glBegin(GL_LINE_STRIP);
  glVertex3d(y0(0), y0(1), y0(2));
  glVertex3d(y1(0), y1(1), y1(2));
  glEnd();

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glLoadName(0);
}

void GraphicTool::DrawAxes()
{
  glDisable(GL_LIGHT0);
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_COLOR_MATERIAL);
  glColor4f(1.0f, 1.0f, 1.0f, 0.0f);

  Eigen::Vector3d r0, r1;

  // Where to draw the "origin" of our axes?
  // At 0, 0, 0 if our model data spans that point
  // But otherwise we're prepared to shift it
  double  origin_x = 0.0;
  double  origin_y = 0.0;
  double  origin_z = 0.0;

  // Move to origin
//  glLoadIdentity();

  // X axis
  r0 << xmin_, origin_y, origin_z;
  r1 << xmax_, origin_y, origin_z;
  DrawLine(r0, r1);

  // Y axis
  r0 << origin_x, ymin_, origin_z;
  r1 << origin_x, ymax_, origin_z;
  DrawLine(r0, r1);

  // Z axis
  r0 << origin_x, origin_y, zmin_;
  r1 << origin_x, origin_y, zmax_;
  DrawLine(r0, r1);

  // Draw axes labels
  glRasterPos3f(xmax_, origin_y, origin_z);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'X');

  glRasterPos3f(origin_x, ymax_, origin_z);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'Y');

  glRasterPos3f(origin_x, origin_y, zmax_);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'Z');

  // Draw axes scale
  // Work out a suitable scale: we want to use the same scale for
  // all the axes so add them all up to start with
  double  axes_range     = xmax_ - xmin_ + ymax_ - ymin_ + zmax_ - zmin_;
  double  log_axes_range = log10(axes_range);
  int     power          = int(log_axes_range);
  double  unit           = pow(10.0, power - 1);

  // Origin
  glRasterPos3f(origin_x, origin_y, origin_z);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '0');

  // X axis
  // Positive
  for (double x = origin_x + unit; x < xmax_; x += unit) {
    glRasterPos3f(x, origin_y, origin_z);
    DrawNumber(x);
  }
  // Negative
  for (double x = origin_x - unit; x > xmin_; x -= unit) {
    glRasterPos3f(x, origin_y, origin_z);
    DrawNumber(x);
  }

  // Y axis
  // Positive
  for (double y = origin_y + unit; y < ymax_; y += unit) {
    glRasterPos3f(origin_x, y, origin_z);
    DrawNumber(y);
  }
  // Negative
  for (double y = origin_y - unit; y > ymin_; y -= unit) {
    glRasterPos3f(origin_x, y, origin_z);
    DrawNumber(y);
  }

  // Z axis
  // Positive
  for (double z = origin_z + unit; z < zmax_; z += unit) {
    glRasterPos3f(origin_x, origin_y, z);
    DrawNumber(z);
  }
  // Negative
  for (double z = origin_z - unit; z > zmin_; z -= unit) {
    glRasterPos3f(origin_x, origin_y, z);
    DrawNumber(z);
  }

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}

void GraphicTool::DrawDescriptors(double draw_patch_x, double draw_patch_y,
                                  const cv::Mat &patch, const int name_to_draw)
{
  if (selection_mode_)
    glLoadName(GLuint(name_to_draw));

  DrawPatch(draw_patch_x, draw_patch_y, patch);
  Draw2DRectangle(draw_patch_x, draw_patch_y,
                  monoslam_ptr_->kBoxSize_ + 2, monoslam_ptr_->kBoxSize_ + 2);

  glLoadName(0);
}

void GraphicTool::DrawPatch(const double u, const double v, const cv::Mat &patch)
{
  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);

  glShadeModel(GL_FLAT);

  int texture_width = int(pow(2.0, int((log(double(patch.size().width)) / log(2.0)) + 1.0)));
  int texture_height = int(pow(2.0, int((log(double(patch.size().height)) / log(2.0)) + 1.0)));

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  GLuint tex_name;
  glGenTextures(1, &tex_name);
  glBindTexture(GL_TEXTURE_2D, tex_name);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  GLubyte *dummy_texture = new GLubyte [texture_width * texture_height];

  for (int i = 0; i < texture_width * texture_height; ++i)
    dummy_texture[i] = (unsigned char)(double(texture_width * texture_height) / double(i) * double(255));

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_width, texture_height, 0,
               GL_LUMINANCE, GL_UNSIGNED_BYTE, dummy_texture);

  delete dummy_texture;

  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, patch.size().width,
                  patch.size().height, GL_LUMINANCE, GL_UNSIGNED_BYTE,
                  patch.data);

  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

  // Save the current drawing and projection matrices
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  Set2DDrawingPosition(u, v);

  double useful_width = double(patch.size().width) / double(texture_width);
  double useful_height = double(patch.size().height) / double(texture_height);

  glBegin(GL_QUADS);
  glTexCoord2f(0.0, useful_height);
  glVertex3f(-0.5 * 11, 0.5 * 11, 0.0);
  glTexCoord2f(0.0,          0.0);
  glVertex3f(-0.5 * 11, -0.5 * 11, 0.0);
  glTexCoord2f(useful_width, 0.0);
  glVertex3f( 0.5 * 11, -0.5 * 11, 0.0);
  glTexCoord2f(useful_width, useful_height);
  glVertex3f( 0.5 * 11, 0.5 * 11, 0.0);
  glEnd();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glDeleteTextures(1, &tex_name);
  glDisable(GL_TEXTURE_2D);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_COLOR_MATERIAL);
}

// Draw a partially-initialised feature into a GL display as a series of ellipses.
// This steps through the particles representing different values for the free
// parameters and draws ellipses representing. Not every particle is drawn: only
// every n are drawn, determined by the constant DRAW_N_OVERLAPPING_ELLIPSE (a
// value of zero for this means draw every particle). The ellipses represent the
// number of standard deviations set by COVARIANCES_NUMBER_OF_SIGMA.
// @param scene The SLAM map to use
// @param threedtool The GL display to draw to
// @param feature_init_info_vector The partially-initialised features (TODO: Why
// is this not just taken from scene?)
// @param fp The particular feature to draw
void GraphicTool::Draw2DPartiallyInitialisedLineEllipses(Feature *fp)
{
  // Find this particular feature in the list of information about
  // partially-initalised features.
  vector<FeatureInitInfo>::const_iterator feat;

  for (feat = monoslam_ptr_->feature_init_info_vector_.begin();
       feat != monoslam_ptr_->feature_init_info_vector_.end(); ++feat) {
    if (feat->fp_ == fp)
      break;
  }

  // Counter so we only draw every Nth ellipse
  unsigned int  draw_counter = 1;

  // Loop over different values of depth (lambda) to draw some ellipses
  // First see how many to skip when drawing?
  unsigned int particles_step;

  particles_step = max(feat->particle_vector_.size() / kDrawNOverlappingEllipses_, FeatureInitInfo::ParticleVector::size_type(1));

  for (vector<Particle>::const_iterator it = feat->particle_vector_.begin();
       it != feat->particle_vector_.end(); ++it) {

    if (draw_counter != particles_step) {
      ++draw_counter;
    }
    else {
      draw_counter = 1;

      monoslam_ptr_->motion_model_->func_xp(monoslam_ptr_->xv_);

      Eigen::VectorXd lambda_vector = it->lambda_;

      monoslam_ptr_->part_feature_model_->func_hpi_and_dhpi_by_dxp_and_dhpi_by_dyi(
            fp->y_, monoslam_ptr_->motion_model_->xpRES_, lambda_vector);

      Eigen::VectorXd local_hpi = monoslam_ptr_->part_feature_model_->hpiRES_;
      Eigen::MatrixXd local_dhpi_by_dyi = monoslam_ptr_->part_feature_model_->dhpi_by_dyiRES_;

      monoslam_ptr_->motion_model_->func_dxp_by_dxv(monoslam_ptr_->xv_);

      Eigen::MatrixXd local_dhpi_by_dxv =
          monoslam_ptr_->part_feature_model_->dhpi_by_dxpRES_ *
          monoslam_ptr_->motion_model_->dxp_by_dxvRES_;

      fp->feature_model_->func_Ri(local_hpi);

      Eigen::MatrixXd local_Ri = fp->feature_model_->RiRES_;

      fp->feature_model_->func_Si(monoslam_ptr_->Pxx_, fp->Pxy_, fp->Pyy_,
                                  local_dhpi_by_dxv, local_dhpi_by_dyi, local_Ri);

      // Draw ellipses with brightness determined by probability
      glColor4f(1.0f, 1.0f, 0.0f, 0.0f);
      Draw2DCovariance(local_hpi(0), local_hpi(1), fp->feature_model_->SiRES_, kCovariancesNumberOfSigma_, 1.0);
    }
  }
}

// Draw the search box for new features.
// @param threedtool The GL viewer
// @param location_selected_flag Has the user selected a new feature?
// @param init_feature_search_region_defined_flag Is there an automatic search region defined
// @param uu The current user-selected x-coordinate
// @param vv The current user-selected y-coordinate
// @param init_feature_search_ustart The x-start of the automatic search region
// @param init_feature_search_vstart The y-start of the automatic search region
// @param init_feature_search_ufinish The x-finish of the automatic search region
// @param init_feature_search_yfinish The y-finish of the automatic search region
void GraphicTool::Draw2DInitialisationBoxes(const bool location_selected_flag,
                                            const bool init_feature_search_region_defined_flag,
                                            const unsigned int uu, unsigned int vv,
                                            const unsigned int init_feature_search_ustart,
                                            const unsigned int init_feature_search_vstart,
                                            const unsigned int init_feature_search_ufinish,
                                            const unsigned int init_feature_search_vfinish,
                                            const unsigned int BOXSIZE)
{
  glColor4f(0.0f, 1.0f, 0.0f, 0.0f);

  // Draw box selected by user
  if (location_selected_flag) {
    Draw2DRectangle(uu, vv, BOXSIZE, BOXSIZE);
  }

  // Draw initialisation box
  if (init_feature_search_region_defined_flag) {
    Draw2DRectangle((init_feature_search_ustart + init_feature_search_ufinish) / 2.0,
                    (init_feature_search_vstart + init_feature_search_vfinish) / 2.0,
                    (init_feature_search_ufinish - init_feature_search_ustart),
                    (init_feature_search_vfinish - init_feature_search_vstart));
  }
}

void GraphicTool::DrawFrame(const cv::Mat &frame, const double zC, const bool rectified)
{
  // Orientation of screen is same as camera orientation
  // Position of screen in world frame: rW = cW + rcW

  // Vector from camera to screen in camera frame
  Eigen::Vector3d rcC;

  rcC << (double(monoslam_ptr_->camera_->width_) / 2.0 -
          double(monoslam_ptr_->camera_->centre_(0))) *
          zC / monoslam_ptr_->camera_->fku_,

          -(double(monoslam_ptr_->camera_->height_) / 2.0 -
            double(monoslam_ptr_->camera_->centre_(1))) *
          zC / monoslam_ptr_->camera_->fkv_,

         -zC;

  Eigen::Quaterniond  qWO = monoslam_ptr_->motion_model_->qRES_ * kQR0_;
  Eigen::Vector3d     rW = monoslam_ptr_->motion_model_->rRES_ + qWO.toRotationMatrix() * rcC;

  if (!rectified || (monoslam_ptr_->camera_->kd1_ == 0.0)) {
    DrawTexturedRectangle(rW,
                          qWO,
                          double(monoslam_ptr_->camera_->width_) * zC / monoslam_ptr_->camera_->fku_,
                          double(monoslam_ptr_->camera_->height_) * zC / monoslam_ptr_->camera_->fkv_,
                          frame);
  }
  else {
    DrawDistortedTexturedRectangle(rW,
                                   qWO,
                                   double(monoslam_ptr_->camera_->width_) * zC / monoslam_ptr_->camera_->fku_,
                                   double(monoslam_ptr_->camera_->height_) * zC / monoslam_ptr_->camera_->fkv_,
                                   frame,
                                   -monoslam_ptr_->camera_->kd1_,
                                   monoslam_ptr_->camera_->centre_);
  }
}

void GraphicTool::DrawTexturedRectangle(const Eigen::Vector3d rW,
                                        const Eigen::Quaterniond qWO,
                                        const double rectangle_width,
                                        const double rectangle_height,
                                        const cv::Mat &imageref)
{
  glDisable(GL_COLOR_MATERIAL);
  glShadeModel(GL_FLAT);

  if ((texWidth_ < imageref.size().width) || (texHeight_ < imageref.size().height)) {
    // Current texture is too small
    texWidth_ = GetNextPowerOf2(imageref.size().width);
    texHeight_ = GetNextPowerOf2(imageref.size().height);

    if (texName_ > 0) {
      // Texture already exists, so reallocate
      glDeleteTextures(1, &texName_);
    }

    glGenTextures(1, &texName_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glBindTexture(GL_TEXTURE_2D, texName_);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // GL_NEAREST or GL_LINEAR
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    GLubyte *dummy_texture = new GLubyte[texWidth_ * texHeight_];

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texWidth_, texHeight_, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, dummy_texture);

    delete  dummy_texture;
  }
  else {
    // Texture size is OK, so just bind to it
    glBindTexture(GL_TEXTURE_2D, texName_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  }

  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                  imageref.size().width, imageref.size().height,
                  GL_LUMINANCE, GL_UNSIGNED_BYTE, imageref.data);

  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

  // Save the current drawing matrix
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  // Transform into Object frame
  glTranslatef(rW(0), rW(1), rW(2));

  double ang = GetAngleFromQuaternion(qWO);

  if (ang != 0.0) {
    Eigen::Vector3d axis = GetAxisFromQuaternion(qWO);
    glRotatef(ang * 180.0 / M_PI, axis(0), axis(1), axis(2));
  }

  double useful_width = double(imageref.size().width) / double(texWidth_);
  double useful_height = double(imageref.size().height) / double(texHeight_);

  glRotatef(180.0, 1.0, 0.0, 0.0);

  glBegin(GL_QUADS);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(-rectangle_width / 2.0, -rectangle_height / 2.0, 0.0);
  glTexCoord2f(0.0, useful_height);
  glVertex3f(-rectangle_width / 2.0, rectangle_height / 2.0, 0.0);
  glTexCoord2f(useful_width, useful_height);
  glVertex3f( rectangle_width / 2.0, rectangle_height / 2.0, 0.0);
  glTexCoord2f(useful_width, 0.0);
  glVertex3f(rectangle_width / 2.0, -rectangle_height / 2.0, 0.0);
  glEnd();

  glPopMatrix();

  glDisable(GL_TEXTURE_2D);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_COLOR_MATERIAL);
}

void GraphicTool::DrawDistortedTexturedRectangle(const Eigen::Vector3d rW,
                                                 const Eigen::Quaterniond qWO,
                                                 const double rectangle_width,
                                                 const double rectangle_height,
                                                 const cv::Mat &imageref,
                                                 double kappa,
                                                 const Eigen::Vector2d& distortion_centre)
{
  static const unsigned int num_divisions = 10;

  glDisable(GL_COLOR_MATERIAL);
  glShadeModel(GL_FLAT);

  if ((texWidth_ < imageref.size().width) || (texHeight_ < imageref.size().height)) {
    // Current texture is too small
    texWidth_ = GetNextPowerOf2(imageref.size().width);
    texHeight_ = GetNextPowerOf2(imageref.size().height);

    if (texName_ > 0) {
      // Texture already exists, so reallocate
      glDeleteTextures(1, &texName_);
    }

    glGenTextures(1, &texName_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glBindTexture(GL_TEXTURE_2D, texName_);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    GLubyte *dummy_texture = new GLubyte [texWidth_ * texHeight_];

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texWidth_, texHeight_,
                 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, dummy_texture);

    delete dummy_texture;
  }
  else {
    // Texture size is OK, so just bind to it
    glBindTexture(GL_TEXTURE_2D, texName_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  }

  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                  imageref.size().width, imageref.size().height,
                  GL_LUMINANCE, GL_UNSIGNED_BYTE, imageref.data);

  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

  // Save the current drawing matrix
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();


  // Transform into Object frame
  glTranslatef(rW(0), rW(1), rW(2));

  double ang = GetAngleFromQuaternion(qWO);

  if (ang != 0.0) {
    Eigen::Vector3d axis = GetAxisFromQuaternion(qWO);
    glRotatef(ang * 180.0 / M_PI, axis(0), axis(1), axis(2));
  }

  float useful_width = float(imageref.size().width) / float(texWidth_);
  float useful_height = float(imageref.size().height) / float(texHeight_);

  // Constants for stepping through texture
  const float tex_step_x = useful_width / float(num_divisions);
  const float tex_step_y = useful_height / float(num_divisions);
  const float im_step_x = imageref.size().width / float(num_divisions);
  const float im_step_y = imageref.size().height / float(num_divisions);

  // Build an array of new locations
  Eigen::Vector2d new_pos[num_divisions+1][num_divisions+1];

  for (unsigned int r=0; r<(num_divisions+1); r++) {

    const float y = r*im_step_y;

    for (unsigned int c=0; c<(num_divisions+1); c++) {

      const float x = c*im_step_x;

      new_pos[r][c] = UndistortPoint(kappa, Eigen::Vector2d(x,y), distortion_centre);

      double norm_x = rectangle_width*((new_pos[r][c](0) / imageref.size().width) - 0.5);

      double norm_y = rectangle_height*((new_pos[r][c](1) / imageref.size().height) - 0.5);

      new_pos[r][c] << norm_x, norm_y;

    }
  }

  glRotatef(180.0, 1.0, 0.0, 0.0);

  for (unsigned int r=0;r<num_divisions;r++) {
    for (unsigned int c=0;c<num_divisions;c++) {

      // Work out source coords
      const float tex_x0 = tex_step_x*c;
      const float tex_y0 = tex_step_y*r;
      const float tex_x1 = tex_step_x*(c+1);
      const float tex_y1 = tex_step_y*(r+1);

      glBegin(GL_QUADS);

      glTexCoord2f(tex_x0, tex_y0);
      glVertex3f(new_pos[r][c](0), new_pos[r][c](1), 0.0);

      glTexCoord2f(tex_x0, tex_y1);
      glVertex3f(new_pos[r+1][c](0), new_pos[r+1][c](1), 0.0);

      glTexCoord2f(tex_x1, tex_y1);
      glVertex3f(new_pos[r+1][c+1](0), new_pos[r+1][c+1](1), 0.0);

      glTexCoord2f(tex_x1, tex_y0);
      glVertex3f(new_pos[r][c+1](0), new_pos[r][c+1](1), 0.0);

      glEnd();
    }
  }

  glPopMatrix();

  glDisable(GL_TEXTURE_2D);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_COLOR_MATERIAL);
}

// Get the first power of 2 greater than or equal to number
unsigned int GraphicTool::GetNextPowerOf2(const unsigned int number)
{
  // Note that the iterative version (below) is *MUCH* faster than
  // taking logs for small powers of 2 (ie. sizes and the like). For
  // a typical number like 4000 -> 4096, it's about 10x faster.

  // Log version
  //const double lognum = log(double(w+i*2)) / log(2.0);
  //result = int (pow(2.0, ceil(lognum)));

  // Iterative version
  unsigned int  testnum = 2;

  while (testnum < number) {
    testnum *= 2;
  }

  return  testnum;
}

Eigen::Vector2d GraphicTool::UndistortPoint(const double kappa,
                                            const Eigen::Vector2d& source_point,
                                            const Eigen::Vector2d& distortion_centre)
{
  double  rxu, ryu, sq;

  Eigen::Vector2d dest_point = source_point;

  rxu = source_point(0) - distortion_centre(0);
  ryu = source_point(1) - distortion_centre(1);

  sq = rxu*rxu + ryu*ryu;

  dest_point(0) = distortion_centre(0) + rxu / sqrt((1.0+2*kappa*sq));
  dest_point(1) = distortion_centre(1) + ryu / sqrt((1.0+2*kappa*sq));

  return  dest_point;
}

void GraphicTool::DrawCylinder(const double radius, const double height, const bool drawends)
{
  gluCylinder(cylinder_quad_, radius, radius, height, 20, 4);

  if (drawends) {
    glPushMatrix();
    gluDisk(circle_quad_, 0, radius, 20, 2);
    glTranslatef(0, 0, height);
    glRotatef(180, 1, 0, 0);  // So that normals point right way!
    gluDisk(circle_quad_, 0, radius, 20, 2);
    glPopMatrix();
  }
}

void GraphicTool::DrawCone(const double bottomradius, const double topradius, const double height)
{
  gluCylinder(cylinder_quad_, bottomradius, topradius, height, 20, 4);
}

// This function is based on
//  - SceneLib 1.0 written by Dr. Andrew Davison and originally licensed under LGPL
//  - VW34 developed at Oxford's Active Vision Lab and originally licensed under LGPL
void GraphicTool::DrawCurve(float width, float curveheight, int m, int n)
{
  const float         curve = m*curveheight;
  static const float  frac = 0.7;

  glBegin(GL_TRIANGLES);

  glNormal3f(0, 0, n);

  glVertex3f(0, 0, 0);
  glVertex3f(width*0.25, 0, 0);
  glVertex3f(width*0.25, frac*curve, 0);

  glVertex3f(width*0.25, frac*curve, 0);
  glVertex3f(width*0.5, curve, 0);
  glVertex3f(width*0.5, 0, 0);

  glVertex3f(width*0.25, frac*curve, 0);
  glVertex3f(width*0.25, 0, 0);
  glVertex3f(width*0.5, 0, 0);

  glVertex3f(width*0.5, 0, 0);
  glVertex3f(width*0.75, 0, 0);
  glVertex3f(width*0.75, frac*curve, 0);

  glVertex3f(width*0.75, frac*curve, 0);
  glVertex3f(width*0.5, curve, 0);
  glVertex3f(width*0.5, 0, 0);

  glVertex3f(width*0.75, frac*curve, 0);
  glVertex3f(width*0.75, 0, 0);
  glVertex3f(width, 0, 0);

  glEnd();
}

void GraphicTool::DrawCurveLid(float width, float curveheight, float depth, int m)
{
  const float         curve = m*curveheight;
  static const float  frac = 0.7;

  const float         radius = ((0.25*width*width) + (curve*curve)) / (2*curve);
  const float         theta = asin(width/(2*radius));

  const float         nx0 = sin(theta);
  const float         ny0 = m*cos(theta);
  const float         nx1 = sin(0.5*theta);
  const float         ny1 = m*cos(0.5*theta);

  glBegin(GL_TRIANGLE_STRIP);

  glNormal3f(nx0, ny0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, depth);

  glNormal3f(nx1, ny1, 0);
  glVertex3f(width*0.25, frac*curve, 0);
  glVertex3f(width*0.25, frac*curve, depth);

  glNormal3f(0, m, 0);
  glVertex3f(width*0.5, curve, 0);
  glVertex3f(width*0.5, curve, depth);

  glNormal3f(-nx1, ny1, 0);
  glVertex3f(width*0.75, frac*curve, 0);
  glVertex3f(width*0.75, frac*curve, depth);

  glNormal3f(-nx0, ny0, 0);
  glVertex3f(width, 0, 0);
  glVertex3f(width, 0, depth);

  glEnd();
}

void GraphicTool::DrawLine(const Eigen::Vector3d &r0, Eigen::Vector3d &r1)
{
  glDisable(GL_LIGHT0);
  glDisable(GL_LIGHTING);

  glBegin(GL_LINE_STRIP);
  glVertex3f(r0(0), r0(1), r0(2));
  glVertex3f(r1(0), r1(1), r1(2));
  glEnd();

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}

void GraphicTool::DrawPoint(const Eigen::Vector3d &r0, const int name)
{
  if (selection_mode_)
    glLoadName(GLuint(name));

  glDisable(GL_LIGHT0);
  glDisable(GL_LIGHTING);

  glPointSize(4);

  glBegin(GL_POINTS);
  glVertex3f(r0(0), r0(1), r0(2));
  glEnd();

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glLoadName(0);
}

void GraphicTool::DrawCovariance(const Eigen::VectorXd rW,
                                 const Eigen::MatrixXd Prr,
                                 const double number_of_sigma,
                                 const int name)
{
  if (selection_mode_)
    glLoadName(GLuint(name));

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  glTranslatef(rW(0), rW(1), rW(2));

  // Calculate eigenvalues, vectors of P
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>  es(Prr);

  // Eigenvectors become rotation matrix
  // Turn into 4*4 transformation
  GLfloat Varray[16] = {(float)(es.eigenvectors().row(0)(0)), (float)(es.eigenvectors().row(1)(0)), (float)(es.eigenvectors().row(2)(0)), 0.0f,
                        (float)(es.eigenvectors().row(0)(1)), (float)(es.eigenvectors().row(1)(1)), (float)(es.eigenvectors().row(2)(1)), 0.0f,
                        (float)(es.eigenvectors().row(0)(2)), (float)(es.eigenvectors().row(1)(2)), (float)(es.eigenvectors().row(2)(2)), 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f};
  glMultMatrixf(Varray);

  glScalef(sqrt(es.eigenvalues()[0]) * number_of_sigma,
           sqrt(es.eigenvalues()[1]) * number_of_sigma,
           sqrt(es.eigenvalues()[2]) * number_of_sigma);

  gluSphere(sphere_quad_, 1, 20, 20);

  glPopMatrix();
  glLoadName(0);
}

void GraphicTool::Draw2DCovariance(const double u, const double v,
                                   const Eigen::Matrix2d &P,
                                   const double number_of_sigma,
                                   const double ring_thickness)
{
  Draw2DBegin(u, v);

  // Calculate eigenvalues, vectors of P
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>  eigen(P);

  // Eigenvectors become rotation matrix
  // Turn into 4*4 transformation
  GLfloat Varray[16] = {(float)(eigen.eigenvectors().row(0)(0)), (float)(eigen.eigenvectors().row(1)(0)), 0.0f, 0.0f,
                        (float)(eigen.eigenvectors().row(0)(1)), (float)(eigen.eigenvectors().row(1)(1)), 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f};
  glMultMatrixf(Varray);

  glScalef(sqrt(eigen.eigenvalues()[0]) * number_of_sigma,
           sqrt(eigen.eigenvalues()[1]) * number_of_sigma,
           1.0);

  double inner_size = (sqrt(eigen.eigenvalues()[0]) * number_of_sigma - ring_thickness) /
                       (sqrt(eigen.eigenvalues()[0]) * number_of_sigma);

  if (inner_size < 0.0)
    inner_size = 0.0;

  gluDisk(circle_quad_, inner_size, 1.0, 20, 20);

  Draw2DEnd();
}

void GraphicTool::DrawNumber(const double number)
{
  char  number_label[100];
  char  *digit_pointer;

  sprintf(number_label, "%g", number);
  digit_pointer = number_label;

  for (unsigned int i = 0; i < strlen(number_label); ++i) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *(digit_pointer++));
  }
}

void GraphicTool::Draw2DRectangle(const double u, const double v, const double width, const double height)
{
  Draw2DBegin(u, v);

  glBegin(GL_LINES);
  glVertex3f(-0.5 * width, -0.5 * height, 0.0);
  glVertex3f(-0.5 * width,  0.5 * height, 0.0);
  glVertex3f(-0.5 * width,  0.5 * height, 0.0);
  glVertex3f( 0.5 * width,  0.5 * height, 0.0);
  glVertex3f( 0.5 * width,  0.5 * height, 0.0);
  glVertex3f( 0.5 * width, -0.5 * height, 0.0);
  glVertex3f( 0.5 * width, -0.5 * height, 0.0);
  glVertex3f(-0.5 * width, -0.5 * height, 0.0);
  glEnd();

  Draw2DEnd();
}

void GraphicTool::RotateDrawingPosition(const Eigen::Quaterniond &qON)
{
  glMatrixMode(GL_MODELVIEW);

  double angle = GetAngleFromQuaternion(qON);

  if (angle != 0.0) {
    Eigen::Vector3d axis = GetAxisFromQuaternion(qON);
    glRotatef(angle * 180.0 / M_PI, axis(0), axis(1), axis(2));
  }
}

void GraphicTool::SetFeatureColour(const bool selected_flag, const bool successful_measurement_flag, const bool marked_flag)
{
  if (marked_flag) {
    // Marked feature green
   glColor4f(0.0, 1.0, 0.0, 0.0);
  }
  else {
    if (selected_flag) {
      if (successful_measurement_flag)
        // Successfully measured feature red
        glColor4f(1.0, 0.0, 0.0, 0.0);
      else
        // Failed measured feature blue
        glColor4f(0.0, 0.0, 1.0, 0.0);
    }
    else {
      // Unselected feature yellow
      glColor4f(1.0, 1.0, 0.0, 0.0);
    }
  }
}

void GraphicTool::Draw2DBegin(double u, double v)
{
  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);

  glShadeModel(GL_FLAT);

  // Save the current drawing and projection matrices
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  Set2DDrawingPosition(u, v);
}

void GraphicTool::Draw2DEnd()
{
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

void GraphicTool::Set2DDrawingPosition(const double u, const double v)
{
  // Set a drawing position and scale on the near clipping plane so that
  // things can be drawn in camera coordinates.

  // Do it all in projection matrix (which should be saved by any function
  // calling this one)
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // Set projection
  if (selection_mode_) {
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    gluPickMatrix((GLdouble)clicked_x_, (GLdouble)clicked_y_, 7.0, 7.0, viewport);
  }

  // Perspective projection
  glFrustum(kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->xL_,
            kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->xR_,
            kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->yB_,
            kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->yT_,
            kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->kNear_,
            monoslam_ptr_->camera_->kFar_);

  // Now make transformation to clipping plane
  // Perspective projection
  glTranslatef((u - monoslam_ptr_->camera_->centre_(0)) * monoslam_ptr_->camera_->kNear_ / monoslam_ptr_->camera_->fku_,
               (monoslam_ptr_->camera_->centre_(1) - v) * monoslam_ptr_->camera_->kNear_ / monoslam_ptr_->camera_->fkv_,
               -monoslam_ptr_->camera_->kNear_);
  glScalef(monoslam_ptr_->camera_->kNear_ / monoslam_ptr_->camera_->fku_,
           -monoslam_ptr_->camera_->kNear_ / monoslam_ptr_->camera_->fkv_, 1.0);

  // Also set the raster position for drawing text
  glRasterPos3f((u - monoslam_ptr_->camera_->centre_(0)) * monoslam_ptr_->camera_->kNear_ / monoslam_ptr_->camera_->fku_,
                (monoslam_ptr_->camera_->centre_(1) - v) * monoslam_ptr_->camera_->kNear_ / monoslam_ptr_->camera_->fkv_,
                0.0);
}

void GraphicTool::SetProjectionMatrix()
{
  glMatrixMode(GL_PROJECTION);

  if (!selection_mode_) {
    glLoadIdentity();
  }

  // Perspective projection
  glFrustum(kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->xL_,
            kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->xR_,
            kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->yB_,
            kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->yT_,
            kMoveClippingPlaneFactor_ * monoslam_ptr_->camera_->kNear_,
            monoslam_ptr_->camera_->kFar_);

  // Take care of the position of our virtual camera
  // camera_position is vector cW
  // camera_orientation is quaternion qWC
  // Apply them negatively to transform from camera frame to world frame
  Eigen::Quaterniond  qWO = monoslam_ptr_->motion_model_->qRES_ * kQR0_;
  Eigen::Quaterniond  qCW = qWO.inverse();
  Eigen::Vector3d     tCW = -monoslam_ptr_->motion_model_->rRES_;

  double ang = GetAngleFromQuaternion(qCW);

  if (ang != 0.0) {
    Eigen::Vector3d axis = GetAxisFromQuaternion(qCW);
    glRotatef(ang * 180.0 / M_PI, axis(0), axis(1), axis(2));
  }

  glTranslatef(tCW(0), tCW(1), tCW(2));

  glMatrixMode(GL_MODELVIEW);
}

void GraphicTool::UpdateDrawingDimension(const Eigen::Vector3d& v)
{
  if (v(0) < xmin_) xmin_ = v(0);
  if (v(0) > xmax_) xmax_ = v(0);
  if (v(1) < ymin_) ymin_ = v(1);
  if (v(1) > ymax_) ymax_ = v(1);
  if (v(2) < zmin_) zmin_ = v(2);
  if (v(2) > zmax_) zmax_ = v(2);
}

int GraphicTool::Picker(int x, int y, bool threed)
{
  const int SELECTBUFSIZE = 512;

  // Work out which object was clicked on
  GLuint  select_buffer[SELECTBUFSIZE];
  GLint   hits;

  clicked_x_ = x;
  clicked_y_ = y;

  glSelectBuffer(SELECTBUFSIZE, select_buffer);
  glRenderMode(GL_SELECT);

  glInitNames();
  glPushName(0);

  glMatrixMode(GL_PROJECTION);

  GLdouble  proj_mat[16];
  glGetDoublev(GL_PROJECTION_MATRIX, proj_mat);

  glLoadIdentity();

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  gluPickMatrix((GLdouble)x, (GLdouble)y, 7.0, 7.0, viewport);

  if (threed)
    glMultMatrixd(proj_mat);

  glMatrixMode(GL_MODELVIEW);

  if (!threed)
    glLoadIdentity();

  selection_mode_ = true;

  if (threed) {
    Draw3dScene(chk_display_trajectory_,
                chk_display_3d_features_,
                chk_display_3d_uncertainties_);
  }
  else {
    DrawAR(*frame_,
           chk_rectify_image_display_,
           chk_display_trajectory_,
           chk_display_3d_features_,
           chk_display_3d_uncertainties_,
           chk_display_2d_descriptors_,
           chk_display_2d_search_regions_,
           chk_display_initialisation_);
  }

  selection_mode_ = false;

  // Process any hits we've got
  hits = glRenderMode(GL_RENDER);

  GLuint  *ptr = select_buffer;

  // Find if we have selected a real item;
  // if more than one then choose the closest
  int selected_item = 0;
  double  closest_distance = 10000000000.0;

  // Structure of select_buffer:
  // For each hit this ints:
  // 1. number of names (always 1?)
  // 2. z1
  // 3. z2 (? what is the difference?)
  // 4. (or more) the names
  for (int i = 0; i < hits; ++i) {
    GLuint  number_of_names = *ptr++;
    // Skip past z1...
    ptr++;

    double z2 = double(*ptr++ / 0x7fffffff);
    GLuint name = 0;

    for (int j = 0; j < int(number_of_names); ++j) {
      // Don't know why there would be more than one name?
      // Just take the last one
      name = *ptr++;
    }

    if (name != 0) {
      // zero name isn't a real object
      if (z2 < closest_distance) {
        selected_item = name;
        closest_distance = z2;
      }
    }
  }

  return  selected_item;
}

} // namespace SceneLib2
