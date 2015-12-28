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

#ifndef GRAPHICTOOL_H
#define GRAPHICTOOL_H

#include <Eigen/Eigen>
#include <pangolin/pangolin.h>

#include <GL/freeglut.h>

#include "feature.h"

namespace SceneLib2 {

using namespace std;

class MonoSLAM;

class GraphicTool {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraphicTool(MonoSLAM *monoslam);
  ~GraphicTool();

  void Draw3dScene(const bool &chk_display_trajectory, const bool &chk_display_3d_features,
                   const bool &chk_display_3d_uncertainties);

  void DrawAR(cv::Mat frame,
              const bool &chk_rectify_image_display, const bool &chk_display_trajectory,
              const bool &chk_display_3d_features, const bool &chk_display_3d_uncertainties,
              const bool &chk_display_2d_descriptors, const bool &chk_display_2d_search_regions,
              const bool &chk_display_initialisation);

  int Picker(int x, int y, bool threed);

 private:
  void Init();
  void DrawRectifiedAR(cv::Mat frame,
                       const bool &chk_display_trajectory,
                       const bool &chk_display_3d_features,
                       const bool &chk_display_3d_uncertainties);
  void DrawRawAR(cv::Mat frame,
                 const bool &chk_display_2d_descriptors,
                 const bool &chk_display_2d_search_regions,
                 const bool &chk_display_initialisation);

  void DrawCamera(const Eigen::Vector3d &r_w, const Eigen::Quaterniond &q_wr);
  void DrawCameraTrajectory(const vector<Eigen::Vector3d> &trajectory_store);
  void DrawEstimatedSemiInfiniteLine(const Eigen::VectorXd &yigraphics,
                                     const double line_length, const int name_to_draw);
  void DrawAxes();
  void DrawDescriptors(double draw_patch_x, double draw_patch_y,
                       const cv::Mat &patch, const int name_to_draw);
  void DrawPatch(const double u, const double v, const cv::Mat &patch);
  void Draw2DPartiallyInitialisedLineEllipses(Feature *fp);
  void Draw2DInitialisationBoxes(const bool location_selected_flag,
                                 const bool init_feature_search_region_defined_flag,
                                 const unsigned int uu, unsigned int vv,
                                 const unsigned int init_feature_search_ustart,
                                 const unsigned int init_feature_search_vstart,
                                 const unsigned int init_feature_search_ufinish,
                                 const unsigned int init_feature_search_vfinish,
                                 const unsigned int BOXSIZE);

  void DrawFrame(const cv::Mat &frame, const double zC, const bool rectified);
  void DrawTexturedRectangle(const Eigen::Vector3d rW,
                             const Eigen::Quaterniond qWO,
                             const double rectangle_width,
                             const double rectangle_height,
                             const cv::Mat &imageref);
  void DrawDistortedTexturedRectangle(const Eigen::Vector3d rW,
                                      const Eigen::Quaterniond qWO,
                                      const double rectangle_width,
                                      const double rectangle_height,
                                      const cv::Mat &imageref,
                                      double kappa,
                                      const Eigen::Vector2d& distortion_centre);
  unsigned int GetNextPowerOf2(const unsigned int number);
  Eigen::Vector2d UndistortPoint(const double kappa,
                                 const Eigen::Vector2d& source_point,
                                 const Eigen::Vector2d& distortion_centre);

  void DrawCylinder(const double radius, const double height, const bool drawends);
  void DrawCone(const double bottomradius, const double topradius, const double height);
  void DrawCurve(float width, float curveheight, int m, int n);
  void DrawCurveLid(float width, float curveheight, float depth, int m);
  void DrawLine(const Eigen::Vector3d &r0, Eigen::Vector3d &r1);
  void DrawPoint(const Eigen::Vector3d &r0, const int name);
  void DrawCovariance(const Eigen::VectorXd rW, const Eigen::MatrixXd Prr,
                      const double number_of_sigma, const int name);
  void Draw2DCovariance(const double u, const double v, const Eigen::Matrix2d &P,
                        const double number_of_sigma, const double ring_thickness);
  void DrawNumber(const double number);
  void Draw2DRectangle(const double u, const double v, const double width, const double height);

  void RotateDrawingPosition(const Eigen::Quaterniond &qON);
  void SetFeatureColour(const bool selected_flag, const bool successful_measurement_flag, const bool marked_flag);
  void Draw2DBegin(double u, double v);
  void Draw2DEnd();
  void Set2DDrawingPosition(const double u, const double v);
  void SetProjectionMatrix();
  void UpdateDrawingDimension(const Eigen::Vector3d& v);

  bool selection_mode_;
  bool bInitialised;

  cv::Mat *frame_;
  bool chk_rectify_image_display_;
  bool chk_display_trajectory_;
  bool chk_display_3d_features_;
  bool chk_display_3d_uncertainties_;
  bool chk_display_2d_descriptors_;
  bool chk_display_2d_search_regions_;
  bool chk_display_initialisation_;

  GLUquadricObj *sphere_quad_;
  GLUquadricObj *cylinder_quad_;
  GLUquadricObj *circle_quad_;

  MonoSLAM  *monoslam_ptr_;

  double  xmin_, xmax_, ymin_, ymax_, zmin_, zmax_;

  GLuint  texName_;
  int     texWidth_, texHeight_;

  int     clicked_x_, clicked_y_;

  // q in state vector is qWR between world frame and Scene robot frame
  // What we need to plot though uses GL object frame O
  // Know qRO: pi rotation about y axis
  // qWO = qWR * qRO
  const Eigen::Quaterniond  kQR0_;    // (0.0, 0.0, 1.0, 0.0)

  const double  kMoveClippingPlaneFactor_;

  const double  kSemiInfiniteLineLength_;
  const double  kCovariancesNumberOfSigma_;
  const int     kDrawNOverlappingEllipses_;
};

} // namespace SceneLib2

#endif // GRAPHICTOOL_H
