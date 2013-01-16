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

#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Eigen>

namespace SceneLib2 {

class Camera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Camera();
  ~Camera();

  void SetCameraParameters(const int camera_width, const int camera_height,
                           const double fku, const double fkv,
                           const double u0, const double v0,
                           const double kd1, const int sd);

  Eigen::Vector2d Project(const Eigen::Vector3d &c);
  Eigen::Vector3d Unproject(const Eigen::Vector2d& image);
  Eigen::MatrixXd ProjectionJacobian();
  Eigen::MatrixXd UnprojectionJacobian();
  Eigen::Matrix2d MeasurementNoise(const Eigen::Vector2d& h);

  int               width_;
  int               height_;
  double            kd1_;
  double            fku_;
  double            fkv_;
  Eigen::Vector2d   centre_;
  double            measurement_sd_;
  Eigen::Matrix3d   c_;
  Eigen::Matrix3d   c_inv_;
  Eigen::Vector3d   last_camera_;
  Eigen::Vector2d   last_image_centred_;

  double            xL_, xR_, yB_, yT_;
  double            xW_, xM_, yH_, yM_;

  const double      kNear_;
  const double      kFar_;
};

} // namespace SceneLib2

#endif // CAMERA_H
