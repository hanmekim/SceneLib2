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

#include "camera.h"

namespace SceneLib2 {

Camera::Camera()
  : kNear_(0.01), kFar_(100000)
{
}

Camera::~Camera()
{
}

void Camera::SetCameraParameters(const int camera_width, const int camera_height,
                                 const double fku, const double fkv,
                                 const double u0, const double v0,
                                 const double kd1, const int sd)
{
  width_ = camera_width;
  height_ = camera_height;
  fku_ = fku;
  fkv_ = fkv;
  centre_(0) = u0;
  centre_(1) = v0;
  kd1_ = kd1;
  measurement_sd_ = sd;

  // Calculate GL versions of these (see explanation in header file)
  xW_ = width_ * kNear_ / fku_;
  yH_ = height_ * kNear_ / fkv_;
  xM_ = (kNear_ / fku_) * (width_ / 2.0 - centre_(0));
  yM_ = -(kNear_ / fkv_) * (height_ / 2.0 - centre_(1));

  xL_ = xM_ - 0.5 * xW_;
  xR_ = xM_ + 0.5 * xW_;
  yB_ = yM_ - 0.5 * yH_;
  yT_ = yM_ + 0.5 * yH_;
}

// Project a point from an ideal (Euclidean) camera frame into image co-ordinates
// with radial distortion.
//
// This first transforms the input position vector \f$ \vct{y} \f$ (relative to
// the camera position) into an undistorted image location \f$ \vct{u}_c \f$
// (relative to an origin at the optical centre)
// \f[
// \vct{u}_c = \evct{u_c \\ v_c} = \evct{-F_{ku} y_x / y_z \\ -F_{kv} y_y / y_z} =
//   \emat{-F_{ku} & 0 \\ 0 & -F_{kv}}\evct{y_x/y_z \\ y_y/y_z}
// \f]
// Then radial distortion is applied to give the final image location \f$ h \f$
// (with the origin back in the normal location of of the top left of the image).
// \f[
// \vct{h} = \frac{\vct{u}_c}{\sqrt{1 + 2 k_1 |\vct{u}_c|^2}} + \evct{u_0 \\ v_0}
// \f]
Eigen::Vector2d Camera::Project(const Eigen::Vector3d &camera)
{
  // Remember this position in case we are asked about the Jacobians of this
  // transformation
  last_camera_ = camera;

  // First do perspective projection
  // This turns the position vector into an undistorted image location (with the
  // origin at the centre)
  // Use -Fkuv to swap the image co-ordinates (0,0) at the top to
  // camera co-ordinates (0,0) at the bottom
  Eigen::Vector2d imagepos_centred;

  imagepos_centred(0) = -fku_ * camera(0) / camera(2);
  imagepos_centred(1) = -fkv_ * camera(1) / camera(2);

  last_image_centred_ = imagepos_centred;

  // 1 distortion coefficient model
  const double radius2 = (imagepos_centred(0)*imagepos_centred(0) +
                          imagepos_centred(1)*imagepos_centred(1));

  double factor = sqrt(1 + 2 * kd1_ * radius2);
  return imagepos_centred / factor + centre_;
}

// Project a point from image co-ordinates into the ideal (Euclidean) camera frame.
//
// The origin for the image point \f$ \vct{h} \f$ is moved to the optical centre
// and the inverse distortion applied to give the undistorted location
// \f$ \vct{u}_c \f$:
// \f[
// \begin{aligned}
// \vct{u}_0 &= \evct{u_0 \\ v_0}
// \vct{u}_c = \evct{u_c \\ v_c} &=
//   \frac{\vct{h} - \vct{u}_0}{\sqrt{1 - 2 k_1 |\vct{h} - \vct{u}_0|^2}}
// \end{aligned}
// \f]
// This is then unprojected to give the world location
// \f$ \vct{y} = \evct{y_x & y_y & 1}^T \f$ in homogeneous co-ordinates.
// \f[
// \vct{y} = \evct{\frac{1}{-F_{ku}} u_c \\ \frac{1}{-F_{kv}} v_c \\ 1}
// \f]
Eigen::Vector3d Camera::Unproject(const Eigen::Vector2d& image)
{
  Eigen::Vector2d centred;
  centred = image - centre_;

  last_image_centred_ = centred;

  const double radius2 = (centred(0)*centred(0) +
                          centred(1)*centred(1));

  double factor = sqrt(1 - 2 * kd1_ * radius2);

  Eigen::Vector2d undistorted = centred / factor;

  Eigen::Vector3d camera;

  camera(0) = undistorted(0) / -fku_;
  camera(1) = undistorted(1) / -fkv_;
  camera(2) = 1.0;

  return camera;
}

// Calculate the Jacobian \f$ \partfrac{\vct{h}}{\vct{y}} \f$ for the
// Project() operation, for the most recent point that was projected.
//
// This is calculated in two stages:
// \f[
// \begin{aligned}
// \partfrac{ \vct{u} }{ \vct{y} } &= \emat{
//   \frac{ -f_{ku} }{ y_z } & 0 & \frac{ f_{ku} y_x }{ y_z^2 }
//   0 & \frac{ -f_{kv} }{ y_z } & \frac{ f_{kv} y_y }{ y_z^2 } }
// \partfrac{\vct{h}}{\vct{u}} &= \emat{
//     -\frac{ 2 u_c^2 k_1 }{ f^{\frac{3}{2}} } +
//       \frac{ 1 }{ f^{\frac{1}{2}} } &
//     -\frac{ 2 u_c v_c k_1 }{ f^{\frac{3}{2}} }
//     -\frac{ 2 v_c u_c k_1 }{ f^{\frac{3}{2}} } &
//     -\frac{ 2 v_c^2 k_1 }{ f^{\frac{3}{2}} } +
//       \frac{ 1 }{ f^{\frac{1}{2}} } } =
//   -\frac{2 k_1}{f^\frac{3}{2}} \vct{u}_c \vct{u}_c^T +
//     \emat{\frac{1}{\sqrt{f}} & 0
//     0 & \frac{1}{\sqrt{f}}}
// \end{aligned}
// \f]
// where \f$ f = 1 + 2 k_1 |\vct{u}_c|^2 \f$. The final Jacobian is then given by
// combining these two
// \f[
// \partfrac{\vct{h}}{\vct{y}} =
//   \partfrac{\vct{h}}{\vct{u}} \partfrac{\vct{u}}{\vct{y}}
// \f]
Eigen::MatrixXd Camera::ProjectionJacobian()
{
  // Jacobians
  // Normal image measurement
  const double fku_yz = fku_ / last_camera_(2);
  const double fkv_yz = fkv_ / last_camera_(2);
  Eigen::MatrixXd du_by_dy(2,3);

  du_by_dy << -fku_yz, 0.0, fku_yz * last_camera_(0) / last_camera_(2),
              0.0, -fkv_yz, fkv_yz * last_camera_(1) / last_camera_(2);

  // Distortion model Jacobians
  // Generate the outer product matrix first
  Eigen::Matrix2d dh_by_du = last_image_centred_ * last_image_centred_.transpose();

  // this matrix is not yet dh_by_du, it is just
  // [ uc*uc  uc*vc ]
  // [ vc*uc  vc*vc ]
  // The trace of this matrix gives the magnitude of the vector
  const double radius2 = dh_by_du(0,0) + dh_by_du(1,1);

  // Calculate various constants to save typing
  const double distor = 1 + 2 * kd1_ * radius2;
  const double distor1_2 = sqrt(distor);
  const double distor3_2 = distor1_2 * distor;

  // Now form the proper dh_by_du by manipulating the outer product matrix
  dh_by_du *= -2 * kd1_ / distor3_2;
  dh_by_du(0,0) += (1/distor1_2);
  dh_by_du(1,1) += (1/distor1_2);

  return dh_by_du * du_by_dy;
}

// Calculate the Jacobian \f$ \partfrac{\vct{y}}{\vct{h}} \f$ for the
// Unproject() operation, for the most recent point that was projected.
//
// This is calculated in two stages: the Jacobian between undistorted and
// distorted image co-ordinates
// \f[
// \partfrac{\vct{u}}{\vct{h}} = \emat{
//     \frac{ 2 u_c^2 k_1 }{ f^{\frac{3}{2}} } +
//       \frac{ 1 }{ f^{\frac{1}{2}} } &
//     \frac{ 2 u_c v_c k_1 }{ f^{\frac{3}{2}} }
//     \frac{ 2 v_c u_c k_1 }{ f^{\frac{3}{2}} } &
//     \frac{ 2 v_c^2 k_1 }{ f^{\frac{3}{2}} } +
//       \frac{ 1 }{ f^{\frac{1}{2}} } } =
//   \frac{2 k_1}{f^\frac{3}{2}} \vct{u}_c \vct{u}_c^T +
//     \emat{\frac{1}{\sqrt{f}} & 0
//     0 & \frac{1}{\sqrt{f}}}
// \f]
// where \f$ f = 1 - 2 k_1 |\vct{u}_c|^2 \f$ and the Jacobian for the unprojection
// operation between image rays and (undistorted, centred) image co-ordinates
// \f[
// \partfrac{ \vct{y} }{ \vct{u} } = \emat{
//   -\frac{ 1 }{ f_{ku} } & 0
//   0 & -\frac{ 1}{ f_{kv}}
//   0 & 0 }
// \f]
//  The final Jacobian is then given by combining these two
// \f[
// \partfrac{\vct{y}}{\vct{h}} =
//   \partfrac{\vct{y}}{\vct{u}} \partfrac{\vct{u}}{\vct{h}}
// \f]
Eigen::MatrixXd Camera::UnprojectionJacobian()
{
  Eigen::MatrixXd dy_by_du(3,2);

  dy_by_du << -1/fku_, 0.0,
              0.0, -1/fkv_,
              0.0, 0.0;

  // Generate the outer product matrix first
  Eigen::Matrix2d du_by_dh = last_image_centred_ * last_image_centred_.transpose();

  // this matrix is not yet du_by_dh, it is just
  // [ uc*uc  uc*vc ]
  // [ vc*uc  vc*vc ]
  // The trace of this matrix gives the magnitude of the vector
  const double radius2 = du_by_dh(0,0) + du_by_dh(1,1);

  // Calculate various constants to save typing
  double distor = 1 - 2 * kd1_ * radius2;
  double distor1_2 = sqrt(distor);
  double distor3_2 = distor1_2 * distor;

  // Now form the proper du_by_dh by manipulating the outer product matrix
  du_by_dh *= 2 * kd1_ / distor3_2;
  du_by_dh(0,0) += (1/distor1_2);
  du_by_dh(1,1) += (1/distor1_2);

  return dy_by_du * du_by_dh;
}

// Calculate the image position measurement noise at this location.
// @param h The image location
//
// This is not constant across the image. It has the value of m_measurement_sd at
// the centre, increasing with radial distance to 2*m_measurement_sd at the corners
Eigen::Matrix2d Camera::MeasurementNoise(const Eigen::Vector2d& h)
{
  // Distance of point we are considering from image centre
  const double distance = (h - centre_).norm();
  const double max_distance = centre_.norm();
  const double ratio = distance / max_distance; // goes from 0 to 1

  const double SD_image_filter_to_use = measurement_sd_ * (1.0 + ratio);

  const double measurement_noise_variance = SD_image_filter_to_use * SD_image_filter_to_use;

  // RiRES is diagonal
  Eigen::Matrix2d noise;

  noise.setIdentity();
  noise *= measurement_noise_variance;

  return noise;
}

} // namespace SceneLib2
