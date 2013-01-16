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

#include "full_feature_model.h"

#include "motion_model.h"
#include "camera.h"

#include <iostream>

namespace SceneLib2 {

FullFeatureModel::FullFeatureModel(int measurement_size, int feature_state_size,
                                   int graphics_state_size, Camera *camera,
                                   MotionModel *motion_model) :
  FeatureModel(measurement_size, feature_state_size, graphics_state_size, camera, motion_model),
  kMaximumLengthRatio_(2.0),
  kMaximumAngleDifference_(M_PI * 45.0 / 180.0),
  kImageSearchBoundary_(20.0)
{
  hiRES_.resize(measurement_size);
  hiRES_.setZero();
  nuiRES_.resize(measurement_size);
  nuiRES_.setZero();
  dhi_by_dxpRES_.resize(measurement_size,motion_model->kPositionStateSize_);
  dhi_by_dxpRES_.setZero();
  dhi_by_dyiRES_.resize(measurement_size,feature_state_size);
  dhi_by_dyiRES_.setZero();
}

FullFeatureModel::~FullFeatureModel()
{
}

void FullFeatureModel::func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
    const Eigen::VectorXd &yi, const Eigen::VectorXd &xp)
{
  // Extract cartesian and quaternion components of xp
  motion_model_->func_r(xp);
  motion_model_->func_q(xp);

  Eigen::Vector3d yWiminusrW = yi - motion_model_->rRES_;

  Eigen::Quaterniond qRW = motion_model_->qRES_.inverse();
  Eigen::Matrix4d dqRW_by_dq = dqbar_by_dq();

  // Rotation RRW
  Eigen::Matrix3d RRW = qRW.toRotationMatrix();

  // Position of feature relative to robot in robot frame
  Eigen::Vector3d zeroedyi = RRW * yWiminusrW;
  zeroedyiRES_ = zeroedyi;

  // Now calculate Jacobians
  // dzeroedyi_by_dyi is RRW
  dzeroedyi_by_dyiRES_ = RRW;

  // dzeroedyi_by_dxp has 2 partitions:
  // dzeroedyi_by_dr (3 * 3)
  // dzeroedyi_by_dq (3 * 4)
  Eigen::Matrix3d dzeroedyi_by_dr = RRW;
  dzeroedyi_by_dr *= -1.0;

  Eigen::Matrix<double, 3, 4> dzeroedyi_by_dqRW = dRq_times_a_by_dq(qRW, yWiminusrW);
  Eigen::Matrix<double, 3, 4> dzeroedyi_by_dq = dzeroedyi_by_dqRW * dqRW_by_dq;

  dzeroedyi_by_dxpRES_.block(0, 0, 3, 3) = dzeroedyi_by_dr;
  dzeroedyi_by_dxpRES_.block(0, 3, 3, 4) = dzeroedyi_by_dq;
}

int FullFeatureModel::visibility_test(const Eigen::VectorXd &xp,
                                      const Eigen::VectorXd &yi,
                                      const Eigen::VectorXd &xp_orig,
                                      const Eigen::VectorXd &hi)
{
  int cant_see_flag = 0;

  // Test image boundaries
  if (hi(0) < 0.0 + kImageSearchBoundary_ ||
      hi(0) > (double)(camera_->width_ - 1 - kImageSearchBoundary_)) {
    cant_see_flag |= kLeftRightFail_;
  }
  if (hi(1) < 0.0 + kImageSearchBoundary_ ||
      hi(1) > (double)(camera_->height_ - 1 - kImageSearchBoundary_)) {
    cant_see_flag |= kUpDownFail_;
  }

  // Do tests on length and angle of predicted view

  // hLWi is current predicted vector from head to feature in
  // world frame

  // This function gives relative position of feature
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  // Test the feature's not behind the camera (because projection
  // may do strange things)
  if (zeroedyiRES_(2) <= 0) {
    cant_see_flag |= kBehindCameraFail_;
  }

  motion_model_->func_q(xp);
  Eigen::Matrix3d RWR = motion_model_->qRES_.toRotationMatrix();

  Eigen::Vector3d hLWi = RWR * Eigen::Vector3d(zeroedyiRES_);

  // hLWi_orig is vector from head to feature in world frame
  // WHEN THAT FEATURE WAS FIRST MEASURED: i.e. when the image
  // patch was saved
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp_orig);

  motion_model_->func_q(xp_orig);
  Eigen::Matrix3d RWR_orig = motion_model_->qRES_.toRotationMatrix();

  Eigen::Vector3d hLWi_orig = RWR_orig * Eigen::Vector3d(zeroedyiRES_);

  // Compare hLWi and hLWi_orig for length and orientation
  double mod_hLWi = hLWi.norm();
  double mod_hLWi_orig = hLWi_orig.norm();

  double length_ratio = mod_hLWi / mod_hLWi_orig;

  if (length_ratio > kMaximumLengthRatio_ ||
      length_ratio < (1.0 / kMaximumLengthRatio_)) {
    cant_see_flag |= kDistanceFail_;
  }

  double dot_prod = hLWi.dot(hLWi_orig);

  double angle = acos(dot_prod / (mod_hLWi * mod_hLWi_orig));
  angle = (angle >= 0.0 ? angle : -angle);  // Make angle positive

  if (angle > kMaximumAngleDifference_) {
    cant_see_flag |= kAngleFail_;
  }

  return cant_see_flag;   // 0 if OK, otherwise error code
}

double FullFeatureModel::selection_score(const Eigen::MatrixXd &Si)
{
  // Return the trace of the innovation covariance
  return Si.trace();
}

void FullFeatureModel::func_hi_and_dhi_by_dxp_and_dhi_by_dyi(const Eigen::VectorXd &yi,
                                                             const Eigen::VectorXd &xp)
{
  // This function gives relative position of feature: also call this hR
  // (vector from camera to feature in robot frame)
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  // Project this into the image using our camera
  hiRES_ = camera_->Project(zeroedyiRES_);

  // And ask the camera what the Jacobian of this projection was
  Eigen::MatrixXd dhid_by_dzeroedyi(2,3);
  dhid_by_dzeroedyi = camera_->ProjectionJacobian();

  // Form the required Jacobians
  dhi_by_dxpRES_ = dhid_by_dzeroedyi * dzeroedyi_by_dxpRES_;
  dhi_by_dyiRES_ = dhid_by_dzeroedyi * dzeroedyi_by_dyiRES_;
}

void FullFeatureModel::func_nui(const Eigen::VectorXd &hi, const Eigen::VectorXd &zi)
{
  nuiRES_ = zi - hi;
}

} // namespace SceneLib2
