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

#include "feature_model.h"

namespace SceneLib2 {

FeatureModel::FeatureModel(int measurement_size, int feature_state_size,
                           int graphics_state_size, Camera *camera, MotionModel *motion_model) :
  kMeasurementSize_(measurement_size),
  kFeatureStateSize_(feature_state_size),
  kGraphicsStateSize_(graphics_state_size)
{
  camera_ = camera;
  motion_model_ = motion_model;

  yigraphicsRES_.resize(kGraphicsStateSize_);
  yigraphicsRES_.setZero();
  zeroedyigraphicsRES_.resize(kGraphicsStateSize_);
  zeroedyigraphicsRES_.setZero();
  zeroedyiRES_.resize(kFeatureStateSize_);
  zeroedyiRES_.setZero();
  PyiyigraphicsRES_.resize(kGraphicsStateSize_,kGraphicsStateSize_);
  PyiyigraphicsRES_.setZero();
  PzeroedyigraphicsRES_.resize(kGraphicsStateSize_,kGraphicsStateSize_);
  PzeroedyigraphicsRES_.setZero();
  RiRES_.resize(kMeasurementSize_,kMeasurementSize_);
  RiRES_.setZero();
  SiRES_.resize(kMeasurementSize_,kMeasurementSize_);
  SiRES_.setZero();
  dzeroedyi_by_dxpRES_.resize(kFeatureStateSize_,motion_model_->kPositionStateSize_);
  dzeroedyi_by_dxpRES_.setZero();
  dzeroedyi_by_dyiRES_.resize(kFeatureStateSize_,kFeatureStateSize_);
  dzeroedyi_by_dyiRES_.setZero();
}

FeatureModel::~FeatureModel()
{
}

// Calculate the innovation covariance. This is the overall measurement
// uncertainty in the feature, a combination of the uncertainty in the robot
// state, the feature state, and the measurement uncertainty. This calculation is
// generic to all feature measurement models, but could be over-ridden if necessary
// (e.g. for an efficient implementation). The innovation covariance is given by
// \f[
// S_i = \frac{\partial h_i}{\partial x_v} P_{xx}
//         \frac{\partial h_i}{\partial x_v}^T
//   + \frac{\partial h_i}{\partial x_v} P_{xy_i}
//         \frac{\partial h_i}{\partial y_i}^T
//   + \frac{\partial h_i}{\partial y_i} P_{y_i x}
//         \frac{\partial h_i}{\partial x_v}^T
//   + \frac{\partial h_i}{\partial y_i} P_{y_i y_i}
//         \frac{\partial h_i}{\partial y_i}^T
//   + R_i
// \f]
// where \f$ R_i \f$ is the noise covariance of measurements (usually assumed to
// be diagonal with magnitude determined by the image resolution).
// @param Pxyi The covariance \f$ P_{xy_i} \f$ between the robot state
//   \f$ x_v \f$ and the feature state \f$ y_i \f$.
// @param Pyiyi The covariance of the feature, \f$ P_{y_iy_i} \f$
// @param dhi_by_dxv The Jacobian \f$ \frac{\partial h_i}{\partial x_v} \f$ between
// the feature measurement \f$ h_i \f$ and the robot state \f$ x_v \f$
// @param dhi_by_dyi The Jacobian \f$ \frac{\partial h_i}{\partial y_i} \f$ between
// the feature measurement \f$ h_i \f$ and the feature state \f$ y_i \f$
// @param Ri The innovation covariance \f$ R_i \f$
void FeatureModel::func_Si(const Eigen::MatrixXd &Pxx, const Eigen::MatrixXd &Pxyi,
                           const Eigen::MatrixXd &Pyiyi, const Eigen::MatrixXd &dhi_by_dxv,
                           const Eigen::MatrixXd &dhi_by_dyi, const Eigen::MatrixXd &Ri)
{
  // Zero SiRES and add bits on
  SiRES_.setZero();

  SiRES_ += dhi_by_dxv * Pxx * dhi_by_dxv.transpose();

  Eigen::MatrixXd Temp_MM1 = dhi_by_dxv * Pxyi * dhi_by_dyi.transpose();
  SiRES_ += Temp_MM1;

  SiRES_ += Temp_MM1.transpose();

  SiRES_ += dhi_by_dyi * Pyiyi * dhi_by_dyi.transpose();

  SiRES_ += Ri;
}

// In this case the graphics representation \f$ y_i^{graphics} \f$ and its
// covariance is the same as the feature state \f$ y_i \f$ and covariance.
void FeatureModel::func_yigraphics_and_Pyiyigraphics(const Eigen::VectorXd &yi,
                                                     const Eigen::MatrixXd &Pyiyi)
{
  // The graphics representation is the same as the state
  yigraphicsRES_ = yi;
  PyiyigraphicsRES_ = Pyiyi;
}

void FeatureModel::func_zeroedyigraphics_and_Pzeroedyigraphics(
    const Eigen::VectorXd &yi, const Eigen::VectorXd &xv, const Eigen::MatrixXd &Pxx,
    const Eigen::MatrixXd &Pxyi, const Eigen::MatrixXd &Pyiyi)
{
  motion_model_->func_xp(xv);

  // In this case (where the feature state is the same as the graphics
  // state) zeroedyigraphics is the same as zeroedyi
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, motion_model_->xpRES_);
  zeroedyigraphicsRES_ = zeroedyiRES_;

  Eigen::MatrixXd dzeroedyigraphics_by_dxv = dzeroedyi_by_dxpRES_ * motion_model_->dxp_by_dxvRES_;

  PzeroedyigraphicsRES_ = dzeroedyigraphics_by_dxv * Pxx * dzeroedyigraphics_by_dxv.transpose() +
                          dzeroedyi_by_dyiRES_ * Pxyi.transpose() * dzeroedyigraphics_by_dxv.transpose() +
                          dzeroedyigraphics_by_dxv * Pxyi * dzeroedyi_by_dyiRES_.transpose() +
                          dzeroedyi_by_dyiRES_ * Pyiyi * dzeroedyi_by_dyiRES_.transpose();
}

void FeatureModel::func_Ri(const Eigen::VectorXd &hi)
{
  RiRES_ = camera_->MeasurementNoise(hi);
}

Eigen::Matrix4d FeatureModel::dqbar_by_dq()
{
  Eigen::Matrix4d M;

  M << 1.0,  0.0,  0.0,  0.0,
       0.0, -1.0,  0.0,  0.0,
       0.0,  0.0, -1.0,  0.0,
       0.0,  0.0,  0.0, -1.0;

  return M;
}

Eigen::MatrixXd FeatureModel::dRq_times_a_by_dq (const Eigen::Quaterniond &q, const Eigen::Vector3d &a)
{
  Eigen::MatrixXd aMat(3,1);

  aMat(0,0) = a(0);
  aMat(1,0) = a(1);
  aMat(2,0) = a(2);

  Eigen::Matrix3d TempR;
  Eigen::MatrixXd Temp31(3,1);
  Eigen::MatrixXd dRq_times_a_by_dq(3,4);

  // Make Jacobian by stacking four vectors together side by side
  TempR = dR_by_dq0(q);
  Temp31 = TempR * aMat;
  dRq_times_a_by_dq.block(0,0,3,1) = Temp31;

  TempR = dR_by_dqx(q);
  Temp31 = TempR * aMat;
  dRq_times_a_by_dq.block(0,1,3,1) = Temp31;

  TempR = dR_by_dqy(q);
  Temp31 = TempR * aMat;
  dRq_times_a_by_dq.block(0,2,3,1) = Temp31;

  TempR = dR_by_dqz(q);
  Temp31 = TempR * aMat;
  dRq_times_a_by_dq.block(0,3,3,1) = Temp31;

  return dRq_times_a_by_dq;
}

Eigen::Matrix3d FeatureModel::dR_by_dq0(const Eigen::Quaterniond &q)
{
  Eigen::Matrix3d M;

  M << 2*q.w(), -2*q.z(),  2*q.y(),
       2*q.z(),  2*q.w(), -2*q.x(),
      -2*q.y(),  2*q.x(),  2*q.w();

  return  M;
}

Eigen::Matrix3d FeatureModel::dR_by_dqx(const Eigen::Quaterniond &q)
{
  Eigen::Matrix3d M;

  M << 2*q.x(), 2*q.y(), 2*q.z(),
       2*q.y(), -2*q.x(), -2*q.w(),
       2*q.z(), 2*q.w(), -2*q.x();

  return  M;
}

Eigen::Matrix3d FeatureModel::dR_by_dqy(const Eigen::Quaterniond &q)
{
  Eigen::Matrix3d M;

  M << -2*q.y(), 2*q.x(), 2*q.w(),
        2*q.x(), 2*q.y(), 2*q.z(),
       -2*q.w(), 2*q.z(), -2*q.y();

  return  M;
}

Eigen::Matrix3d FeatureModel::dR_by_dqz(const Eigen::Quaterniond &q)
{
  Eigen::Matrix3d M;

  M << -2*q.z(), -2*q.w(), 2*q.x(),
        2*q.w(), -2*q.z(), 2*q.y(),
        2*q.x(), 2*q.y(), 2*q.z();

  return  M;
}

} // namespace SceneLib2
