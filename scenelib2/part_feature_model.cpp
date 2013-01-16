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

#include <iostream>

#include "part_feature_model.h"

#include "motion_model.h"
#include "camera.h"

namespace SceneLib2 {

PartFeatureModel::PartFeatureModel(int measurement_size, int feature_state_size, int graphics_state_size,
                                   Camera *camera, MotionModel *motion_model,
                                   int full_feature_state_size) :
  FeatureModel(measurement_size, feature_state_size, graphics_state_size, camera, motion_model),
  kFreeParameterSize_(1),
  kFullFeatureStateSize_(full_feature_state_size)
{
  riRES_.setZero();
  hhatiRES_.setZero();

  ypiRES_.resize(feature_state_size);
  ypiRES_.setZero();
  hpiRES_.resize(measurement_size);
  hpiRES_.setZero();
  yfiRES_.resize(kFullFeatureStateSize_);
  yfiRES_.setZero();
  dypi_by_dxpRES_.resize(feature_state_size,motion_model_->kPositionStateSize_);
  dypi_by_dxpRES_.setZero();
  dypi_by_dhiRES_.resize(feature_state_size,measurement_size);
  dypi_by_dhiRES_.setZero();
  dhpi_by_dxpRES_.resize(measurement_size,motion_model_->kPositionStateSize_);
  dhpi_by_dxpRES_.setZero();
  dhpi_by_dyiRES_.resize(measurement_size,feature_state_size);
  dhpi_by_dyiRES_.setZero();
  dyfi_by_dypiRES_.resize(kFullFeatureStateSize_,feature_state_size);
  dyfi_by_dypiRES_.setZero();
  dyfi_by_dlambdaRES_.resize(kFullFeatureStateSize_,kFreeParameterSize_);
  dyfi_by_dlambdaRES_.setZero();
}

PartFeatureModel::~PartFeatureModel()
{

}

void PartFeatureModel::func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
    const Eigen::VectorXd &yi, const Eigen::VectorXd &xp)
{
  // Extract cartesian and quaternion components of xp
  motion_model_->func_r(xp);
  motion_model_->func_q(xp);

  // Extract ri and hhati components of yi = ypi
  func_ri(yi);
  func_hhati(yi);

  // ri part: transformation is just the same as in the normal point case
  // zeroedri = RRW(rWi - rW)

  // ri - r
  Eigen::Vector3d yWiminusrW = riRES_ - motion_model_->rRES_;

  Eigen::Quaterniond qRW = motion_model_->qRES_.inverse();
  Eigen::Matrix4d dqRW_by_dq = dqbar_by_dq();

  // Rotation RRW
  Eigen::Matrix3d RRW = qRW.toRotationMatrix();

  // RRW(rWi - rW)
  Eigen::Vector3d zeroedri = RRW * yWiminusrW;

  // Now calculate Jacobians
  // dzeroedri_by_dri is RRW
  // dzeroedri_by_dhhati = 0
  Eigen::Matrix3d dzeroedri_by_dri = RRW;

  // dzeroedyi_by_dxp:
  // dzeroedri_by_dr = -RRW
  // dzeroedri_by_dq = d_dq(RRW (ri - r))
  Eigen::Matrix3d dzeroedri_by_dr = RRW * -1.0;

  Eigen::Matrix<double, 3, 4> dzeroedri_by_dqRW = dRq_times_a_by_dq(qRW, yWiminusrW);
  Eigen::Matrix<double, 3, 4> dzeroedri_by_dq = dzeroedri_by_dqRW * dqRW_by_dq;

  // Now for the hhati part (easier...)
  // zeroedhhati = RRW hhati
  Eigen::Vector3d zeroedhhati = RRW * hhatiRES_;

  // Jacobians
  // dzeroedhhati_by_dr = 0
  // dzeroedhhati_by_dq = d_dq(RRW hhati)
  // dzeroedhhati_by_dhhati = RRW
  // dzeroedhhati_by_dri = 0
  Eigen::Matrix<double, 3, 4> dzeroedhhati_by_dqRW = dRq_times_a_by_dq(qRW, hhatiRES_);
  Eigen::Matrix<double, 3, 4> dzeroedhhati_by_dq = dzeroedhhati_by_dqRW * dqRW_by_dq;
  Eigen::Matrix<double, 3, 3> dzeroedhhati_by_dhhati = RRW;

  // And put it all together
  zeroedyiRES_ << zeroedri, zeroedhhati;

  dzeroedyi_by_dxpRES_.setZero();
  dzeroedyi_by_dxpRES_.block(0, 0, 3, 3) = dzeroedri_by_dr;
  dzeroedyi_by_dxpRES_.block(0, 3, 3, 4) = dzeroedri_by_dq;
  dzeroedyi_by_dxpRES_.block(3, 3, 3, 4) = dzeroedhhati_by_dq;

  dzeroedyi_by_dyiRES_.setZero();
  dzeroedyi_by_dyiRES_.block(0, 0, 3, 3) = dzeroedri_by_dri;
  dzeroedyi_by_dyiRES_.block(3, 3, 3, 3) = dzeroedhhati_by_dhhati;
}

int PartFeatureModel::visibility_test(const Eigen::VectorXd &xp,
                                      const Eigen::VectorXd &yi,
                                      const Eigen::VectorXd &xp_orig,
                                      const Eigen::VectorXd &hi)
{
  // Always visible for now
  return  0;
}

double PartFeatureModel::selection_score(const Eigen::MatrixXd &Si)
{
  // Always measureable for now
  return  100000.0;
}

// Redefined virtual functions from
// Partially_Initialised_Fully_Initialised_Feature_Measurement_Model
void PartFeatureModel::func_ypi_and_dypi_by_dxp_and_dypi_by_dhi_and_Ri(
    const Eigen::VectorXd &hi, const Eigen::VectorXd &xp)
{
  // Representation of a line here:
  // ypi = (rWi    )
  //       (hLhatWi)

  // Form the ray (in camera co-ordinates by unprojecting this image location
  Eigen::Vector3d hLRi = camera_->Unproject(hi);

  // Form hLhatRi from hLRi
  // Normalise
  Eigen::Vector3d hLhatRi = hLRi;
  hLhatRi.normalize();

  Eigen::Matrix<double, 3, 3> dhLhatRi_by_dhLRi = dvnorm_by_dv(hLRi);

  // Now convert this into a direction in world co-ordinates by rotating
  // Form hLhatWi from hLhatRi
  // Rotate
  motion_model_->func_q(xp);
  Eigen::Matrix3d RWR = motion_model_->qRES_.toRotationMatrix();
  Eigen::Vector3d hLhatWi(RWR * hLhatRi);

  // Extract rW from xp
  motion_model_->func_r(xp);

  // And form ypiRES
  ypiRES_ << motion_model_->rRES_(0), motion_model_->rRES_(1), motion_model_->rRES_(2),
             hLhatWi(0), hLhatWi(1), hLhatWi(2);

  // Form Jacobians dypi_by_dxp and dypi_by_dhi

  // dypi_by_dxp = (drWi_by_dr     drWi_by_dq    )
  //               (dhLhatWi_by_dr dhLhatWi_by_dq)
  //             = (I              0             )
  //               (0              dhLhatWi_by_dq)

  // hLhatWi = RWR * hLhatRi
  // => dhLhatWi_by_dq = d/dq ( R(qWR) * hLhatRi)

  Eigen::Matrix<double, 3, 4> dhLhatWi_by_dq = dRq_times_a_by_dq(motion_model_->qRES_, hLhatRi);

  // Put dypi_by_dxp together
  dypi_by_dxpRES_.setZero();
  dypi_by_dxpRES_(0,0) = 1.0;
  dypi_by_dxpRES_(1,1) = 1.0;
  dypi_by_dxpRES_(2,2) = 1.0;
  dypi_by_dxpRES_.block(3,3,3,4) = dhLhatWi_by_dq;

  // dypi_by_dhi = (drWi_by_dhi    )
  //               (dhLhatWi_by_dhi)
  //             = (0              )
  //               (dhLhatWi_by_dhi)

  // hLhatWi = RWR * hLhatRi
  // Need to work out derivative for this
  // dhLhatWi_by_dhi = RWR * dhLhatRi_by_dhLRi * dhLRi_by_dhi

  Eigen::MatrixXd dhLhatWi_by_dhi(3,2);
  dhLhatWi_by_dhi = RWR * dhLhatRi_by_dhLRi * camera_->UnprojectionJacobian();

  dypi_by_dhiRES_.setZero();
  dypi_by_dhiRES_.block(3,0,3,2) = dhLhatWi_by_dhi;

  // And construct Ri
  func_Ri(hi);
}

void PartFeatureModel::func_hpi_and_dhpi_by_dxp_and_dhpi_by_dyi(
    const Eigen::VectorXd &yi, const Eigen::VectorXd &xp, const Eigen::VectorXd &lambda)
{
  // This function gives relative position of feature: also call this hR
  // (vector from camera to feature in robot frame)
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  // Parameters of vector hLR from camera to feature in robot frame
  // hLR = zeroedri + lambda * zeroedhhati
  // Calculate the vector from the camera to the feature given the current
  // lambda
  Eigen::Vector3d hLR = Eigen::Vector3d(zeroedyiRES_(0),zeroedyiRES_(1),zeroedyiRES_(2)) +
                        lambda(0) *
                        Eigen::Vector3d(zeroedyiRES_(3),zeroedyiRES_(4),zeroedyiRES_(5));

  // Project this into the image
  hpiRES_ = camera_->Project(hLR);

  // What is the Jacobian of this projection?
  Eigen::MatrixXd dhpi_by_dhLRi(2,3);
  dhpi_by_dhLRi = camera_->ProjectionJacobian();

  // Calculate the required result Jacobians

  // Now how the vector to the feature depends on the parameterised line
  // (this is a function of lambda)
  Eigen::Matrix<double, 3, 6> dhLRi_by_dzeroedyi;

  dhLRi_by_dzeroedyi << 1.0, 0.0, 0.0, lambda(0), 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, lambda(0), 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0, lambda(0);

  dhpi_by_dxpRES_ = dhpi_by_dhLRi * dhLRi_by_dzeroedyi * dzeroedyi_by_dxpRES_;
  dhpi_by_dyiRES_ = dhpi_by_dhLRi * dhLRi_by_dzeroedyi * dzeroedyi_by_dyiRES_;
}

void PartFeatureModel::func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda(
    const Eigen::VectorXd &ypi, const Eigen::VectorXd &lambda)
{
  // Simple: lambda is a scalar
  // yfi = ri + lambda hhati

  func_ri(ypi);
  func_hhati(ypi);

  yfiRES_ << riRES_(0) + lambda(0) * hhatiRES_(0),
             riRES_(1) + lambda(0) * hhatiRES_(1),
             riRES_(2) + lambda(0) * hhatiRES_(2);

  dyfi_by_dypiRES_ << 1.0, 0.0, 0.0, lambda(0), 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0, lambda(0), 0.0,
                      0.0, 0.0, 1.0, 0.0, 0.0, lambda(0);

  dyfi_by_dlambdaRES_(0,0) = hhatiRES_(0);
  dyfi_by_dlambdaRES_(1,0) = hhatiRES_(1);
  dyfi_by_dlambdaRES_(2,0) = hhatiRES_(2);
}

// Special functions: extract r and hhat parts of line
void PartFeatureModel::func_ri(const Eigen::VectorXd &ypi)
{
  riRES_ << ypi(0), ypi(1), ypi(2);
}

void PartFeatureModel::func_hhati(const Eigen::VectorXd &ypi)
{
  hhatiRES_ << ypi(3), ypi(4), ypi(5);
}

Eigen::Matrix3d PartFeatureModel::dvnorm_by_dv(Eigen::Vector3d v)
{
  Eigen::Matrix3d M;

  double vv = v(0)*v(0) + v(1)*v(1) + v(2)*v(2);

  // We can use dqi_by_dqi and dqi_by_dqj functions because they are the same
  M << dqi_by_dqi(v(0), vv),
       dqi_by_dqj(v(0), v(1), vv),
       dqi_by_dqj(v(0), v(2), vv),

       dqi_by_dqj(v(1), v(0), vv),
       dqi_by_dqi(v(1), vv),
       dqi_by_dqj(v(1), v(2), vv),

       dqi_by_dqj(v(2), v(0), vv),
       dqi_by_dqj(v(2), v(1), vv),
       dqi_by_dqi(v(2), vv);

  return M;
}

// Auxiliary functions used by dqnorm_by_dq()
// Value of diagonal element of Jacobian
double PartFeatureModel::dqi_by_dqi(double qi, double qq)
{
  return (1 - qi*qi / (qq*qq)) / qq;
}

// Value of off-diagonal element of Jacobian
double PartFeatureModel::dqi_by_dqj(double qi, double qj, double qq)
{
  return -qi * qj / (qq*qq*qq);
}

} // namespace SceneLib2
