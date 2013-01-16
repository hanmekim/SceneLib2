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

#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Eigen>

namespace SceneLib2 {

class MotionModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MotionModel();
  ~MotionModel();


  // Extract cartesian part of position state vector
  void func_r(const Eigen::VectorXd &xp);
  // And Jacobian
  void func_dr_by_dxp(const Eigen::VectorXd &xp);
  // Extract quaternion part of position state vector
  void func_q(const Eigen::VectorXd &xp);

  // Redefined virtual functions
  void func_fv_and_dfv_by_dxv(const Eigen::VectorXd &xv, const Eigen::VectorXd &u,
                              const double delta_t);
  void func_Q(const Eigen::VectorXd &xv, const Eigen::VectorXd &u, const double delta_t);
  void func_xp(const Eigen::VectorXd &xv);
  void func_dxp_by_dxv(const Eigen::VectorXd &xv);
  void func_xvnorm_and_dxvnorm_by_dxv(const Eigen::VectorXd &xv);

  // Easy access to state blocks: fill matrices r, q, v, omega with
  // values based on state xv
  void extract_r_q_v_omega(const Eigen::VectorXd &xv, Eigen::Vector3d &r,
                           Eigen::Quaterniond &q, Eigen::Vector3d &v,
                           Eigen::Vector3d &omega);
  // The opposite: put r, q, v, omega back into vector xv
  void compose_xv(const Eigen::Vector3d &r, const Eigen::Quaterniond &q,
                  const Eigen::Vector3d &v, const Eigen::Vector3d &omega,
                  Eigen::VectorXd &xv);
  // Calculate commonly used Jacobian part dq(omega * delta_t) by domega
  void dqomegadt_by_domega(const Eigen::Vector3d &omega, const double delta_t,
                           Eigen::MatrixXd &dqomegadt_by_domega);

  // Ancillary functions: calculate parts of Jacobian dq_by_domega
  // which are repeatable due to symmetry.
  double dq0_by_domegaA(const double omegaA, const double omega, const double delta_t);
  double dqA_by_domegaA(const double omegaA, const double omega, const double delta_t);
  double dqA_by_domegaB(const double omegaA, const double omegaB, const double omega,
                        const double delta_t);

  Eigen::Matrix4d dqnorm_by_dq(const Eigen::Quaterniond &q);
  double dqi_by_dqi(double qi, double qq);
  double dqi_by_dqj(double qi, double qj, double qq);

  Eigen::VectorXd fvRES_;
  Eigen::VectorXd xpRES_;
  Eigen::VectorXd xvnormRES_;
  Eigen::MatrixXd dfv_by_dxvRES_;
  Eigen::MatrixXd QxRES_;
  Eigen::MatrixXd dxp_by_dxvRES_;
  Eigen::MatrixXd dxvnorm_by_dxvRES_;

  Eigen::Vector3d rRES_;
  Eigen::MatrixXd dr_by_dxpRES_;
  Eigen::Quaterniond  qRES_;

  const int  kPositionStateSize_;
  const int  kStateSize_;
  const int  kControlSize_;
  const double  kSdAComponentFilter_;
  const double  kSdAlphaComponentFilter_;
};

} // namespace SceneLib2

#endif // MOTION_MODEL_H
