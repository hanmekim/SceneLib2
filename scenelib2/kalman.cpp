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

#include "kalman.h"

namespace SceneLib2 {

Kalman::Kalman()
{

}

Kalman::~Kalman()
{

}

void Kalman::KalmanFilterPredict(MonoSLAM *monoslam, Eigen::Vector3d &u)
{
  // Make model calculations: results are stored in RES matrices
  monoslam->motion_model_->func_fv_and_dfv_by_dxv(monoslam->xv_, u,
                                                  monoslam->kDeltaT_);
  monoslam->motion_model_->func_Q(monoslam->xv_, u, monoslam->kDeltaT_);

  monoslam->xv_ = monoslam->motion_model_->fvRES_;

  monoslam->Pxx_ = monoslam->motion_model_->dfv_by_dxvRES_ *
                   monoslam->Pxx_ *
                   monoslam->motion_model_->dfv_by_dxvRES_.transpose() +
                   monoslam->motion_model_->QxRES_;

  // Change the covariances between vehicle state and feature states
  for (vector<Feature *>::iterator it = monoslam->feature_list_.begin();
       it != monoslam->feature_list_.end(); ++it) {
    (*it)->Pxy_ = monoslam->motion_model_->dfv_by_dxvRES_ * (*it)->Pxy_;
  }
}

// Update the filter in a simple overall way
void Kalman::KalmanFilterUpdate(MonoSLAM *monoslam)
{
  // Steps to update the total filter:
  //   1. Form h and dh_by_dx and R(k+1) and z
  //   2. Calculate S(k+1)
  //   3. Calculate W(k+1)
  //   4. Calculate x(k+1|k+1)
  //   5. Calculate P(k+1|k+1)
  int size = monoslam->successful_measurement_vector_size_;
  int size2 = monoslam->total_state_size_;

  Eigen::VectorXd x(size2);
  Eigen::MatrixXd P(size2, size2);

  x.setZero();
  monoslam->construct_total_state(x);

  P.setZero();
  monoslam->construct_total_covariance(P);

  // 1. Form nu and dh_by_dx
  Eigen::VectorXd nu_tot(size);
  Eigen::MatrixXd dh_by_dx_tot(size, size2);
  Eigen::MatrixXd R_tot(size, size);

  monoslam->construct_total_measurement_stuff(nu_tot, dh_by_dx_tot, R_tot);

  // 2. Calculate S(k+1)
  Eigen::MatrixXd S = dh_by_dx_tot * P * dh_by_dx_tot.transpose();
  S += R_tot;

  // 3. Calculate W(k+1)
  Eigen::LLT<Eigen::MatrixXd>  S_cholesky(S);
  Eigen::MatrixXd S_L = S_cholesky.matrixL();
  Eigen::MatrixXd S_Linv = S_L.inverse();
  Eigen::MatrixXd Sinv = S_Linv.transpose() * S_Linv;

  Eigen::MatrixXd W = P * dh_by_dx_tot.transpose() * Sinv;

  // 4. Calculate x(k+1|k+1)
  x += W * nu_tot;

  // 5. Calculate P(k+1|k+1)
  P -= W * S * W.transpose();

  monoslam->fill_states(x);
  monoslam->fill_covariances(P);
}

} // namespace SceneLib2
