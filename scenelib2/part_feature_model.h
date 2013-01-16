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

#ifndef PART_FEATURE_MODEL_H
#define PART_FEATURE_MODEL_H

#include "feature_model.h"

namespace SceneLib2 {

using namespace std;

class PartFeatureModel : public FeatureModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PartFeatureModel(int measurement_size, int feature_state_size, int graphics_state_size,
                   Camera *camera, MotionModel *motion_model,
                   int full_feature_state_size);
  ~PartFeatureModel();

  void func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(const Eigen::VectorXd &yi, const Eigen::VectorXd &xp);
  int visibility_test(const Eigen::VectorXd &xp, const Eigen::VectorXd &yi, const Eigen::VectorXd &xp_orig, const Eigen::VectorXd &hi);

  double selection_score(const Eigen::MatrixXd &Si);

  // Redefined virtual functions from
  // Partially_Initialised_Fully_Initialised_Feature_Measurement_Model
  void func_ypi_and_dypi_by_dxp_and_dypi_by_dhi_and_Ri(const Eigen::VectorXd &hi, const Eigen::VectorXd &xp);
  void func_hpi_and_dhpi_by_dxp_and_dhpi_by_dyi(const Eigen::VectorXd &yi, const Eigen::VectorXd &xp, const Eigen::VectorXd &lambda);
  void func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda(const Eigen::VectorXd &ypi, const Eigen::VectorXd &lambda);

  // Special functions: extract r and hhat parts of line
  void func_ri(const Eigen::VectorXd &ypi);
  void func_hhati(const Eigen::VectorXd &ypi);

  Eigen::Matrix3d dvnorm_by_dv(Eigen::Vector3d v);
  double dqi_by_dqi(double qi, double qq);
  double dqi_by_dqj(double qi, double qj, double qq);

  Eigen::Vector3d riRES_;
  Eigen::Vector3d hhatiRES_;

  Eigen::VectorXd ypiRES_;
  Eigen::VectorXd hpiRES_;
  Eigen::VectorXd yfiRES_;
  Eigen::MatrixXd dypi_by_dxpRES_;
  Eigen::MatrixXd dypi_by_dhiRES_;
  Eigen::MatrixXd dhpi_by_dxpRES_;
  Eigen::MatrixXd dhpi_by_dyiRES_;
  Eigen::MatrixXd dyfi_by_dypiRES_;
  Eigen::MatrixXd dyfi_by_dlambdaRES_;

  const int  kFreeParameterSize_;
  const int  kFullFeatureStateSize_;
};

} // namespace SceneLib2

#endif // PART_FEATURE_MODEL_H
