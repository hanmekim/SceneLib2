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

#ifndef FULL_FEATURE_MODEL_H
#define FULL_FEATURE_MODEL_H

#include "feature_model.h"

namespace SceneLib2 {

using namespace std;

class FullFeatureModel : public FeatureModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FullFeatureModel(int measurement_size, int feature_state_size, int graphics_state_size,
                   Camera *camera, MotionModel *motion_model);
  ~FullFeatureModel();

  void func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(const Eigen::VectorXd &yi, const Eigen::VectorXd &xp);

  int visibility_test(const Eigen::VectorXd &xp, const Eigen::VectorXd &yi, const Eigen::VectorXd &xp_orig, const Eigen::VectorXd &hi);
  double selection_score(const Eigen::MatrixXd &Si);

  // Redefined virtual functions from
  // Fully_Initialised_Feature_Measurement_Model
  void func_hi_and_dhi_by_dxp_and_dhi_by_dyi(const Eigen::VectorXd &yi, const Eigen::VectorXd &xp);
  void func_nui(const Eigen::VectorXd &hi, const Eigen::VectorXd &zi);

  Eigen::VectorXd hiRES_;
  Eigen::VectorXd nuiRES_;
  Eigen::MatrixXd dhi_by_dxpRES_;
  Eigen::MatrixXd dhi_by_dyiRES_;

  const double kMaximumLengthRatio_;
  const double kMaximumAngleDifference_;
  const double kImageSearchBoundary_;

  // The return flag from visibility_test() ORs these together
  // to indicate the reasons for failure.
  enum visibility{kLeftRightFail_ = 1,
                  kUpDownFail_ = 2,
                  kDistanceFail_ = 4,
                  kAngleFail_ = 8,
                  kBehindCameraFail_ = 16};
};

} // namespace SceneLib2

#endif // FULL_FEATURE_MODEL_H
