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

#ifndef FEATURE_H
#define FEATURE_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "camera.h"
#include "feature_model.h"

namespace SceneLib2 {

using namespace std;

class MonoSLAM;

// A class to manage information about a feature. This stores the feature's
// Identifier, its state and covariances and its current measurement and
// prediction. This class is used whether the feature is fully- or
// partially-initialised. This class also knows how to convert from a partially-
// to a fully-initialised feature.
class Feature {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor for partially-initialised features.
  Feature(cv::Mat patch, const Eigen::VectorXd &h, MonoSLAM *monoslam);

  // Constructor for known features. The different number of arguments
  // differentiates it from the constructor for partially-initialised features
  Feature(FeatureModel *feature_model,
          const Eigen::VectorXd &y, const Eigen::VectorXd &xp,
          const int label, const int total_state_size, const string &identifier,
          vector<Feature *> feature_list);

  ~Feature();

  void Initialise();

  void convert_from_partially_to_fully_initialised(const Eigen::VectorXd &lambda,
                                                   const Eigen::MatrixXd &Plambda,
                                                   MonoSLAM *monoslam);

  // The estimate of the feature state yi. (e.g. the feature's 3D position)
  Eigen::VectorXd y_;
  // The robot position xp state from which we initialised this feature
  Eigen::VectorXd xp_org_;

  // The covariance of this feature \f$ P_{y_iy_i} \f$
  Eigen::MatrixXd Pyy_;
  // The covariance between the robot state and this feature's state \f$ P_{x_vy_i} \f$
  Eigen::MatrixXd Pxy_;

  // The identifier for this feature. Identifier is a pointer to something in
  // the real world which identifies this feature uniquely.
  cv::Mat patch_;

  // The covariances between this feature and others.
  vector<Eigen::MatrixXd> matrix_block_list_;

  // Places to store predicted (h) and actual (z) measurements and Jacobians
  // The predicted feature measurement \f$ h_i \f$ state.
  Eigen::VectorXd h_;
  // The actual feature measurement state \f$ z_i \f$.
  Eigen::VectorXd z_;
  // The feature innovation \f$ \nu_i = z_i - h_i \f$.
  Eigen::VectorXd nu_; // Innovation
  // The Jacobian between the predicted measurement and the vehicle state
  // \f$ \frac{\partial h_i}{\partial x_v} \f$.
  Eigen::MatrixXd dh_by_dxv_;
  // The Jacobian between the feature measurement and its state
  // \f$ \frac{\partial h_i}{\partial y_i} \f$. */
  Eigen::MatrixXd dh_by_dy_;
  // The innovation covariance \f$ R_i \f$
  Eigen::MatrixXd R_;
  // The feature measurement covariance \f$ S_i \f$
  Eigen::MatrixXd S_;

  // The label for this feature within scene class.
  int label_;

  // The current position of this feature in the list of features.
  int position_in_list_;
  // With general feature state sizes, need this to identify the starting
  // position of this feature in total state vector.
  int position_in_total_state_vector_;
  // The number of times that a measurement of this feature has been
  // attempted. Used together with successful_measurements_of_feature to
  // determine whether the feature is a bad one that should be deleted.
  int attempted_measurements_of_feature_;
  // The number of times that this feature has been successfully measured. Used
  // together with attempted_measurements_of_feature to determine whether the
  // feature is a bad one that should be deleted.
  int successful_measurements_of_feature_;

  // Important bookkeeping data
  // Is this feature currently selected?
  bool selected_flag_;
  // Set if we want shortly to delete this feature but can't do it right now
  bool scheduled_for_termination_flag_;
  // Keep track of the attempted and successful measurements made of a
  // feature over time
  // Was the last measurement of this feature successful?
  bool successful_measurement_flag_;

  bool fully_initialised_flag_;

  FeatureModel  *feature_model_;
};

} // namespace SceneLib2

#endif // FEATURE_H
