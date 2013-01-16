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

#ifndef MONOSLAM_H
#define MONOSLAM_H

#include <vector>
#include <Eigen/Eigen>

#include "framegrabber/framegrabber.h"
#include "graphic/graphictool.h"
#include "camera.h"
#include "motion_model.h"
#include "feature_model.h"
#include "full_feature_model.h"
#include "part_feature_model.h"
#include "feature.h"
#include "feature_init_info.h"

namespace SceneLib2 {

using namespace std;

class Kalman;

// Array of the scores we've found for local use
class FeatureAndScore {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureAndScore() {score = 0; fp = NULL;}
  FeatureAndScore(double s, Feature *f) {score = s; fp = f;}
  double  score;
  Feature *fp;
};

class MonoSLAM {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MonoSLAM();
  ~MonoSLAM();

  void Init(const string &config_path);

  bool GoOneStep(cv::Mat frame, bool save_trajectory, bool enable_mapping);
  void InitialiseAutoFeature(cv::Mat frame);
  void print_robot_state();
  bool SavePatch();

  int auto_select_n_features(int n);
  bool deselect_feature(Feature *fp);
  void predict_single_feature_measurements(Feature *sfp);
  bool select_feature(Feature *fp);
  int make_measurements(cv::Mat image);
  bool measure_feature(cv::Mat image, cv::Mat patch, Eigen::VectorXd &z, const Eigen::VectorXd &h, const Eigen::MatrixXd &S);
  bool elliptical_search(const cv::Mat &image,
                         const cv::Mat &patch,
                         const Eigen::Vector2d centre,
                         const Eigen::Matrix2d &PuInv,
                         int *u,
                         int *v,
                         const int uBOXSIZE);
  void failed_measurement_of_feature(Feature *sfp);
  void successful_measurement_of_feature(Feature *sfp);
  void construct_total_state(Eigen::VectorXd &V);
  void construct_total_covariance(Eigen::MatrixXd &M);
  void construct_total_measurement_stuff(Eigen::VectorXd &nu_tot, Eigen::MatrixXd &dh_by_dx_tot, Eigen::MatrixXd &R_tot);
  void fill_states(const Eigen::VectorXd &V);
  void fill_covariances(const Eigen::MatrixXd &M);
  void normalise_state();
  void delete_bad_features();
  void exterminate_features();
  bool toggle_feature_lab(int lab);
  Feature* find_feature_lab(int lab);
  void mark_feature_by_lab(int lab);
  bool delete_feature();
  bool AutoInitialiseFeature(cv::Mat frame, const Eigen::Vector3d &u);
  bool FindNonOverlappingRegion(Eigen::VectorXd local_u,
                                int &init_feature_search_ustart,
                                int &init_feature_search_vstart,
                                int &init_feature_search_ufinish,
                                int &init_feature_search_vfinish,
                                const int FEATURE_INIT_STEPS_TO_PREDICT,
                                const double FEATURE_INIT_DEPTH_HYPOTHESIS);
  bool FindNonOverlappingRegionNoPredict(int safe_feature_search_ustart,
                                         int safe_feature_search_vstart,
                                         int safe_feature_search_ufinish,
                                         int safe_feature_search_vfinish,
                                         int &init_feature_search_ustart,
                                         int &init_feature_search_vstart,
                                         int &init_feature_search_ufinish,
                                         int &init_feature_search_vfinish);
  double set_image_selection_automatically(cv::Mat frame,
                                           int ustart,
                                           int vstart,
                                           int ufinish,
                                           int vfinish);
  void find_best_patch_inside_region(const cv::Mat &image,
                                     int *ubest,
                                     int *vbest,
                                     double *evbest,
                                     const int BOXSIZE,
                                     int ustart,
                                     int vstart,
                                     int ufinish,
                                     int vfinish);
  void find_eigenvalues(double A, double B, double C, double *eval1ptr,
                        double *eval2ptr);
  void InitialiseFeature(cv::Mat frame);
  void copy_into_patch(const cv::Mat frame, cv::Mat patch);
  void add_new_partially_initialised_feature(cv::Mat patch, const Eigen::VectorXd &y);
  void AddNewKnownFeature(const Eigen::VectorXd &y, const Eigen::VectorXd &xp,
                          const string &identifier);
  void MatchPartiallyInitialisedFeatures(cv::Mat frame);
  void predict_partially_initialised_feature_measurements();
  void measure_feature_with_multiple_priors(cv::Mat frame, cv::Mat patch,
                                            vector<Particle> &particle_vector);
  void update_partially_initialised_feature_probabilities(
      const double prune_probability_threshold);
  void delete_partially_initialised_features_past_sell_by_date(
      const int erase_partially_init_feature_after_this_many_attempts,
      const int min_number_of_particles);
  void delete_partially_initialised_feature(vector<FeatureInitInfo>::iterator feat);

  Camera            *camera_;
  MotionModel       *motion_model_;
  FullFeatureModel  *full_feature_model_;
  PartFeatureModel  *part_feature_model_;

  FrameGrabber      *frame_grabber_;
  GraphicTool       *graphic_tool_;

  Kalman            *kalman_;

  Eigen::VectorXd xv_;
  Eigen::MatrixXd Pxx_;

  // Lists of pointers to feature models, fm_list_ contains all features
  vector<Feature *>   feature_list_;
  // selected_fm_list just lists those currently selected for measurement
  vector<Feature *>   selected_feature_list_;
  // A separate list here of partially-initialised features with additional
  // probabilistic information. These features are also in the main feature_list.
  vector<FeatureInitInfo> feature_init_info_vector_;

  // For saving and drawing the camera trajectory
  vector<Eigen::Vector3d> trajectory_store_;

  int           number_of_visible_features_;
  int           next_free_label_;
  int           marked_feature_label_;
  int           total_state_size_;
  // The size of the most recent vector of measurements
  int           successful_measurement_vector_size_;

  // Constant-likes
  double        kDeltaT_;
  int           kNumberOfFeaturesToSelect_;
  int           kNumberOfFeaturesToKeepVisible_;
  int           kMaxFeaturesToInitAtOnce_;
  double        kMinLambda_;
  double        kMaxLambda_;
  int           kNumberOfParticles_;
  double        kStandardDeviationDepthRatio_;
  int           kMinNumberOfParticles_;
  double        kPruneProbabilityThreshold_;
  int           kErasePartiallyInitFeatureAfterThisManyAttempts_;

  // Corners of box we search for new features: save so we can display it
  int           init_feature_search_ustart_;
  int           init_feature_search_vstart_;
  int           init_feature_search_ufinish_;
  int           init_feature_search_vfinish_;
  bool          init_feature_search_region_defined_flag_;

  int           minimum_attempted_measurements_of_feature_;
  double        successful_match_fraction_;

  int           uu_, vv_;
  bool          location_selected_flag_;

  const int     kBoxSize_;
  const double  kNoSigma_;
  const double  kCorrThresh2_;
  const double  kCorrelationSigmaThreshold_;
};

} // namespace SceneLib2

#endif // MONOSLAM_H
