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

#include <pangolin/pangolin.h>

#include "monoslam.h"
#include "graphic/graphictool.h"
#include "kalman.h"
#include "support/eigen_util.h"
#include "improc/improc.h"
#include "improc/search_multiple_overlapping_ellipses.h"

namespace SceneLib2 {

MonoSLAM::MonoSLAM() :
  kBoxSize_(11), kNoSigma_(3.0), kCorrThresh2_(0.40),
  kCorrelationSigmaThreshold_(10.0)
{
  camera_ = NULL;
  motion_model_ = NULL;
  full_feature_model_ = NULL;
  part_feature_model_ = NULL;

  frame_grabber_ = NULL;
  graphic_tool_ = NULL;

  kalman_ = NULL;
}

MonoSLAM::~MonoSLAM()
{
  if (camera_ != NULL)
    delete  camera_;

  if (motion_model_ != NULL)
    delete motion_model_;

  if (full_feature_model_ != NULL)
    delete full_feature_model_;

  if (part_feature_model_ != NULL)
    delete part_feature_model_;

  if (frame_grabber_ != NULL)
    delete  frame_grabber_;

  if (graphic_tool_ != NULL)
    delete  graphic_tool_;

  if (kalman_ != NULL)
    delete  kalman_;

  while (!feature_list_.empty())
    feature_list_.pop_back();

  while (!selected_feature_list_.empty())
    selected_feature_list_.pop_back();

  while (!trajectory_store_.empty())
    trajectory_store_.pop_back();
}

// Step the MonoSLAM application on by one frame.
// This should be called every time a new frame is captured (and care should be
// taken to avoid skipping frames).
//
// GoOneStep() performs the following processing steps:
//  - Kalman filter prediction step
//  - Select a set of features to make measurements from
//  - Predict the locations and and make measurements of those features
//  - Kalman filter update step
//  - Delete any bad features (those that have repeatedly failed to be matched)
//  - If we are not currently initialising a enough new features, and the camera
//    is translating, initialise a new feature somewhere sensible
//  - Update the partially-initialised features
bool MonoSLAM::GoOneStep(cv::Mat frame, bool save_trajectory, bool enable_mapping)
{
  location_selected_flag_ = false;  // Equivalent to robot->nullify_image_selection()
  init_feature_search_region_defined_flag_ = false;

  // Control vector of accelerations
  Eigen::Vector3d u;
  u.setZero();

  // Record the current position so that I can estimate velocity
  // (We can guarantee that the state vector has position; we can't guarantee
  // that it has velocity.)
  motion_model_->func_xp(xv_);

  Eigen::Vector3d prev_xp_pos;
  prev_xp_pos << motion_model_->xpRES_(0),motion_model_->xpRES_(1),motion_model_->xpRES_(2);

  // Prediction step
  kalman_->KalmanFilterPredict(this, u);

  number_of_visible_features_ = auto_select_n_features(kNumberOfFeaturesToSelect_);

  if (selected_feature_list_.size() != 0) {
    // Calls function in control_general.cc
    make_measurements(frame);

    if (successful_measurement_vector_size_ != 0) {
      kalman_->KalmanFilterUpdate(this);

      normalise_state();
    }
  }

  delete_bad_features();

  // Let's enforce symmetry of covariance matrix...
  // Add to transpose and divide by 2
  Eigen::MatrixXd Pxx(total_state_size_, total_state_size_);
  construct_total_covariance(Pxx);

  Eigen::MatrixXd PxxT = Pxx.transpose();
  Pxx = Pxx * 0.5 + PxxT * 0.5;
  fill_covariances(Pxx);

  // Look at camera speed estimate
  // Get the current position and estimate the speed from it
  motion_model_->func_xp(xv_);

  Eigen::Vector3d xp_pos, velocity;

  xp_pos << motion_model_->xpRES_(0), motion_model_->xpRES_(1), motion_model_->xpRES_(2);
  velocity = (xp_pos - prev_xp_pos) / kDeltaT_;

  double speed = sqrt(velocity(0)*velocity(0) + velocity(1)*velocity(1) + velocity(2)*velocity(2));

  if (speed > 0.2 && enable_mapping) {
    if (number_of_visible_features_ < kNumberOfFeaturesToKeepVisible_ &&
      feature_init_info_vector_.size() < (unsigned int)kMaxFeaturesToInitAtOnce_) {
      AutoInitialiseFeature(frame, u);
    }
  }

  MatchPartiallyInitialisedFeatures(frame);

  if (save_trajectory) {
    trajectory_store_.push_back(motion_model_->rRES_);
    if (trajectory_store_.size() > 1000) {
      trajectory_store_.erase(trajectory_store_.begin());
    }
  }

  return  true;
}

// Automatically select the n features with the best selection scores. For each
// feature, this predicts their location and calls
// Feature_Measurement_Model::selection_score(), selecting the n with the best
// score.
// @returns the number of visible features.
int MonoSLAM::auto_select_n_features(int n)
{
  int cant_see_flag;    // Flag which we will set to 0 if a feature is
                        // visible and various other values if it isn't

  // Deselect all features
  while (selected_feature_list_.size() != 0)
    deselect_feature(*selected_feature_list_.begin());

  // Have vector of up to n scores; highest first
  vector<FeatureAndScore> feature_and_score_vector;

  vector<Feature *>::iterator it;

  for (it = feature_list_.begin(); it != feature_list_.end(); ++it) {
    if ((*it)->fully_initialised_flag_) {
      predict_single_feature_measurements(*it);

      // See if the feature is visible
      cant_see_flag = (*it)->feature_model_->visibility_test(motion_model_->xpRES_,
                                                             (*it)->y_, (*it)->xp_org_, (*it)->h_);

      // Feature has passed visibility tests
      if (cant_see_flag == 0) {
        double score = (*it)->feature_model_->selection_score((*it)->feature_model_->SiRES_);

        FeatureAndScore fas(score, (*it));

        bool already_added = false;

        for (vector<FeatureAndScore>::iterator fasit = feature_and_score_vector.begin();
            fasit != feature_and_score_vector.end(); ++fasit) {

          if (score > (*fasit).score) {
            // Insert new feature before old one it trumps
            feature_and_score_vector.insert(fasit, fas);
            already_added = true;
            break;
          }
        }

        if (!already_added)
          feature_and_score_vector.push_back(fas);
      }
    }
  }

  // See what we've got
  int n_actual = 0;

  if (feature_and_score_vector.size() == 0) {
    return 0;
  }
  else {
    for (vector<FeatureAndScore>::iterator fasit = feature_and_score_vector.begin();
        fasit != feature_and_score_vector.end(); ++fasit) {
      if ((*fasit).score == 0.0 || n_actual == n)
        return  feature_and_score_vector.size();
      else {
        select_feature((*fasit).fp);
        ++n_actual;
      }
    }
  }

  // Return the number of visible features
  return  feature_and_score_vector.size();
}

// Remove this feature from the list for selection.
// @param fp The feature to remove.
bool MonoSLAM::deselect_feature(Feature *fp)
{
  if (fp->selected_flag_ == false) {
    return  true;
  }

  vector<Feature *>::iterator it;

  for (it = selected_feature_list_.begin(); it != selected_feature_list_.end(); ++it) {
    if (*it == fp)
      break;
  }

  if (it != selected_feature_list_.end()) {
    // We've found it so remove from list
    (*it)->selected_flag_ = false;
    selected_feature_list_.erase(it);
    return  true;
  }
  else {
    // We haven't found that feature in the selected list
    return  false;
  }
}

// For a single feature, work out the predicted feature measurement and the
// Jacobians with respect to the vehicle position and the feature position.
// This calls the appropriate member functions in Feature to set the measurement
// \f$ \vct{h} \f$, the feature location Jacobian \f$ \partfracv{h}{y} \f$, robot
// position Jacobian \f$ \partfrac{\vct{h}}{\vct{x}_v} \f$, measurement covariance
// \f$ \mat{R} \f$ and innovation covariance \f$ \mat{S} \f$ respectively.
void MonoSLAM::predict_single_feature_measurements(Feature *sfp)
{
  motion_model_->func_xp(xv_);

  full_feature_model_->func_hi_and_dhi_by_dxp_and_dhi_by_dyi(sfp->y_, motion_model_->xpRES_);

  sfp->h_ = full_feature_model_->hiRES_;
  sfp->dh_by_dy_ = full_feature_model_->dhi_by_dyiRES_;

  motion_model_->func_dxp_by_dxv(xv_);

  sfp->dh_by_dxv_ = full_feature_model_->dhi_by_dxpRES_ * motion_model_->dxp_by_dxvRES_;

  full_feature_model_->func_Ri(sfp->h_);
  sfp->R_ = full_feature_model_->RiRES_;

  full_feature_model_->func_Si(Pxx_, sfp->Pxy_, sfp->Pyy_, sfp->dh_by_dxv_,
                               sfp->dh_by_dy_, sfp->R_);
  sfp->S_ = full_feature_model_->SiRES_;
}

// Add this feature to the list for selection.
// @param fp The feature to add.
bool MonoSLAM::select_feature(Feature *fp)
{
  if (fp->selected_flag_ == true) {
    return  true;
  }

  fp->selected_flag_ = true;

  selected_feature_list_.push_back(fp);

  return  true;
}

// Make measurements of all the currently-selected features. Features can be
// selected using Scene_Single::auto_select_n_features(), or manually using
// Scene_Single::select_feature(). This calls
// Scene_Single::starting_measurements() and then Sim_Or_Rob::measure_feature()
// for each selected feature. Each
// feature for which a measurement
// attempt is made has its Feature::attempted_measurements_of_feature and
// Feature::successful_measurements_of_feature counts updated.
// @param scene The SLAM map to use
// @param sim_or_rob The class to use for measuring features.
// @returns The number of features successfully measured.
int MonoSLAM::make_measurements(cv::Mat image)
{
  int count = 0;

  if (selected_feature_list_.size() == 0) {
    return  0;
  }

  successful_measurement_vector_size_ = 0;

  for (vector<Feature *>::const_iterator it = selected_feature_list_.begin();
      it != selected_feature_list_.end(); ++it) {

    if(measure_feature(image, (*it)->patch_, (*it)->z_, (*it)->h_, (*it)->S_) == false) {
      failed_measurement_of_feature((*it));
    }
    else {
      successful_measurement_of_feature((*it));
      ++count;
    }
  }

  return  count;
}

// Make a measurement of a feature. This function calls elliptical_search() to
// find the best match within three standard deviations of the predicted location.
// @param id The identifier for this feature (in this case an image patch)
// @param z The best image location match for the feature, to be filled in by this
// function
// @param h The expected image location
// @param S The expected location covariance, used to specify the search region.
bool MonoSLAM::measure_feature(cv::Mat image, cv::Mat patch, Eigen::VectorXd &z,
                               const Eigen::VectorXd &h, const Eigen::MatrixXd &S)
{
  Eigen::LLT<Eigen::MatrixXd>  S_cholesky(S);
  Eigen::MatrixXd S_L = S_cholesky.matrixL();
  Eigen::MatrixXd S_Linv = S_L.inverse();
  Eigen::MatrixXd Sinv = S_Linv.transpose() * S_Linv;

  int  u_found, v_found;

  if(elliptical_search(image, patch, h, Sinv, &u_found, &v_found, kBoxSize_) != true) {
    return  false;
  }

  z(0) = (double)u_found;
  z(1) = (double)v_found;

  return  true;
}

// Do a search for patch in image within an elliptical region. The
// search region is parameterised an inverse covariance matrix (a distance of
// NO_SIGMA is used). The co-ordinates returned are those of centre of the patch.
// @param image The image to search
// @param patch The patch  to search for
// @param centre The centre of the search ellipse
// @param PuInv The inverse covariance matrix to use
// @param u The x-co-ordinate of the best patch location
// @param v The y-co-ordinate of the best patch location
// @param uBOXSIZE The size of the image patch (TODO: Shouldn't this be the same
// as the size of patch?)
// @returns <true> if the a good match is found (above CORRTHRESH2), <false>
// otherwise
bool MonoSLAM::elliptical_search(const cv::Mat &image,
                                 const cv::Mat &patch,
                                 const Eigen::Vector2d centre,
                                 const Eigen::Matrix2d &PuInv,
                                 int *u,
                                 int *v,
                                 const int uBOXSIZE)
{
  // We want to pass BOXSIZE as an unsigned int since it is,
  // but if we use it in the if statements below then C++ casts the
  // whole calculation to unsigned ints, so it is never < 0!
  // Force it to get the right answer by using an int version of BOXSIZE
  int BOXSIZE = uBOXSIZE;

  // The dimensions of the bounding box of the ellipse we want to search in
  int  halfwidth = (int)(kNoSigma_ / sqrt(PuInv(0,0) - PuInv(0,1) * PuInv(0,1) / PuInv(1,1)));
  int  halfheight = (int)(kNoSigma_ / sqrt(PuInv(1,1) - PuInv(0,1) * PuInv(0,1) / PuInv(0,0)));

  int ucentre = int(centre(0) + 0.5);
  int vcentre = int(centre(1) + 0.5);

  // Limits of search
  int urelstart = -halfwidth;
  int urelfinish = halfwidth;
  int vrelstart = -halfheight;
  int vrelfinish = halfheight;

  // Check these limits aren't outside the image
  if(ucentre + urelstart - (BOXSIZE-1) / 2 < 0)
    urelstart = (BOXSIZE-1) / 2 - ucentre;

  if(ucentre + urelfinish - (BOXSIZE-1) / 2 > int(image.size().width) - BOXSIZE)
    urelfinish = image.size().width - BOXSIZE - ucentre + (BOXSIZE-1) / 2;

  if(vcentre + vrelstart - (BOXSIZE-1) / 2 < 0)
    vrelstart = (BOXSIZE-1) / 2 - vcentre;

  if(vcentre + vrelfinish - (BOXSIZE-1) / 2 > int(image.size().height) - BOXSIZE)
    vrelfinish = int(image.size().height) - BOXSIZE - vcentre + (BOXSIZE-1) / 2;

  // Search counters
  int urel, vrel;

  double  corrmax = 1000000.0;
  double  corr;

  // For passing to and_correlate2_warning
  double  sdpatch, sdimage;

  // Do the search
  for (urel = urelstart; urel <= urelfinish; ++urel) {
    for (vrel = vrelstart; vrel <= vrelfinish; ++vrel) {
      if(PuInv(0,0) * urel * urel + 2 * PuInv(0,1) * urel * vrel + PuInv(1,1) * vrel * vrel  <
         kNoSigma_*kNoSigma_) {
        corr = correlate2_warning(0, 0, BOXSIZE, BOXSIZE, ucentre + urel - (BOXSIZE - 1) / 2, vcentre + vrel - (BOXSIZE - 1) / 2, patch, image, &sdpatch, &sdimage);

        if (corr <= corrmax) {
          if (sdpatch < kCorrelationSigmaThreshold_)
            ; // cout << "Low patch sigma." << endl;
          else if (sdimage < kCorrelationSigmaThreshold_)
            ; // cout << "Low image sigma." << endl;
          else {
            corrmax = corr;
            *u = urel + ucentre;
            *v = vrel + vcentre;
          }
        }
      }
    }
  }

  if (corrmax > kCorrThresh2_) {
    return  false;
  }

  return  true;
}

void MonoSLAM::failed_measurement_of_feature(Feature *sfp)
{
  sfp->successful_measurement_flag_ = false;
  ++sfp->attempted_measurements_of_feature_;
}

void MonoSLAM::successful_measurement_of_feature(Feature *sfp)
{
  sfp->successful_measurement_flag_ = true;

  successful_measurement_vector_size_ += full_feature_model_->kMeasurementSize_;

  full_feature_model_->func_nui(sfp->h_, sfp->z_);
  sfp->nu_ = full_feature_model_->nuiRES_;

  ++sfp->successful_measurements_of_feature_;
  ++sfp->attempted_measurements_of_feature_;
}

// Create the overall state vector by concatenating the robot state $x_v$ and all
// the feature states $y_i$.
// @param V The vector to fill with the state
void MonoSLAM::construct_total_state(Eigen::VectorXd &V)
{
  int  y_position = 0;

  VectorUpdate(V, xv_, y_position);
  y_position += motion_model_->kStateSize_;

  for (vector<Feature *>::iterator it = feature_list_.begin(); it != feature_list_.end(); ++it) {
    VectorUpdate(V, (*it)->y_, y_position);
    y_position += (*it)->feature_model_->kFeatureStateSize_;
  }
}

// Create the overall covariance matrix by concatenating the robot
// covariance $P_{xx}$ and all the feature covariances $P_{xy_i}$, $P_{y_iy_i}$
// and $P_{y_iy_j}$.
// @param M The matrix to fill with the state
void MonoSLAM::construct_total_covariance(Eigen::MatrixXd &M)
{
  M.block(0,0,Pxx_.rows(),Pxx_.cols()) = Pxx_;

  int x_position = motion_model_->kStateSize_;

  for (vector<Feature *>::iterator it = feature_list_.begin();
       it != feature_list_.end(); ++it) {

    int y_position = 0;

    M.block(y_position,x_position,(*it)->Pxy_.rows(),(*it)->Pxy_.cols()) = (*it)->Pxy_;
    M.block(x_position,y_position,(*it)->Pxy_.transpose().rows(),(*it)->Pxy_.transpose().cols()) = (*it)->Pxy_.transpose();

    y_position += motion_model_->kStateSize_;

    for (vector<Eigen::MatrixXd>::iterator itmat = (*it)->matrix_block_list_.begin();
        itmat != (*it)->matrix_block_list_.end(); ++itmat) {

      M.block(y_position,x_position,(*itmat).rows(),(*itmat).cols()) = (*itmat);
      M.block(x_position,y_position,(*itmat).transpose().rows(),(*itmat).transpose().cols()) = (*itmat).transpose();

      y_position += (*itmat).rows();
    }

    M.block(y_position,x_position,(*it)->Pyy_.rows(),(*it)->Pyy_.cols()) = (*it)->Pyy_;
    x_position += (*it)->feature_model_->kFeatureStateSize_;
  }
}

void MonoSLAM::construct_total_measurement_stuff(Eigen::VectorXd &nu_tot, Eigen::MatrixXd &dh_by_dx_tot, Eigen::MatrixXd &R_tot)
{
  nu_tot.setZero();
  dh_by_dx_tot.setZero();
  R_tot.setZero();

  int vector_position = 0;

  for (vector<Feature *>::iterator it = selected_feature_list_.begin();
       it != selected_feature_list_.end(); ++it) {

    if ((*it)->successful_measurement_flag_) {
      VectorUpdate(nu_tot, (*it)->nu_, vector_position);

      dh_by_dx_tot.block(vector_position,0,(*it)->dh_by_dxv_.rows(),(*it)->dh_by_dxv_.cols()) = (*it)->dh_by_dxv_;

      dh_by_dx_tot.block(vector_position,(*it)->position_in_total_state_vector_,
                         (*it)->dh_by_dy_.rows(),(*it)->dh_by_dy_.cols()) = (*it)->dh_by_dy_;

      R_tot.block(vector_position,vector_position,(*it)->R_.rows(),(*it)->R_.cols()) = (*it)->R_;

      vector_position += (*it)->feature_model_->kMeasurementSize_;
    }
  }
}

void MonoSLAM::fill_states(const Eigen::VectorXd &V)
{
  int y_position = 0;

  xv_ = VectorExtract(V, y_position, motion_model_->kStateSize_);
  y_position += motion_model_->kStateSize_;

  for (vector<Feature *>::iterator it = feature_list_.begin();
       it != feature_list_.end() && y_position < V.size(); ++it) {
    (*it)->y_ = VectorExtract(V, y_position, (*it)->feature_model_->kFeatureStateSize_);
    y_position += (*it)->feature_model_->kFeatureStateSize_;
  }
}

void MonoSLAM::fill_covariances(const Eigen::MatrixXd &M)
{
  Pxx_ = M.block(0,0,motion_model_->kStateSize_, motion_model_->kStateSize_);

  int x_position = motion_model_->kStateSize_;

  for (vector<Feature *>::iterator it = feature_list_.begin();
       it != feature_list_.end() && x_position < M.cols(); ++it) {

    int y_position = 0;

    (*it)->Pxy_ = M.block(y_position,x_position,motion_model_->kStateSize_,(*it)->feature_model_->kFeatureStateSize_);

    y_position += motion_model_->kStateSize_;

    for (vector<Eigen::MatrixXd>::iterator itmat = (*it)->matrix_block_list_.begin();
        itmat != (*it)->matrix_block_list_.end(); ++itmat) {

      *itmat = M.block(y_position,x_position,itmat->rows(),itmat->cols());
      y_position += itmat->rows();
    }

    (*it)->Pyy_ = M.block(y_position,x_position,(*it)->feature_model_->kFeatureStateSize_,(*it)->feature_model_->kFeatureStateSize_);

    x_position += (*it)->feature_model_->kFeatureStateSize_;
  }
}

void MonoSLAM::normalise_state()
{
  // Normalising state:
  //
  // This deals with the case where the robot state needs normalising
  // (e.g. if it contains a quaternion)
  //
  // We assume the feature states do not need normalising
  motion_model_->func_xvnorm_and_dxvnorm_by_dxv(xv_);

  // Change the state vector
  xv_ = motion_model_->xvnormRES_;

  // Change the vehicle state covariance
  Pxx_ = motion_model_->dxvnorm_by_dxvRES_ * Pxx_ * motion_model_->dxvnorm_by_dxvRES_.transpose();

  // Change the covariances between vehicle state and feature states
  for (vector<Feature *>::iterator it = feature_list_.begin();
       it != feature_list_.end(); ++it) {
    (*it)->Pxy_ = motion_model_->dxvnorm_by_dxvRES_ * (*it)->Pxy_;
  }
}

// Delete any features which are consistently failing measurements. Features are
// deleted if there has been more than a certain number of attempts to match them
// (set by Scene_Single::MINIMUM_ATTEMPTED_MEASUREMENTS_OF_FEATURE) and they have
// been successfully matched on too few of those occasions (set by
// Scene_Single::SUCCESSFUL_MATCH_FRACTION).
void MonoSLAM::delete_bad_features()
{
  vector<Feature *>::iterator it;

  for (it = feature_list_.begin(); it != feature_list_.end(); ++it) {
    // First: test if this feature needs deleting
    if((*it)->attempted_measurements_of_feature_ >= minimum_attempted_measurements_of_feature_ &&
       double(((*it)->successful_measurements_of_feature_)) /
       double(((*it)->attempted_measurements_of_feature_)) < successful_match_fraction_) {
      (*it)->scheduled_for_termination_flag_ = true;
      continue;
    }
  }

  // Delete bad features
  exterminate_features();
}

// Delete all features with scheduled_for_termination_flag set
void MonoSLAM::exterminate_features()
{
  vector<Feature *>::iterator it;

  for (it = feature_list_.begin(); it != feature_list_.end(); ) {

    if ((*it)->scheduled_for_termination_flag_) {
      vector<Feature *>::iterator it_to_delete = it;
      ++it;

      // We have to do something special if deleting the last feature
      bool deleting_last_feature_flag = false;

      if (it == feature_list_.end())
        deleting_last_feature_flag = true;

      // Save currently marked feature so we can mark this scheduled
      // for termination feature (delete_feature deletes the marked feature)
      int currently_marked_feature = marked_feature_label_;

      // Unless it's the one we're about to delete
      if (currently_marked_feature == (int)((*it_to_delete)->label_))
        currently_marked_feature = -1;

      mark_feature_by_lab((*it_to_delete)->label_);
      delete_feature();

      // Now re-mark currently marked feature
      if (currently_marked_feature != -1)
        mark_feature_by_lab(currently_marked_feature);

      if (deleting_last_feature_flag) {
        // jump out of loop
        break;
      }
    }
    else {
      ++it;
    }
  }
}

// Toggle the selection of a feature (for making measurements). This is called
// to manually select or deselect a feature.
// @param lab The label (starting from zero) for the feature to toggle.
bool MonoSLAM::toggle_feature_lab(int lab)
{
  Feature *fp;

  if (!(fp = find_feature_lab(lab))) {
    cerr << "Feature with label " << lab << " not found." << endl;
    return  false;
  }

  if (fp->selected_flag_)
    return  deselect_feature(fp);
  else
    return  select_feature(fp);
}

// Returns the feature with a given label. If the feature does not exist, NULL
// is returned.
Feature* MonoSLAM::find_feature_lab(int lab)
{
  vector<Feature *>::iterator it;

  for (it = feature_list_.begin(); it != feature_list_.end(); ++it) {
    if ((*it)->label_ == lab)
      return *it;
  }

  return  NULL;
}

// Set the current marked feature. Marking a feature is used to identify a feature
// for deletion (by calling delete_feature()), or before calling
// print_marked_feature_state(), get_marked_feature_state() or
// get_feature_measurement_model_for_marked_feature().
// @param lab The label (starting from zero) for the feature to mark. A setting of
// -1 indicates no selection.
void MonoSLAM::mark_feature_by_lab(int lab)
{
  // Check this is valid
  // Can we find it?
  vector<Feature *>::const_iterator found = feature_list_.begin();

  if (lab > 0) {
    for ( ; found != feature_list_.end(); found++) {
      // Below is SceneLib1's code (look at a semi-colon at the end of if!
      //if((*found)->label_ == marked_feature_label_);
      //  break;

      // I think it need to be changed like this.
      if((*found)->label_ == lab)
        break;
    }
  }

  if(found == feature_list_.end() && lab != -1) {
    return;
  }

  marked_feature_label_ = lab;
}

// Delete the currently-marked feature. Features can be marked using
// mark_feature_by_lab(). The function also frees up the identifier.
bool MonoSLAM::delete_feature()
{
  if (marked_feature_label_ == -1) {
    return  false;
  }

  vector<Feature *>::iterator it_to_delete;

  for (it_to_delete = feature_list_.begin(); it_to_delete != feature_list_.end();
       ++it_to_delete) {
    if ((int)((*it_to_delete)->label_) == marked_feature_label_)
      break;
  }

  if (it_to_delete == feature_list_.end()) {
    return  false;
  }

  // Remove the covariance elements relating to this feature from the
  // subsequent features
  for (vector<Feature *>::iterator it = it_to_delete + 1;
       it != feature_list_.end(); ++it) {

    --(*it)->position_in_list_;
    vector<Eigen::MatrixXd>::iterator target = (*it)->matrix_block_list_.begin() + (*it_to_delete)->position_in_list_;
    (*it)->matrix_block_list_.erase(target);

    (*it)->position_in_total_state_vector_ -= (*it_to_delete)->feature_model_->kFeatureStateSize_;
  }

  if ((*it_to_delete)->selected_flag_)
    deselect_feature(*it_to_delete);

  total_state_size_ -= (*it_to_delete)->feature_model_->kFeatureStateSize_;

  // Delete extra data associated with this feature
  delete  (*it_to_delete);
  feature_list_.erase(it_to_delete);

  marked_feature_label_ = -1;

  return  true;
}

// Initialise a feature at a position determined automatically. This predicts
// where the image centre will be soon (in 10 frames), and tries initialising a
// feature near there. This may not necessarily give a new feature - the score
// could not be good enough, or no suitable non-overlapping region might be found.
// @param u The input control vector (zero in the MonoSLAM application)
// @param delta_t The time between frames
// @returns <code>true</code> on success, or <code>false</code> on failure (i.e.
// if no non-overlapping region is found, or if no feature could be found with a
// good enough score).
bool MonoSLAM::AutoInitialiseFeature(cv::Mat frame, const Eigen::Vector3d &u)
{
  // A cute method for deciding where to centre our search for a new feature
  // Idea: look for a point in a position that we expect to be near the
  // image centre soon

  // Predict the camera position a few steps into the future
  const int  FEATURE_INIT_STEPS_TO_PREDICT = 10;

  // Project a point a "reasonable" distance forward from there along
  // the optic axis
  const double FEATURE_INIT_DEPTH_HYPOTHESIS = 2.5;

  // First find a suitable patch
  const double SUITABLE_PATCH_SCORE_THRESHOLD = 20000;

  Eigen::VectorXd local_u(motion_model_->kControlSize_);

  if (FindNonOverlappingRegion(u,
                               init_feature_search_ustart_,
                               init_feature_search_vstart_,
                               init_feature_search_ufinish_,
                               init_feature_search_vfinish_,
                               FEATURE_INIT_STEPS_TO_PREDICT,
                               FEATURE_INIT_DEPTH_HYPOTHESIS)) {

    init_feature_search_region_defined_flag_ = true;

    if (set_image_selection_automatically(frame,
                                          init_feature_search_ustart_,
                                          init_feature_search_vstart_,
                                          init_feature_search_ufinish_,
                                          init_feature_search_vfinish_) > SUITABLE_PATCH_SCORE_THRESHOLD) {
      // Then initialise it
      InitialiseFeature(frame);
    }
    else {
      return  false;
    }
  }
  else {
      return  false;
  }

  return  true;
}

bool MonoSLAM::FindNonOverlappingRegion(Eigen::VectorXd local_u,
                                        int &init_feature_search_ustart,
                                        int &init_feature_search_vstart,
                                        int &init_feature_search_ufinish,
                                        int &init_feature_search_vfinish,
                                        const int FEATURE_INIT_STEPS_TO_PREDICT,
                                        const double FEATURE_INIT_DEPTH_HYPOTHESIS)
{
  Eigen::VectorXd local_xv = xv_;

  for (int i = 0; i < FEATURE_INIT_STEPS_TO_PREDICT; ++i) {
    motion_model_->func_fv_and_dfv_by_dxv(local_xv, local_u, kDeltaT_);
    local_xv = motion_model_->fvRES_;
  }

  motion_model_->func_xp(local_xv);

  Eigen::VectorXd local_xp = motion_model_->xpRES_;

  motion_model_->func_r(local_xp);
  Eigen::Vector3d rW = motion_model_->rRES_;
  motion_model_->func_q(local_xp);
  Eigen::Quaterniond  qWR = motion_model_->qRES_;

  // yW =  rW + RWR hR
  Eigen::Vector3d hR(0.0, 0.0, FEATURE_INIT_DEPTH_HYPOTHESIS);

  // Used Inverse + transpose because this was't compiling the normal way
  Eigen::Vector3d yW = rW + qWR.toRotationMatrix() * hR;

  // Then project that point into the current camera position
  motion_model_->func_xp(xv_);

  full_feature_model_->func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yW, motion_model_->xpRES_);

  // Now, this defines roughly how much we expect a feature initialised
  // to move
  double  predicted_motion_u = camera_->width_ / 2.0 - full_feature_model_->hiRES_(0);
  double  predicted_motion_v = camera_->height_ / 2.0 - full_feature_model_->hiRES_(1);

  // So, the limits of a "safe" region within which we can initialise
  // features so that they end up staying within the screen
  // (Making the approximation that the whole screen moves like the
  // centre point)
  int safe_feature_search_ustart = (int)(-predicted_motion_u);
  int safe_feature_search_vstart = (int)(-predicted_motion_v);
  int safe_feature_search_ufinish = (int)(camera_->width_ - predicted_motion_u);
  int safe_feature_search_vfinish = (int)(camera_->height_ - predicted_motion_v);

  if (safe_feature_search_ustart < ((int)((kBoxSize_-1)/2) + 1))
    safe_feature_search_ustart = (kBoxSize_-1)/2 + 1;
  if (safe_feature_search_ufinish > (int)camera_->width_ - ((int)((kBoxSize_-1)/2) + 1))
    safe_feature_search_ufinish = (int) camera_->width_ - (kBoxSize_-1)/2 - 1;
  if (safe_feature_search_vstart < ((int)((kBoxSize_-1)/2) + 1))
    safe_feature_search_vstart = (kBoxSize_-1)/2 + 1;
  if (safe_feature_search_vfinish > (int)camera_->height_ - ((int)((kBoxSize_-1)/2) + 1))
    safe_feature_search_vfinish = camera_->height_ - (kBoxSize_-1)/2 - 1;

  return  FindNonOverlappingRegionNoPredict(safe_feature_search_ustart,
                                            safe_feature_search_vstart,
                                            safe_feature_search_ufinish,
                                            safe_feature_search_vfinish,
                                            init_feature_search_ustart,
                                            init_feature_search_vstart,
                                            init_feature_search_ufinish,
                                            init_feature_search_vfinish);
}

bool MonoSLAM::FindNonOverlappingRegionNoPredict(int safe_feature_search_ustart,
                                                 int safe_feature_search_vstart,
                                                 int safe_feature_search_ufinish,
                                                 int safe_feature_search_vfinish,
                                                 int &init_feature_search_ustart,
                                                 int &init_feature_search_vstart,
                                                 int &init_feature_search_ufinish,
                                                 int &init_feature_search_vfinish)
{
  const int INIT_FEATURE_SEARCH_WIDTH = 80;
  const int INIT_FEATURE_SEARCH_HEIGHT = 60;

  // Within this, choose a random region
  // Check that we've got some room for manouevre
  if (safe_feature_search_ufinish - safe_feature_search_ustart > INIT_FEATURE_SEARCH_WIDTH &&
      safe_feature_search_vfinish - safe_feature_search_vstart > INIT_FEATURE_SEARCH_HEIGHT) {

    // Try a few times to get one that's not overlapping with any features
    // we know about
    const int NUMBER_OF_RANDOM_INIT_FEATURE_SEARCH_REGION_TRIES = 5;
    const int FEATURE_SEPARATION_MINIMUM = 10;

    // Build vectors of feature positions so we only have to work them out once
    vector<double>  u_array;
    vector<double>  v_array;

    for (vector<Feature *>::const_iterator it = feature_list_.begin();
         it != feature_list_.end(); ++it) {

      if ((*it)->fully_initialised_flag_) {
        full_feature_model_->func_hi_and_dhi_by_dxp_and_dhi_by_dyi((*it)->y_, motion_model_->xpRES_);

        full_feature_model_->func_zeroedyigraphics_and_Pzeroedyigraphics(
              (*it)->y_,
              xv_,
              Pxx_,
              (*it)->Pxy_,
              (*it)->Pyy_);

        if (full_feature_model_->zeroedyigraphicsRES_(2) > 0) {
          u_array.push_back(full_feature_model_->hiRES_(0));
          v_array.push_back(full_feature_model_->hiRES_(1));
        }
      }
    }

    int i = 0;

    while (i < NUMBER_OF_RANDOM_INIT_FEATURE_SEARCH_REGION_TRIES) {

      int u_offset = int((safe_feature_search_ufinish - safe_feature_search_ustart - INIT_FEATURE_SEARCH_WIDTH) * drand48());
      int v_offset = int((safe_feature_search_vfinish - safe_feature_search_vstart - INIT_FEATURE_SEARCH_HEIGHT) * drand48());

      init_feature_search_ustart = safe_feature_search_ustart + u_offset;
      init_feature_search_ufinish = init_feature_search_ustart + INIT_FEATURE_SEARCH_WIDTH;
      init_feature_search_vstart = safe_feature_search_vstart + v_offset;
      init_feature_search_vfinish = init_feature_search_vstart + INIT_FEATURE_SEARCH_HEIGHT;

      bool found_a_feature_in_region_flag = false;

      // These arrays will be the same size
      vector<double>::const_iterator uit = u_array.begin();
      vector<double>::const_iterator vit = v_array.begin();

      while(uit != u_array.end()) {
        if (*uit >= init_feature_search_ustart - FEATURE_SEPARATION_MINIMUM &&
            *uit < init_feature_search_ufinish + FEATURE_SEPARATION_MINIMUM &&
            *vit >= init_feature_search_vstart - FEATURE_SEPARATION_MINIMUM &&
            *vit < init_feature_search_vfinish + FEATURE_SEPARATION_MINIMUM) {
          found_a_feature_in_region_flag = true;
          break;
        }
        ++uit;
        ++vit;
      }

      // If no clashes, fine, leave while loop
      // Else go round again
      if (!found_a_feature_in_region_flag) {
        break;
      }

      ++i;
    }

    if (i == NUMBER_OF_RANDOM_INIT_FEATURE_SEARCH_REGION_TRIES) {
      return false;
    }
  }
  else {
    return  false;
  }

  return  true;
}

// Search a region for the best image patch, and set the current selection to
// this. This just calls find_best_patch_inside_region() to find a patch using the
// Shi and Tomasi criterion.
// @param ustart The x-cordinate of the start of the region
// @param vstart The y-cordinate of the start of the region
// @param ufinish The x-cordinate of the end of the region
// @param vfinish The y-cordinate of the end of the region
// @returns The smallest eigenvalue of the best patch (high means good for
// correlation)
double MonoSLAM::set_image_selection_automatically(cv::Mat frame,
                                                   int ustart,
                                                   int vstart,
                                                   int ufinish,
                                                   int vfinish)
{
  double  evbest = 0;

  find_best_patch_inside_region(frame, &uu_, &vv_, &evbest, kBoxSize_, ustart, vstart, ufinish, vfinish);
  location_selected_flag_ = true;

  return evbest;
}

// Function to scan over (a window in an) image and find the best patch by the Shi
// and Tomasi criterion.
// Method: as described in notes from 1/7/97. Form sums in an incremental
// way to speed things up.
// @param image The image to scan
// @param ubest The x-co-ordinate of the best patch
// @param vbest The y-co-ordinate of the best patch
// @param evbest The smallest eigenvalue of the best patch (larger is better)
// @param BOXSIZE The size of the patch to use
// @param ustart The x-co-ordinate of the start of the search window
// @param vstart The y-co-ordinate of the start of the search window
// @param ufinish The x-co-ordinate of the end of the search window
// @param vfinish The v-co-ordinate of the end of the search window
void MonoSLAM::find_best_patch_inside_region(const cv::Mat &image,
                                             int *ubest,
                                             int *vbest,
                                             double *evbest,
                                             const int BOXSIZE,
                                             int ustart,
                                             int vstart,
                                             int ufinish,
                                             int vfinish)
{
  // Check that these limits aren't too close to the image edges.
  // Note that we can't use the edge pixels because we can't form
  // gradients here.
  if (ustart < (BOXSIZE-1)/2 + 1)
    ustart = (BOXSIZE-1)/2 + 1;
  if (ufinish > image.size().width - (BOXSIZE-1)/2 - 1)
    ufinish = image.size().width - (BOXSIZE-1)/2 - 1;
  if (vstart < (BOXSIZE-1)/2 + 1)
    vstart = (BOXSIZE-1)/2 + 1;
  if (vfinish > image.size().height - (BOXSIZE-1)/2 - 1)
    vfinish = image.size().height - (BOXSIZE-1)/2 - 1;

  // Is there anything left to search? If not, set the score to zero and return.
  if(vstart >= vfinish || ustart >= ufinish) {
    *ubest = ustart;
    *vbest = vstart;
    *evbest = 0;
    return;
  }

  // Calculate the width we need to find gradients in.
  int calc_width = ufinish - ustart + BOXSIZE - 1;

  // Arrays which store column sums of height BOXSIZE
  double CSgxsq[calc_width], CSgysq[calc_width], CSgxgy[calc_width];

  // For the final sums at each position (u, v)
  double TSgxsq = 0.0, TSgysq = 0.0, TSgxgy = 0.0;

  double gx, gy;
  int u = ustart, v = vstart;
  double eval1, eval2;

  // Initial stage: fill these sums for the first horizontal position
  int cstart = ustart - (BOXSIZE-1)/2;
  int cfinish = ufinish + (BOXSIZE-1)/2;
  int rstart = vstart - (BOXSIZE-1)/2;
  int i;
  int c, r;

  for (c = cstart, i = 0; c < cfinish; ++c, ++i) {
    CSgxsq[i] = 0; CSgysq[i] = 0; CSgxgy[i] = 0;

    for (r = rstart; r < rstart + BOXSIZE; ++r) {

      gx = (image.at<unsigned char>(r, c+1) - image.at<unsigned char>(r, c-1)) / 2.0;
      gy = (image.at<unsigned char>(r+1, c) - image.at<unsigned char>(r-1, c)) / 2.0;

      CSgxsq[i] += gx * gx;
      CSgysq[i] += gy * gy;
      CSgxgy[i] += gx * gy;
    }
  }

  // Now loop through u and v to form sums
  *evbest = 0;

  for (v = vstart; v < vfinish; ++v) {
    u = ustart;

    // Form first sums for u = ustart
    TSgxsq = 0.0, TSgysq = 0.0, TSgxgy = 0.0;

    for (i = 0; i < BOXSIZE; ++i) {
      TSgxsq += CSgxsq[i];
      TSgysq += CSgysq[i];
      TSgxgy += CSgxgy[i];
    }

    for (u = ustart; u < ufinish; ++u) {
      if (u != ustart) {
        // Subtract old column, add new one
        TSgxsq += CSgxsq[u - ustart + BOXSIZE - 1] - CSgxsq[u - ustart - 1];
        TSgysq += CSgysq[u - ustart + BOXSIZE - 1] - CSgysq[u - ustart - 1];
        TSgxgy += CSgxgy[u - ustart + BOXSIZE - 1] - CSgxgy[u - ustart - 1];
      }

      find_eigenvalues(TSgxsq, TSgxgy, TSgysq, &eval1, &eval2);

      // eval2 will be the smaller eigenvalue. Compare it with the one
      // we've already got
      if (eval2 > *evbest) {
        *ubest = u;
        *vbest = v;
        *evbest = eval2;
      }
    }

    if (v != vfinish - 1) {

      // Update the column sums for the next v
      for (c = cstart, i = 0; c < cfinish; ++c, ++i) {
        // Subtract the old top pixel
        gx = (image.at<unsigned char>(v - (BOXSIZE-1)/2, c+1) -
              image.at<unsigned char>(v - (BOXSIZE-1)/2, c-1)) / 2.0;
        gy = (image.at<unsigned char>(v - (BOXSIZE-1)/2 + 1, c) -
              image.at<unsigned char>(v - (BOXSIZE-1)/2 - 1, c)) / 2.0;

        CSgxsq[i] -= gx * gx;
        CSgysq[i] -= gy * gy;
        CSgxgy[i] -= gx * gy;

        // Add the new bottom pixel
        gx = (image.at<unsigned char>(v + (BOXSIZE-1)/2 + 1, c+1) -
              image.at<unsigned char>(v + (BOXSIZE-1)/2 + 1, c-1)) / 2.0;
        gy = (image.at<unsigned char>(v + (BOXSIZE-1)/2 + 1 + 1, c) -
              image.at<unsigned char>(v + (BOXSIZE-1)/2 + 1 - 1, c)) / 2.0;

        CSgxsq[i] += gx * gx;
        CSgysq[i] += gy * gy;
        CSgxgy[i] += gx * gy;
      }
    }
  }
}

// Simple function to find the eigenvalues of the 2*2 symmetric matrix
// \f$ \begin{pmatrix}A & B \\ B & C \end{pmatrix} \nonumber \f$
void MonoSLAM::find_eigenvalues(double A, double B, double C,
                                double *eval1ptr, double *eval2ptr)
{
  double BB = sqrt((A + C) * (A + C) - 4 * (A * C - B * B));

  *eval1ptr = (A + C + BB) / 2.0;
  *eval2ptr = (A + C - BB) / 2.0;
}

// Initialise a feature at the currently-selected image location. The image
// selection is usually either chosen automatically with
// Robot::set_image_selection_automatically() or manually by the user by calling
// Robot::set_image_selection().
void MonoSLAM::InitialiseFeature(cv::Mat frame)
{
  // Fill in z and id for the currently-selected feature
  Eigen::VectorXd z(part_feature_model_->kMeasurementSize_);
  z << uu_, vv_;

  cv::Mat patch(kBoxSize_, kBoxSize_, frame.type());
  copy_into_patch(frame, patch);

  add_new_partially_initialised_feature(patch, z);

  // And set up a particle distribution for this partially initialised feature
  double lambda_step = (1.0 / double(kNumberOfParticles_)) * (kMaxLambda_ - kMinLambda_);
  const double uniform_probability = 1.0 / double(kNumberOfParticles_);

  // Initialise particle set with uniform prior
  Eigen::VectorXd lambda(1);
  lambda(0) = kMinLambda_;

  for (int i = 0; i < kNumberOfParticles_; ++i) {
    feature_init_info_vector_.back().add_particle(lambda, uniform_probability);

    lambda(0) += lambda_step;
  }
}

// Little service function to copy image region centred at uu, vv into patch
void MonoSLAM::copy_into_patch(const cv::Mat frame, cv::Mat patch)
{
  int r_mul, v_mul;

  for (int r = 0; r < kBoxSize_; ++r) {

    r_mul = r * kBoxSize_;
    v_mul = (r + vv_ - (kBoxSize_-1) / 2) * frame.size().width;

    for (int c = 0; c < kBoxSize_; ++c) {
      *(patch.data +r_mul +c) = *(frame.data +v_mul +c + uu_ - (kBoxSize_-1) / 2);
    }
  }
}

// Create a new partially-initialised feature. This creates a new Feature, and
// creates a new empty FeatureInitInfo to store the extra initialisation
// information, which must be then filled in by the caller of this function.
// @param id The unique identifier for this feature (e.g. the image patch)
// @param h The initial measured state for this feature (e.g. the image location)
// @param f_m_m The partially-initialised feature measurement model to apply to
// this feature.
// @returns A pointer to the FeatureInitInfo object to be filled in with further
// initialisation information.
void MonoSLAM::add_new_partially_initialised_feature(
    cv::Mat patch, const Eigen::VectorXd &y)
{
  Feature *nf = new Feature(patch, y, this);

  feature_list_.push_back(nf);
  total_state_size_ += part_feature_model_->kFeatureStateSize_;
  ++next_free_label_; // Potential millenium-style bug when this overloads
                      // (hello if you're reading this in the year 3000)

  // Set stuff to store extra probability information for partially
  // initialised feature
  FeatureInitInfo feat(nf, part_feature_model_->kFreeParameterSize_, part_feature_model_->kMeasurementSize_);
  feature_init_info_vector_.push_back(feat);
}

void MonoSLAM::AddNewKnownFeature(const Eigen::VectorXd &y,
                                  const Eigen::VectorXd &xp,
                                  const string &identifier)
{
  Feature *nf = new Feature(full_feature_model_, y, xp,
                            next_free_label_, total_state_size_, identifier,
                            feature_list_);

  feature_list_.push_back(nf);

  total_state_size_ += full_feature_model_->kFeatureStateSize_;
  ++next_free_label_; // Potential millenium-style bug when this overloads
                      // (hello if you're reading this in the year 3000)
}

// Try to match the partially-initialised features, then update their
// distributions. If possible (if the standard deviation ratio \f$ \Sigma_{11} /
// \mu_1 \f$ is small enough) this also converts them to fully-initalised
// features. In addition, this also calls
// Scene_Single::delete_partially_initialised_features_past_sell_by_date() to
// delete any that have not collapses fast enough.
void MonoSLAM::MatchPartiallyInitialisedFeatures(cv::Mat frame)
{
  // Go through all partially initialised features and decide which ones
  // to try to measure; predict measurements for these
  predict_partially_initialised_feature_measurements();

  // Loop through partially initialised features
  for (vector<FeatureInitInfo>::iterator feat = feature_init_info_vector_.begin();
       feat != feature_init_info_vector_.end(); ++feat) {

    if (feat->making_measurement_on_this_step_flag_) {
      // Make search for this feature over overlapping ellipses of particles
      measure_feature_with_multiple_priors(frame, feat->fp_->patch_, feat->particle_vector_);
    }
  }

  // Update particle distributions with measurements, normalise, prune, etc.
  update_partially_initialised_feature_probabilities(kPruneProbabilityThreshold_);

  // Now go through the features again and decide whether to convert
  // any to fully-initialised features
  // Do this here rather than generically in Scene because the test
  // for conversion is specific to this application
  for (vector<FeatureInitInfo>::iterator feat = feature_init_info_vector_.begin();
       feat != feature_init_info_vector_.end(); ++feat) {
    // Only try to convert if we've been making measurements!
    if (feat->making_measurement_on_this_step_flag_) {
      double  mean_sd_ratio = sqrt(feat->covariance_(0, 0)) / feat->mean_(0);

      if (mean_sd_ratio < kStandardDeviationDepthRatio_ &&
          feat->particle_vector_.size() > (unsigned int)kMinNumberOfParticles_) {
        feat->fp_->convert_from_partially_to_fully_initialised(feat->mean_, feat->covariance_, this);
        feature_init_info_vector_.erase(feat--);
      }
    }
  }

  // Go through and get rid of any partially initialised features whose
  // distributions haven't collapsed fast enough
  delete_partially_initialised_features_past_sell_by_date(
        kErasePartiallyInitFeatureAfterThisManyAttempts_, kMinNumberOfParticles_);
}

// Update the measurement states of the partially-initialised features to
// correspond to the current robot state. This is called prior to matching them
// in a new frame. For each Particle in each FeatureInitInfo, the measurement state
// \f$ h_{pi} \f$ is updated, and the inverse innovation covariance
// \f$ S_i^{-1} \f$, and the determinant of \f$ S_i \f$ are stored.
void MonoSLAM::predict_partially_initialised_feature_measurements()
{
  vector<FeatureInitInfo>::iterator feat;
  Feature *fp;

  // Get the current position
  motion_model_->func_xp(xv_);
  Eigen::VectorXd local_xp = motion_model_->xpRES_;

  motion_model_->func_dxp_by_dxv(xv_);
  Eigen::MatrixXd local_dxp_by_dxv = motion_model_->dxp_by_dxvRES_;

  // Loop over different partially initialised features
  for (feat = feature_init_info_vector_.begin();
       feat < feature_init_info_vector_.end(); ++feat) {
    fp = feat->fp_;

    // We don't try to match a feature immediately after it has been
    // initialised
    if(feat->number_of_match_attempts_++ != 0) {
      feat->making_measurement_on_this_step_flag_ = true;

      // Loop over particles for a particular feature
      // Fill in h, Sinv, detS information predicting measurement
      for (vector<Particle>::iterator part = feat->particle_vector_.begin();
           part != feat->particle_vector_.end(); ++part) {
        // Get and update the measurement state for the current feature with
        // this particle's lambda from the current position
        part_feature_model_->func_hpi_and_dhpi_by_dxp_and_dhpi_by_dyi(fp->y_,
                                                                      local_xp,
                                                                      part->lambda_);
        part->m_h_ = part_feature_model_->hpiRES_;

        // Set the measurement covariance from the feature's measurement state
        fp->feature_model_->func_Ri(part->m_h_);

        // And calculate the innovation covariance
        // (i.e. combine the uncertainties in robot state, feature state, and
        // measurement to give the overall uncertainty in the measurement state
        // This will be used to determine what area of the image to search
        fp->feature_model_->func_Si(Pxx_, fp->Pxy_, fp->Pyy_,
                                    part_feature_model_->dhpi_by_dxpRES_ *
                                    local_dxp_by_dxv,
                                    part_feature_model_->dhpi_by_dyiRES_,
                                    fp->feature_model_->RiRES_);

        part->set_S(fp->feature_model_->SiRES_);
      }
    }
    else {
      feat->making_measurement_on_this_step_flag_ = false;
    }
  }
}

// Make measurements of a feature which is represented by a set of particles.
// This is typically a partially-initialised feature (see
// Partially_Initialised_Feature_Measurement_Model), where the particles represent
// the probability distribution in the direction of the free parameters.
// @param id The Identified for this feature (in this case this will be an image
// patch)
// @param particle_vector The vector of particles. The covariance of each particle
// is used to define the region over which the feature is matched and measured.
void MonoSLAM::measure_feature_with_multiple_priors(cv::Mat frame, cv::Mat patch,
                                                    vector<Particle> &particle_vector)
{
  SearchMultipleOverlappingEllipses ellipse_search(frame, patch, kBoxSize_);

  for (vector<Particle>::const_iterator part = particle_vector.begin();
       part != particle_vector.end(); ++part) {
    ellipse_search.add_ellipse(part->m_SInv_, part->m_h_);
  }

  ellipse_search.search();

  // Turn results into matrix forms
  SearchMultipleOverlappingEllipses::SearchData::const_iterator e = ellipse_search.begin();

  for (vector<Particle>::iterator it = particle_vector.begin();
       it != particle_vector.end(); ++it, ++e) {
    if(e->result_flag_) {
      // Save the measurement location back into the particle
      Eigen::Vector2d z_local(e->result_u_, e->result_v_);
      it->m_z_ = z_local;
      it->m_successful_measurement_flag_ = true;
    }
    else {
      it->m_successful_measurement_flag_ = false;
    }
  }
}

// Update the probabiluties of the partially-initialised features after
// their measurement. For each Particle in each FeatureInitInfo this updates the
// probabilities of the particles according to their measurement (i.e how big the
// innovation \f$ \nu_i = z_i - h_i \f$ was compared to the innovation covariance
// \f$ S_i \f$), and then normalises the probabilities.
// Feature::prune_particle_vector() is also called to prune the particles, and any
// features where all the particle measurements failed are deleted. */
void MonoSLAM::update_partially_initialised_feature_probabilities(
    const double prune_probability_threshold)
{
  // Loop over different partially initialised features
  for (vector<FeatureInitInfo>::iterator feat = feature_init_info_vector_.begin();
      feat < feature_init_info_vector_.end(); ++feat) {
    if (feat->making_measurement_on_this_step_flag_) {
      // Update the probabilities.
      for (vector<Particle>::iterator it = feat->particle_vector_.begin();
          it != feat->particle_vector_.end(); ++it) {
        double likelihood;

        if (it->m_successful_measurement_flag_ == true) {
          // Evaluate Gaussian
          Eigen::VectorXd nu = it->m_z_ - it->m_h_;
          Eigen::VectorXd SInv_times_nu = it->m_SInv_ * nu;

          // A scalar
          // Equivalent to DotProduct(nu, SInv_times_nu)
          double nuT_Sinv_nu = nu.dot(SInv_times_nu);

          // pux is the probability that the true feature lies in this position
          double pux = (1.0 / (sqrt(2.0 * M_PI * it->m_detS_))) * exp(-0.5 * nuT_Sinv_nu);

          likelihood = pux;
        }
        else {
          likelihood = 0.0;
        }

        // And Bayes rule
        it->probability_ = it->probability_ * likelihood;
      }

      if (feat->normalise_particle_vector_and_calculate_cumulative()) {
        feat->prune_particle_vector(prune_probability_threshold);

        feat->calculate_mean_and_covariance();
      }
      else {
        // normalise_particle_vector_and_calculate_cumulative
        // returns false if all probabilities are zero (all failed matches)
        // so delete this feature
        delete_partially_initialised_feature(feat);
      }
    }
  }
}

// Go through the list of partially-initialised features and delete any that have
// timed out. A feature times out if it has not collapsed after a certain number
// of attempts, or if it has fewer than the minimum number of particles.
// @param erase_partially_init_feature_after_this_many_attempts The number of
// match attempts before the particle is deleted.
// @param min_number_of_particles The number of particles below which the feature
// is deleted.
void MonoSLAM::delete_partially_initialised_features_past_sell_by_date(
    const int erase_partially_init_feature_after_this_many_attempts,
    const int min_number_of_particles)
{
  vector<FeatureInitInfo>::iterator feat = feature_init_info_vector_.begin();

  while (feat != feature_init_info_vector_.end()) {
    if(feat->number_of_match_attempts_ > erase_partially_init_feature_after_this_many_attempts ||
       feat->particle_vector_.size() <= (unsigned int)min_number_of_particles) {
      delete_partially_initialised_feature(feat);
    }
    else {
      ++feat;
    }
  }
}

void MonoSLAM::delete_partially_initialised_feature(
    vector<FeatureInitInfo>::iterator feat)
{
  // Save currently marked feature
  int currently_marked_feature = marked_feature_label_;

  // Mark feature to delete
  mark_feature_by_lab(feat->fp_->label_);

  delete_feature();
  feature_init_info_vector_.erase(feat);

  if (currently_marked_feature != -1)
    mark_feature_by_lab(currently_marked_feature);
}

void MonoSLAM::InitialiseAutoFeature(cv::Mat frame)
{
  Eigen::Vector3d u;
  u.setZero();

  AutoInitialiseFeature(frame, u);
}

void MonoSLAM::print_robot_state()
{
  cout << "[Robot state]" << endl;
  cout << xv_ << endl;
  cout << "[Robot covariance]" << endl;
  cout << Pxx_ << endl;
}

bool MonoSLAM::SavePatch()
{
  if (marked_feature_label_ == -1) {
    return  false;
  }

  vector<Feature *>::iterator it_to_save;

  for (it_to_save = feature_list_.begin(); it_to_save != feature_list_.end();
       ++it_to_save) {
    if ((int)((*it_to_save)->label_) == marked_feature_label_)
      break;
  }

  if (it_to_save == feature_list_.end()) {
    return  false;
  }

  cv::imwrite("patch.png", (*it_to_save)->patch_);

  return  true;
}

void MonoSLAM::Init(const string &config_path)
{
  // TODO: Need to use array, vector, quaternion and matrix forms
  // instead listing all components!
  pangolin::ParseVarsFile(config_path);

  pangolin::Var<bool>   input_mode("input.mode", false);
  pangolin::Var<string> input_name("input.name", "empty");

  pangolin::Var<double> delta_t("params.delta_t", 0.0);
  pangolin::Var<int>    number_of_features_to_select("params.number_of_features_to_select", 0);
  pangolin::Var<int>    number_of_features_to_keep_visible("params.number_of_features_to_keep_visible", 0);
  pangolin::Var<int>    max_features_to_init_at_once("params.max_features_to_init_at_once", 0);
  pangolin::Var<double> min_lambda("params.min_lambda", 0.0);
  pangolin::Var<double> max_lambda("params.max_lambda", 0.0);
  pangolin::Var<int>    number_of_particles("params.number_of_particles", 0);
  pangolin::Var<double> standard_deviation_depth_ratio("params.standard_deviation_depth_ratio", 0.0);
  pangolin::Var<int>    min_number_of_particles("params.min_number_of_particles", 0);
  pangolin::Var<double> prune_probability_threshold("params.prune_probability_threshold", 0.0);
  pangolin::Var<int>    erase_partially_init_feature_after_this_many_attempts("params.erase_partially_init_feature_after_this_many_attempts", 0);

  pangolin::Var<int>    cam_width("cam.width", 0);
  pangolin::Var<int>    cam_height("cam.height", 0);
  pangolin::Var<int>    cam_fku("cam.fku", 0);
  pangolin::Var<int>    cam_fkv("cam.fkv", 0);
  pangolin::Var<int>    cam_u0("cam.u0", 0);
  pangolin::Var<int>    cam_v0("cam.v0", 0);
  pangolin::Var<double> cam_kd1("cam.kd1", 0.0);
  pangolin::Var<int>    cam_sd("cam.sd", 0);

  pangolin::Var<double> state_rw_x("state.rw_x", 0.0);
  pangolin::Var<double> state_rw_y("state.rw_y", 0.0);
  pangolin::Var<double> state_rw_z("state.rw_z", 0.0);
  pangolin::Var<double> state_qwr_x("state.qwr_x", 0.0);
  pangolin::Var<double> state_qwr_y("state.qwr_y", 0.0);
  pangolin::Var<double> state_qwr_z("state.qwr_z", 0.0);
  pangolin::Var<double> state_qwr_w("state.qwr_w", 0.0);
  pangolin::Var<double> state_vw_x("state.vw_x", 0.0);
  pangolin::Var<double> state_vw_y("state.vw_y", 0.0);
  pangolin::Var<double> state_vw_z("state.vw_z", 0.0);
  pangolin::Var<double> state_ww_x("state.ww_x", 0.0);
  pangolin::Var<double> state_ww_y("state.ww_y", 0.0);
  pangolin::Var<double> state_ww_z("state.ww_z", 0.0);

  pangolin::Var<double> state_pxx0_0("state.pxx0_0", 0.0);
  pangolin::Var<double> state_pxx0_1("state.pxx0_1", 0.0);
  pangolin::Var<double> state_pxx0_2("state.pxx0_2", 0.0);
  pangolin::Var<double> state_pxx0_3("state.pxx0_3", 0.0);
  pangolin::Var<double> state_pxx0_4("state.pxx0_4", 0.0);
  pangolin::Var<double> state_pxx0_5("state.pxx0_5", 0.0);
  pangolin::Var<double> state_pxx0_6("state.pxx0_6", 0.0);
  pangolin::Var<double> state_pxx0_7("state.pxx0_7", 0.0);
  pangolin::Var<double> state_pxx0_8("state.pxx0_8", 0.0);
  pangolin::Var<double> state_pxx0_9("state.pxx0_9", 0.0);
  pangolin::Var<double> state_pxx0_10("state.pxx0_10", 0.0);
  pangolin::Var<double> state_pxx0_11("state.pxx0_11", 0.0);
  pangolin::Var<double> state_pxx0_12("state.pxx0_12", 0.0);

  pangolin::Var<double> state_pxx1_0("state.pxx1_0", 0.0);
  pangolin::Var<double> state_pxx1_1("state.pxx1_1", 0.0);
  pangolin::Var<double> state_pxx1_2("state.pxx1_2", 0.0);
  pangolin::Var<double> state_pxx1_3("state.pxx1_3", 0.0);
  pangolin::Var<double> state_pxx1_4("state.pxx1_4", 0.0);
  pangolin::Var<double> state_pxx1_5("state.pxx1_5", 0.0);
  pangolin::Var<double> state_pxx1_6("state.pxx1_6", 0.0);
  pangolin::Var<double> state_pxx1_7("state.pxx1_7", 0.0);
  pangolin::Var<double> state_pxx1_8("state.pxx1_8", 0.0);
  pangolin::Var<double> state_pxx1_9("state.pxx1_9", 0.0);
  pangolin::Var<double> state_pxx1_10("state.pxx1_10", 0.0);
  pangolin::Var<double> state_pxx1_11("state.pxx1_11", 0.0);
  pangolin::Var<double> state_pxx1_12("state.pxx1_12", 0.0);

  pangolin::Var<double> state_pxx2_0("state.pxx2_0", 0.0);
  pangolin::Var<double> state_pxx2_1("state.pxx2_1", 0.0);
  pangolin::Var<double> state_pxx2_2("state.pxx2_2", 0.0);
  pangolin::Var<double> state_pxx2_3("state.pxx2_3", 0.0);
  pangolin::Var<double> state_pxx2_4("state.pxx2_4", 0.0);
  pangolin::Var<double> state_pxx2_5("state.pxx2_5", 0.0);
  pangolin::Var<double> state_pxx2_6("state.pxx2_6", 0.0);
  pangolin::Var<double> state_pxx2_7("state.pxx2_7", 0.0);
  pangolin::Var<double> state_pxx2_8("state.pxx2_8", 0.0);
  pangolin::Var<double> state_pxx2_9("state.pxx2_9", 0.0);
  pangolin::Var<double> state_pxx2_10("state.pxx2_10", 0.0);
  pangolin::Var<double> state_pxx2_11("state.pxx2_11", 0.0);
  pangolin::Var<double> state_pxx2_12("state.pxx2_12", 0.0);

  pangolin::Var<double> state_pxx3_0("state.pxx3_0", 0.0);
  pangolin::Var<double> state_pxx3_1("state.pxx3_1", 0.0);
  pangolin::Var<double> state_pxx3_2("state.pxx3_2", 0.0);
  pangolin::Var<double> state_pxx3_3("state.pxx3_3", 0.0);
  pangolin::Var<double> state_pxx3_4("state.pxx3_4", 0.0);
  pangolin::Var<double> state_pxx3_5("state.pxx3_5", 0.0);
  pangolin::Var<double> state_pxx3_6("state.pxx3_6", 0.0);
  pangolin::Var<double> state_pxx3_7("state.pxx3_7", 0.0);
  pangolin::Var<double> state_pxx3_8("state.pxx3_8", 0.0);
  pangolin::Var<double> state_pxx3_9("state.pxx3_9", 0.0);
  pangolin::Var<double> state_pxx3_10("state.pxx3_10", 0.0);
  pangolin::Var<double> state_pxx3_11("state.pxx3_11", 0.0);
  pangolin::Var<double> state_pxx3_12("state.pxx3_12", 0.0);

  pangolin::Var<double> state_pxx4_0("state.pxx4_0", 0.0);
  pangolin::Var<double> state_pxx4_1("state.pxx4_1", 0.0);
  pangolin::Var<double> state_pxx4_2("state.pxx4_2", 0.0);
  pangolin::Var<double> state_pxx4_3("state.pxx4_3", 0.0);
  pangolin::Var<double> state_pxx4_4("state.pxx4_4", 0.0);
  pangolin::Var<double> state_pxx4_5("state.pxx4_5", 0.0);
  pangolin::Var<double> state_pxx4_6("state.pxx4_6", 0.0);
  pangolin::Var<double> state_pxx4_7("state.pxx4_7", 0.0);
  pangolin::Var<double> state_pxx4_8("state.pxx4_8", 0.0);
  pangolin::Var<double> state_pxx4_9("state.pxx4_9", 0.0);
  pangolin::Var<double> state_pxx4_10("state.pxx4_10", 0.0);
  pangolin::Var<double> state_pxx4_11("state.pxx4_11", 0.0);
  pangolin::Var<double> state_pxx4_12("state.pxx4_12", 0.0);

  pangolin::Var<double> state_pxx5_0("state.pxx5_0", 0.0);
  pangolin::Var<double> state_pxx5_1("state.pxx5_1", 0.0);
  pangolin::Var<double> state_pxx5_2("state.pxx5_2", 0.0);
  pangolin::Var<double> state_pxx5_3("state.pxx5_3", 0.0);
  pangolin::Var<double> state_pxx5_4("state.pxx5_4", 0.0);
  pangolin::Var<double> state_pxx5_5("state.pxx5_5", 0.0);
  pangolin::Var<double> state_pxx5_6("state.pxx5_6", 0.0);
  pangolin::Var<double> state_pxx5_7("state.pxx5_7", 0.0);
  pangolin::Var<double> state_pxx5_8("state.pxx5_8", 0.0);
  pangolin::Var<double> state_pxx5_9("state.pxx5_9", 0.0);
  pangolin::Var<double> state_pxx5_10("state.pxx5_10", 0.0);
  pangolin::Var<double> state_pxx5_11("state.pxx5_11", 0.0);
  pangolin::Var<double> state_pxx5_12("state.pxx5_12", 0.0);

  pangolin::Var<double> state_pxx6_0("state.pxx6_0", 0.0);
  pangolin::Var<double> state_pxx6_1("state.pxx6_1", 0.0);
  pangolin::Var<double> state_pxx6_2("state.pxx6_2", 0.0);
  pangolin::Var<double> state_pxx6_3("state.pxx6_3", 0.0);
  pangolin::Var<double> state_pxx6_4("state.pxx6_4", 0.0);
  pangolin::Var<double> state_pxx6_5("state.pxx6_5", 0.0);
  pangolin::Var<double> state_pxx6_6("state.pxx6_6", 0.0);
  pangolin::Var<double> state_pxx6_7("state.pxx6_7", 0.0);
  pangolin::Var<double> state_pxx6_8("state.pxx6_8", 0.0);
  pangolin::Var<double> state_pxx6_9("state.pxx6_9", 0.0);
  pangolin::Var<double> state_pxx6_10("state.pxx6_10", 0.0);
  pangolin::Var<double> state_pxx6_11("state.pxx6_11", 0.0);
  pangolin::Var<double> state_pxx6_12("state.pxx6_12", 0.0);

  pangolin::Var<double> state_pxx7_0("state.pxx7_0", 0.0);
  pangolin::Var<double> state_pxx7_1("state.pxx7_1", 0.0);
  pangolin::Var<double> state_pxx7_2("state.pxx7_2", 0.0);
  pangolin::Var<double> state_pxx7_3("state.pxx7_3", 0.0);
  pangolin::Var<double> state_pxx7_4("state.pxx7_4", 0.0);
  pangolin::Var<double> state_pxx7_5("state.pxx7_5", 0.0);
  pangolin::Var<double> state_pxx7_6("state.pxx7_6", 0.0);
  pangolin::Var<double> state_pxx7_7("state.pxx7_7", 0.0);
  pangolin::Var<double> state_pxx7_8("state.pxx7_8", 0.0);
  pangolin::Var<double> state_pxx7_9("state.pxx7_9", 0.0);
  pangolin::Var<double> state_pxx7_10("state.pxx7_10", 0.0);
  pangolin::Var<double> state_pxx7_11("state.pxx7_11", 0.0);
  pangolin::Var<double> state_pxx7_12("state.pxx7_12", 0.0);

  pangolin::Var<double> state_pxx8_0("state.pxx8_0", 0.0);
  pangolin::Var<double> state_pxx8_1("state.pxx8_1", 0.0);
  pangolin::Var<double> state_pxx8_2("state.pxx8_2", 0.0);
  pangolin::Var<double> state_pxx8_3("state.pxx8_3", 0.0);
  pangolin::Var<double> state_pxx8_4("state.pxx8_4", 0.0);
  pangolin::Var<double> state_pxx8_5("state.pxx8_5", 0.0);
  pangolin::Var<double> state_pxx8_6("state.pxx8_6", 0.0);
  pangolin::Var<double> state_pxx8_7("state.pxx8_7", 0.0);
  pangolin::Var<double> state_pxx8_8("state.pxx8_8", 0.0);
  pangolin::Var<double> state_pxx8_9("state.pxx8_9", 0.0);
  pangolin::Var<double> state_pxx8_10("state.pxx8_10", 0.0);
  pangolin::Var<double> state_pxx8_11("state.pxx8_11", 0.0);
  pangolin::Var<double> state_pxx8_12("state.pxx8_12", 0.0);

  pangolin::Var<double> state_pxx9_0("state.pxx9_0", 0.0);
  pangolin::Var<double> state_pxx9_1("state.pxx9_1", 0.0);
  pangolin::Var<double> state_pxx9_2("state.pxx9_2", 0.0);
  pangolin::Var<double> state_pxx9_3("state.pxx9_3", 0.0);
  pangolin::Var<double> state_pxx9_4("state.pxx9_4", 0.0);
  pangolin::Var<double> state_pxx9_5("state.pxx9_5", 0.0);
  pangolin::Var<double> state_pxx9_6("state.pxx9_6", 0.0);
  pangolin::Var<double> state_pxx9_7("state.pxx9_7", 0.0);
  pangolin::Var<double> state_pxx9_8("state.pxx9_8", 0.0);
  pangolin::Var<double> state_pxx9_9("state.pxx9_9", 0.0);
  pangolin::Var<double> state_pxx9_10("state.pxx9_10", 0.0);
  pangolin::Var<double> state_pxx9_11("state.pxx9_11", 0.0);
  pangolin::Var<double> state_pxx9_12("state.pxx9_12", 0.0);

  pangolin::Var<double> state_pxx10_0("state.pxx10_0", 0.0);
  pangolin::Var<double> state_pxx10_1("state.pxx10_1", 0.0);
  pangolin::Var<double> state_pxx10_2("state.pxx10_2", 0.0);
  pangolin::Var<double> state_pxx10_3("state.pxx10_3", 0.0);
  pangolin::Var<double> state_pxx10_4("state.pxx10_4", 0.0);
  pangolin::Var<double> state_pxx10_5("state.pxx10_5", 0.0);
  pangolin::Var<double> state_pxx10_6("state.pxx10_6", 0.0);
  pangolin::Var<double> state_pxx10_7("state.pxx10_7", 0.0);
  pangolin::Var<double> state_pxx10_8("state.pxx10_8", 0.0);
  pangolin::Var<double> state_pxx10_9("state.pxx10_9", 0.0);
  pangolin::Var<double> state_pxx10_10("state.pxx10_10", 0.0);
  pangolin::Var<double> state_pxx10_11("state.pxx10_11", 0.0);
  pangolin::Var<double> state_pxx10_12("state.pxx10_12", 0.0);

  pangolin::Var<double> state_pxx11_0("state.pxx11_0", 0.0);
  pangolin::Var<double> state_pxx11_1("state.pxx11_1", 0.0);
  pangolin::Var<double> state_pxx11_2("state.pxx11_2", 0.0);
  pangolin::Var<double> state_pxx11_3("state.pxx11_3", 0.0);
  pangolin::Var<double> state_pxx11_4("state.pxx11_4", 0.0);
  pangolin::Var<double> state_pxx11_5("state.pxx11_5", 0.0);
  pangolin::Var<double> state_pxx11_6("state.pxx11_6", 0.0);
  pangolin::Var<double> state_pxx11_7("state.pxx11_7", 0.0);
  pangolin::Var<double> state_pxx11_8("state.pxx11_8", 0.0);
  pangolin::Var<double> state_pxx11_9("state.pxx11_9", 0.0);
  pangolin::Var<double> state_pxx11_10("state.pxx11_10", 0.0);
  pangolin::Var<double> state_pxx11_11("state.pxx11_11", 0.0);
  pangolin::Var<double> state_pxx11_12("state.pxx11_12", 0.0);

  pangolin::Var<double> state_pxx12_0("state.pxx12_0", 0.0);
  pangolin::Var<double> state_pxx12_1("state.pxx12_1", 0.0);
  pangolin::Var<double> state_pxx12_2("state.pxx12_2", 0.0);
  pangolin::Var<double> state_pxx12_3("state.pxx12_3", 0.0);
  pangolin::Var<double> state_pxx12_4("state.pxx12_4", 0.0);
  pangolin::Var<double> state_pxx12_5("state.pxx12_5", 0.0);
  pangolin::Var<double> state_pxx12_6("state.pxx12_6", 0.0);
  pangolin::Var<double> state_pxx12_7("state.pxx12_7", 0.0);
  pangolin::Var<double> state_pxx12_8("state.pxx12_8", 0.0);
  pangolin::Var<double> state_pxx12_9("state.pxx12_9", 0.0);
  pangolin::Var<double> state_pxx12_10("state.pxx12_10", 0.0);
  pangolin::Var<double> state_pxx12_11("state.pxx12_11", 0.0);
  pangolin::Var<double> state_pxx12_12("state.pxx12_12", 0.0);

  pangolin::Var<string> f1_identifier("f1.identifier", "empty");
  pangolin::Var<double> f1_yi_x("f1.yi_x", 0.0);
  pangolin::Var<double> f1_yi_y("f1.yi_y", 0.0);
  pangolin::Var<double> f1_yi_z("f1.yi_z", 0.0);
  pangolin::Var<double> f1_xp_org_0("f1.xp_org_0", 0.0);
  pangolin::Var<double> f1_xp_org_1("f1.xp_org_1", 0.0);
  pangolin::Var<double> f1_xp_org_2("f1.xp_org_2", 0.0);
  pangolin::Var<double> f1_xp_org_3("f1.xp_org_3", 0.0);
  pangolin::Var<double> f1_xp_org_4("f1.xp_org_4", 0.0);
  pangolin::Var<double> f1_xp_org_5("f1.xp_org_5", 0.0);
  pangolin::Var<double> f1_xp_org_6("f1.xp_org_6", 0.0);

  pangolin::Var<string> f2_identifier("f2.identifier", "empty");
  pangolin::Var<double> f2_yi_x("f2.yi_x", 0.0);
  pangolin::Var<double> f2_yi_y("f2.yi_y", 0.0);
  pangolin::Var<double> f2_yi_z("f2.yi_z", 0.0);
  pangolin::Var<double> f2_xp_org_0("f2.xp_org_0", 0.0);
  pangolin::Var<double> f2_xp_org_1("f2.xp_org_1", 0.0);
  pangolin::Var<double> f2_xp_org_2("f2.xp_org_2", 0.0);
  pangolin::Var<double> f2_xp_org_3("f2.xp_org_3", 0.0);
  pangolin::Var<double> f2_xp_org_4("f2.xp_org_4", 0.0);
  pangolin::Var<double> f2_xp_org_5("f2.xp_org_5", 0.0);
  pangolin::Var<double> f2_xp_org_6("f2.xp_org_6", 0.0);

  pangolin::Var<string> f3_identifier("f3.identifier", "empty");
  pangolin::Var<double> f3_yi_x("f3.yi_x", 0.0);
  pangolin::Var<double> f3_yi_y("f3.yi_y", 0.0);
  pangolin::Var<double> f3_yi_z("f3.yi_z", 0.0);
  pangolin::Var<double> f3_xp_org_0("f3.xp_org_0", 0.0);
  pangolin::Var<double> f3_xp_org_1("f3.xp_org_1", 0.0);
  pangolin::Var<double> f3_xp_org_2("f3.xp_org_2", 0.0);
  pangolin::Var<double> f3_xp_org_3("f3.xp_org_3", 0.0);
  pangolin::Var<double> f3_xp_org_4("f3.xp_org_4", 0.0);
  pangolin::Var<double> f3_xp_org_5("f3.xp_org_5", 0.0);
  pangolin::Var<double> f3_xp_org_6("f3.xp_org_6", 0.0);

  pangolin::Var<string> f4_identifier("f4.identifier", "empty");
  pangolin::Var<double> f4_yi_x("f4.yi_x", 0.0);
  pangolin::Var<double> f4_yi_y("f4.yi_y", 0.0);
  pangolin::Var<double> f4_yi_z("f4.yi_z", 0.0);
  pangolin::Var<double> f4_xp_org_0("f4.xp_org_0", 0.0);
  pangolin::Var<double> f4_xp_org_1("f4.xp_org_1", 0.0);
  pangolin::Var<double> f4_xp_org_2("f4.xp_org_2", 0.0);
  pangolin::Var<double> f4_xp_org_3("f4.xp_org_3", 0.0);
  pangolin::Var<double> f4_xp_org_4("f4.xp_org_4", 0.0);
  pangolin::Var<double> f4_xp_org_5("f4.xp_org_5", 0.0);
  pangolin::Var<double> f4_xp_org_6("f4.xp_org_6", 0.0);

  // Keep this order!
  //  1. Camera
  //  2. MotionModel
  //  3. FullFeatureModel and PartFeatureModel
  camera_ = new Camera();
  camera_->SetCameraParameters(cam_width, cam_height, cam_fku, cam_fkv,
                               cam_u0, cam_v0, cam_kd1, cam_sd);

  motion_model_ = new MotionModel();

  full_feature_model_ = new FullFeatureModel(2, 3, 3, camera_, motion_model_);
  part_feature_model_ = new PartFeatureModel(2, 6, 6, camera_, motion_model_, 3);

  // Initialise constant-like variables
  kDeltaT_ = delta_t;
  kNumberOfFeaturesToSelect_ = number_of_features_to_select;
  kNumberOfFeaturesToKeepVisible_ = number_of_features_to_keep_visible;
  kMaxFeaturesToInitAtOnce_ = max_features_to_init_at_once;
  kMinLambda_ = min_lambda;
  kMaxLambda_ = max_lambda;
  kNumberOfParticles_ = number_of_particles;
  kStandardDeviationDepthRatio_ = standard_deviation_depth_ratio;
  kMinNumberOfParticles_ = min_number_of_particles;
  kPruneProbabilityThreshold_ = prune_probability_threshold;
  kErasePartiallyInitFeatureAfterThisManyAttempts_ = erase_partially_init_feature_after_this_many_attempts;

  number_of_visible_features_ = 0;
  minimum_attempted_measurements_of_feature_ = 10;
  successful_match_fraction_ = 0.5;
  next_free_label_ = 0;
  marked_feature_label_ = -1;
  total_state_size_ = motion_model_->kStateSize_;

  xv_.resize(motion_model_->kStateSize_);
  xv_ << state_rw_x, state_rw_y, state_rw_z,
         state_qwr_w, state_qwr_x, state_qwr_y, state_qwr_z,
         state_vw_x, state_vw_y, state_vw_z,
         state_ww_x, state_ww_y, state_ww_z;

  Pxx_.resize(motion_model_->kStateSize_, motion_model_->kStateSize_);
  Pxx_ << state_pxx0_0,state_pxx0_1,state_pxx0_2,state_pxx0_3,
          state_pxx0_4,state_pxx0_5,state_pxx0_6,state_pxx0_7,
          state_pxx0_8,state_pxx0_9,state_pxx0_10,state_pxx0_11,state_pxx0_12,

          state_pxx1_0,state_pxx1_1,state_pxx1_2,state_pxx1_3,
          state_pxx1_4,state_pxx1_5,state_pxx1_6,state_pxx1_7,
          state_pxx1_8,state_pxx1_9,state_pxx1_10,state_pxx1_11,state_pxx1_12,

          state_pxx2_0,state_pxx2_1,state_pxx2_2,state_pxx2_3,
          state_pxx2_4,state_pxx2_5,state_pxx2_6,state_pxx2_7,
          state_pxx2_8,state_pxx2_9,state_pxx2_10,state_pxx2_11,state_pxx2_12,

          state_pxx3_0,state_pxx3_1,state_pxx3_2,state_pxx3_3,
          state_pxx3_4,state_pxx3_5,state_pxx3_6,state_pxx3_7,
          state_pxx3_8,state_pxx3_9,state_pxx3_10,state_pxx3_11,state_pxx3_12,

          state_pxx4_0,state_pxx4_1,state_pxx4_2,state_pxx4_3,
          state_pxx4_4,state_pxx4_5,state_pxx4_6,state_pxx4_7,
          state_pxx4_8,state_pxx4_9,state_pxx4_10,state_pxx4_11,state_pxx4_12,

          state_pxx5_0,state_pxx5_1,state_pxx5_2,state_pxx5_3,
          state_pxx5_4,state_pxx5_5,state_pxx5_6,state_pxx5_7,
          state_pxx5_8,state_pxx5_9,state_pxx5_10,state_pxx5_11,state_pxx5_12,

          state_pxx6_0,state_pxx6_1,state_pxx6_2,state_pxx6_3,
          state_pxx6_4,state_pxx6_5,state_pxx6_6,state_pxx6_7,
          state_pxx6_8,state_pxx6_9,state_pxx6_10,state_pxx6_11,state_pxx6_12,

          state_pxx7_0,state_pxx7_1,state_pxx7_2,state_pxx7_3,
          state_pxx7_4,state_pxx7_5,state_pxx7_6,state_pxx7_7,
          state_pxx7_8,state_pxx7_9,state_pxx7_10,state_pxx7_11,state_pxx7_12,

          state_pxx8_0,state_pxx8_1,state_pxx8_2,state_pxx8_3,
          state_pxx8_4,state_pxx8_5,state_pxx8_6,state_pxx8_7,
          state_pxx8_8,state_pxx8_9,state_pxx8_10,state_pxx8_11,state_pxx8_12,

          state_pxx9_0,state_pxx9_1,state_pxx9_2,state_pxx9_3,
          state_pxx9_4,state_pxx9_5,state_pxx9_6,state_pxx9_7,
          state_pxx9_8,state_pxx9_9,state_pxx9_10,state_pxx9_11,state_pxx9_12,

          state_pxx10_0,state_pxx10_1,state_pxx10_2,state_pxx10_3,
          state_pxx10_4,state_pxx10_5,state_pxx10_6,state_pxx10_7,
          state_pxx10_8,state_pxx10_9,state_pxx10_10,state_pxx10_11,state_pxx10_12,

          state_pxx11_0,state_pxx11_1,state_pxx11_2,state_pxx11_3,
          state_pxx11_4,state_pxx11_5,state_pxx11_6,state_pxx11_7,
          state_pxx11_8,state_pxx11_9,state_pxx11_10,state_pxx11_11,state_pxx11_12,

          state_pxx12_0,state_pxx12_1,state_pxx12_2,state_pxx12_3,
          state_pxx12_4,state_pxx12_5,state_pxx12_6,state_pxx12_7,
          state_pxx12_8,state_pxx12_9,state_pxx12_10,state_pxx12_11,state_pxx12_12;

  // Add initial features
  Eigen::VectorXd y_temp(3), xp_temp(7);

  y_temp << f1_yi_x,f1_yi_y,f1_yi_z;
  xp_temp << f1_xp_org_0,f1_xp_org_1,f1_xp_org_2,f1_xp_org_3,f1_xp_org_4,f1_xp_org_5,f1_xp_org_6;
  AddNewKnownFeature(y_temp, xp_temp, f1_identifier);

  y_temp << f2_yi_x,f2_yi_y,f2_yi_z;
  xp_temp << f2_xp_org_0,f2_xp_org_1,f2_xp_org_2,f2_xp_org_3,f2_xp_org_4,f2_xp_org_5,f2_xp_org_6;
  AddNewKnownFeature(y_temp, xp_temp, f2_identifier);

  y_temp << f3_yi_x,f3_yi_y,f3_yi_z;
  xp_temp << f3_xp_org_0,f3_xp_org_1,f3_xp_org_2,f3_xp_org_3,f3_xp_org_4,f3_xp_org_5,f3_xp_org_6;
  AddNewKnownFeature(y_temp, xp_temp, f3_identifier);

  y_temp << f4_yi_x,f4_yi_y,f4_yi_z;
  xp_temp << f4_xp_org_0,f4_xp_org_1,f4_xp_org_2,f4_xp_org_3,f4_xp_org_4,f4_xp_org_5,f4_xp_org_6;
  AddNewKnownFeature(y_temp, xp_temp, f4_identifier);

  // Good for the last step, ready for capturing, drawing and processing
  kalman_ = new Kalman();
  graphic_tool_ = new GraphicTool(this);
  frame_grabber_ = new FrameGrabber();
  frame_grabber_->Init(input_name, input_mode);

  init_feature_search_region_defined_flag_ = false;
  location_selected_flag_ = false;

  srand48(0); // Always the same seed (pick a number), so deterministic
}

} // namespace SceneLib2
