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

#include "feature_init_info.h"

namespace SceneLib2 {

Particle::Particle(const Eigen::VectorXd &l, double p, const int measurement_size)
{
  lambda_ = l;
  probability_ = p;

  m_z_.resize(measurement_size);
  m_h_.resize(measurement_size);
  m_SInv_.resize(measurement_size, measurement_size);
}

Particle::~Particle()
{
}

// Set the current innovation covariance \f$ S_i^{-1} \f$ for this
// particle. Called from
// Scene_Single::predict_partially_initialised_feature_measurements()
void Particle::set_S(const Eigen::MatrixXd &Si)
{
  Eigen::LLT<Eigen::MatrixXd>  local_Si_cholesky(Si);
  Eigen::MatrixXd S_L = local_Si_cholesky.matrixL();
  Eigen::MatrixXd S_Linv = S_L.inverse();

  m_SInv_ = S_Linv.transpose() * S_Linv;
  m_detS_ = Si.determinant();
}

FeatureInitInfo::FeatureInitInfo(Feature *fp, const int particle_dimension,
                                 const int measurement_size) : fp_(fp)
{
  kParticleDimension_ = particle_dimension;
  kMeasurementSize_ = measurement_size;
  number_of_match_attempts_ = 0;

  mean_.resize(particle_dimension);
  covariance_.resize(particle_dimension, particle_dimension);
}

FeatureInitInfo::~FeatureInitInfo()
{
}

// Create a particle and add it to this partially-initialised feature's list of
// particles.
// @param lambda The value(s) of the free parameters represented by this particle.
// @param probability The initial probability to assign to the particle.
void FeatureInitInfo::add_particle(const Eigen::VectorXd &lambda, double probability)
{
  Particle  p(lambda, probability, kMeasurementSize_);
  particle_vector_.push_back(p);
}

// Normalise the probabilities of the particles (divide them by the
// total probability so that they sum to one). This function sets each
// Particle::probability to its normalised value, and also sets
// Particle::cumulative_probability to the total probability of all the particles
// up to and including the current one (after normalisation).
// @returns <code>false</code> if the total probability is zero, <code>true</code>
// otherwise.
bool FeatureInitInfo::normalise_particle_vector_and_calculate_cumulative()
{
  double  total = 0.0;

  for (vector<Particle>::iterator it = particle_vector_.begin();
       it != particle_vector_.end(); ++it) {
    total += it->probability_;
  }

  if (total == 0.0)
    return  false;

  double  cumulative_total = 0.0;

  for (vector<Particle>::iterator it = particle_vector_.begin();
       it != particle_vector_.end(); ++it) {
    it->probability_ = it->probability_ / total;
    it->cumulative_probability_ = cumulative_total + it->probability_;
    cumulative_total += it->probability_;
  }

  return  true;
}

// Prune the vector of particles.
// It is assumed that the particle probabilities are normalised before we do this.
// This function remove particles with probability below
// prune_probability_threshold * (1/N) and then normalises again.
// @param prune_probability_threshold The threshold to use.
void FeatureInitInfo::prune_particle_vector(const double prune_probability_threshold)
{
  double  prune_threshold = prune_probability_threshold / double(particle_vector_.size());

  vector<Particle>::iterator it = particle_vector_.begin();

  while (it != particle_vector_.end()) {
    if (it->probability_ < prune_threshold) {
      it = particle_vector_.erase(it);
    }
    else {
      ++it;
    }
  }

  normalise_particle_vector_and_calculate_cumulative();
}

// Calculate the mean and covariance of \f$ \lambda \f$ over all the particles,
// i.e.
// \f{align}
//   \text{mean} &= \mu = \sum_i \lambda_i p(i) \nonumber
//   \text{mean} &= \sum_i \lambda_i\lambda_i^T p(i) - \mu\mu^T \nonumber
// \f}
// The result is not returned, but is instead stored in the class to be read using
// get_mean() and get_covariance().
void FeatureInitInfo::calculate_mean_and_covariance()
{
  // Vector which will store expected value of lambda * lambda^T
  // (allows us to do this calculation with one pass through particles)
  Eigen::MatrixXd ExpectedSquared(kParticleDimension_, kParticleDimension_);
  ExpectedSquared.setZero();

  // Zero mean vector before filling it up
  mean_.setZero();

  for (vector<Particle>::iterator it = particle_vector_.begin();
       it != particle_vector_.end(); ++it) {
    mean_ += it->probability_ * it->lambda_;

    // (it->lambda_ * it->lambda_) is outer product in Eigen3?
    ExpectedSquared += it->probability_ * (it->lambda_ * it->lambda_);
  }

  // (mean_ * mean_) is outer product in Eigen3?
  covariance_ = ExpectedSquared - (mean_ * mean_);
}

} // namespace SceneLib2
