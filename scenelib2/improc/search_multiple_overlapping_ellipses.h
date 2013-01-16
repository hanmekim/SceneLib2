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

#ifndef SEARCH_MULTIPLE_OVERLAPPING_ELLIPSES_H
#define SEARCH_MULTIPLE_OVERLAPPING_ELLIPSES_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>

namespace SceneLib2 {

using namespace std;

// Threshold for stereo correlation using correlate2()
const double CORRTHRESH2 = 0.40;
// Value for the standard deviation of the intensity values in a patch
// below which we deem it unsuitable for correlation
const double CORRELATION_SIGMA_THRESHOLD = 10.0;
// Number of standard deviations to search within images
const double NO_SIGMA = 3.0;
// Hackish value we add to correlation score to penalise if patch sigma is low
const double LOW_SIGMA_PENALTY = 5.0;

class SearchMultipleOverlappingEllipses {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor.
  // @param image The image to search
  // @param patch The image patch to search for
  // @param BOXSIZE The size of the image patch to use
  SearchMultipleOverlappingEllipses(const cv::Mat &image, cv::Mat &patch,
                                    const int BOXSIZE) :
    m_image_(&image),
    m_patch_(&patch),
    m_boxsize_(BOXSIZE) { }

  virtual ~SearchMultipleOverlappingEllipses() {}

  void add_ellipse(const Eigen::Matrix2d &PuInv, const Eigen::Vector2d &search_centre);

  virtual void search();

  // Structure to hold the data for a particular ellipse.
  struct SearchDatum {
    friend class SearchMultipleOverlappingEllipses;
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SearchDatum(const Eigen::Matrix2d &PuInv,
                const Eigen::Vector2d &search_centre);

    int minx() {return int(search_centre_[0] + 0.5) - halfwidth_;}
    int maxx() {return int(search_centre_[0] + 0.5) + halfwidth_;}
    int miny() {return int(search_centre_[1] + 0.5) - halfwidth_;}
    int maxy() {return int(search_centre_[1] + 0.5) + halfwidth_;}

    bool inside_fast(int u, int v);
    bool inside(int u, int v);
    bool inside_relative(int u, int v);

    // The inverse covariance. Set in the constructor via add_ellipse()
    Eigen::Matrix2d PuInv_;
    // The search centre. Set in the constructor via add_ellipse()
    Eigen::Vector2d search_centre_;
    // Set to <code>true</code> after search() if a successful match was found
    bool result_flag_;
    // The x-location of the successful match, if found
    int result_u_;
    // The y-location of the successful match, if found
    int result_v_;
    // Half the width of the search ellipse. Used to quickly decide where to search
    int halfwidth_;
    // Half the height of the search ellipse. Used to quickly decide where to search
    int halfheight_;
  };

  // Typdef the SearchDatum container for convenience
  typedef vector<SearchDatum, Eigen::aligned_allocator<SearchDatum> > SearchData;

  // How many ellipses are registered with the class?
  int size() const {return m_searchdata_.size();}
  // Iterator set to the start of the container of ellipse data
  SearchData::const_iterator begin() const {return m_searchdata_.begin();}
  // Iterator set to one past the end of the container of ellipse data
  SearchData::const_iterator end() const {return m_searchdata_.end();}

  const cv::Mat* image() const {return m_image_;}
  const cv::Mat* patch() const {return m_patch_;}
  int boxsize() const {return m_boxsize_;}
  SearchData& searchdata() {return m_searchdata_;}

  const cv::Mat* m_image_;
  const cv::Mat* m_patch_;
  int m_boxsize_;
  SearchData m_searchdata_;
};

} // namespace SceneLib2

#endif // SEARCH_MULTIPLE_OVERLAPPING_ELLIPSES_H
