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

#ifndef PANGOLIN_UTIL_H
#define PANGOLIN_UTIL_H

#include "monoslam.h"

namespace SceneLib2 {

struct Handler3D : pangolin::Handler3D {
  Handler3D(pangolin::OpenGlRenderState& view_state, MonoSLAM *monoslam,
            pangolin::AxisDirection enforce_up=pangolin::AxisNone,
            float trans_scale=0.01f) :
    pangolin::Handler3D(view_state, enforce_up, trans_scale) {  monoslam_ptr_ = monoslam; view_state_ = &view_state;  }

  void Mouse(pangolin::View&, pangolin::MouseButton button, int x, int y, bool pressed, int button_state);

  MonoSLAM  *monoslam_ptr_;
  pangolin::OpenGlRenderState *view_state_;
};

struct Handler2D : pangolin::Handler {
  Handler2D(MonoSLAM *monoslam) { monoslam_ptr_ = monoslam; }

  void Mouse(pangolin::View&, pangolin::MouseButton button, int x, int y, bool pressed, int button_state);

  MonoSLAM  *monoslam_ptr_;
};

} // namespace SceneLib2

#endif // PANGOLIN_UTIL_H
