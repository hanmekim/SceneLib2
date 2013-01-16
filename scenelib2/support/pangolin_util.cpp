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

#include "pangolin_util.h"

namespace SceneLib2 {

void Handler3D::Mouse(pangolin::View& display, pangolin::MouseButton button, int x, int y, bool pressed, int button_state)
{
  pangolin::Handler3D::Mouse(display, button, x, y, pressed, button_state);

  if(pressed && (button_state == 1)) {
    int picked;

    display.ActivateScissorAndClear(*view_state_);
    picked = monoslam_ptr_->graphic_tool_->Picker(x, y, true);

    if (picked != 0) {
      monoslam_ptr_->toggle_feature_lab(picked-1);
      monoslam_ptr_->mark_feature_by_lab(picked-1);
    }
  }
}

void Handler2D::Mouse(pangolin::View& display, pangolin::MouseButton button, int x, int y, bool pressed, int button_state)
{
  if(pressed && (button_state == 1)) {
    int picked;

    double  scale_w = (double)monoslam_ptr_->camera_->width_ / (double)display.v.w;
    double  scale_h = (double)monoslam_ptr_->camera_->height_ / (double)display.v.h;

    int local_x = (int)((double)(x - display.v.l) * scale_w);
    int local_y = (int)((double)((display.v.b + display.v.h) - y) * scale_h);

    display.ActivateScissorAndClear();
    picked = monoslam_ptr_->graphic_tool_->Picker(x, y, false);

    if (picked != 0) {
      monoslam_ptr_->toggle_feature_lab(picked-1);
      monoslam_ptr_->mark_feature_by_lab(picked-1);
    }
    else {
      monoslam_ptr_->uu_ = local_x;
      monoslam_ptr_->vv_ = local_y;
      monoslam_ptr_->location_selected_flag_ = true;
    }
  }
}

} // namespace SceneLib2
