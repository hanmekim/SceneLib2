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

#include "framegrabber.h"
#include "filegrabber.h"
#include "usbcamgrabber.h"

namespace SceneLib2 {

FrameGrabber::FrameGrabber()
{
  file_grabber_ = NULL;
  usb_cam_grabber_ = NULL;
}

FrameGrabber::~FrameGrabber()
{
  if (file_grabber_ != NULL)
    delete  file_grabber_;

  if (usb_cam_grabber_ != NULL)
    delete  usb_cam_grabber_;

  while (!frame_buffer_.empty()) {
    frame_buffer_.pop();
  }
}

void FrameGrabber::Init(const string &dev, const bool mode)
{
  if (!mode) {
    file_grabber_ = new FileGrabber;
    file_grabber_->Init(dev, this);
  }
  else {
    usb_cam_grabber_ = new UsbCamGrabber;
    usb_cam_grabber_->Init(dev, this);
  }
}

bool FrameGrabber::GetFrame(int frame_id, Frame *frame)
{
  boost::mutex::scoped_lock lock(fg_mutex_);

  if (frame_buffer_.size() < 1) {
    return  false;
  }

  *frame = frame_buffer_.front();
//  assert(frame->frame_id == frame_id);
  frame_buffer_.pop();

  return  true;
}

void FrameGrabber::SetFrame(const Frame &frame)
{
  boost::mutex::scoped_lock lock(fg_mutex_);

  frame_buffer_.push(frame);
}

bool FrameGrabber::IsFrameBufferFull()
{
  boost::mutex::scoped_lock lock(fg_mutex_);

  if (frame_buffer_.size() < 50) {
    return  false;
  }

  return  true;
}

} // namespace SceneLib2
