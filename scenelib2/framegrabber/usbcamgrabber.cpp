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

#include "usbcamgrabber.h"

namespace SceneLib2 {

UsbCamGrabber::UsbCamGrabber()
{
  initialised_  = false;
  frame_id_     = 0;

  video_        = NULL;
  video_format_ = NULL;
}

UsbCamGrabber::~UsbCamGrabber()
{
  if (video_ != NULL) {
    delete  video_;
  }

  if (video_format_ != NULL) {
    delete  video_format_;
  }

}

void UsbCamGrabber::Init(const string &uri, FrameGrabber *frame_grabber)
{
  video_              = new pangolin::VideoInput(uri);
  video_format_       = new pangolin::VideoPixelFormat;
  *video_format_      = pangolin::VideoFormatFromString(video_->PixFormat());
  video_width_        = video_->Width();
  video_height_       = video_->Height();

  frame_grabber_      = frame_grabber;
  initialised_        = true;

  ucg_thread_ = boost::thread(boost::ref(*this));
}

void UsbCamGrabber::operator ()()
{
  while (initialised_) {
    if (frame_grabber_->IsFrameBufferFull()==false) {

      Frame   frame;
      cv::Mat temp1, temp2;

      if (!video_->PixFormat().compare("YUV422P"))
        temp1.create(video_->Height(), video_->Width(), CV_8UC2);
      else
        temp1.create(video_->Height(), video_->Width(), CV_8UC3);

      temp2.create(video_->Height(), video_->Width(), CV_8UC1);

      video_->GrabNext(temp1.data, true);

      if (!video_->PixFormat().compare("YUV422P"))
        cv::cvtColor(temp1, temp2, CV_YUV2GRAY_Y422);
      else
        cv::cvtColor(temp1, temp2, CV_RGB2GRAY);

      frame.frame_id = frame_id_;
      frame.data.create(240, 320, CV_8UC1);

      if ((temp2.cols!=320) || (temp2.rows!=240))
        cv::resize(temp2, frame.data, cv::Size(320,240), 1, 1, CV_INTER_LINEAR);
      else
        frame.data = temp2.clone();

      ++frame_id_;

      frame_grabber_->SetFrame(frame);
    }
    else {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
  }
}

} // namespace SceneLib2
