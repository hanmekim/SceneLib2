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

#include "math_util.h"

namespace SceneLib2 {

const double  GetAngleFromQuaternion(const Eigen::Quaterniond &q)
{
  return  (2.0 * atan2(sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z()), q.w()));
}

const Eigen::Vector3d GetAxisFromQuaternion(const Eigen::Quaterniond &q)
{
  Eigen::Vector3d axis(0, 0, 0);
  const double  ang = GetAngleFromQuaternion(q);

  if (ang > 0.0) {
    const double  scale = ang / sin(ang / 2.0);

    axis(0) = q.x() * scale;
    axis(1) = q.y() * scale;
    axis(2) = q.z() * scale;
  }

  return  axis;
}

Eigen::Quaterniond QuaternionFromAngularVelocity(const Eigen::Vector3d av)
{
  Eigen::Quaterniond  q;
  const double        angle = sqrt(av(0)*av(0) + av(1)*av(1) + av(2)*av(2));

  if (angle > 0.0) {
    const double s = sin(angle/2.0) / angle;
    const double c = cos(angle/2.0);

    q.x() = s * av(0);
    q.y() = s * av(1);
    q.z() = s * av(2);
    q.w() = c;
  } else {
    q.x() = q.y() = q.z() = 0.0;
    q.w() = 1.0;
  }

  return  q;
}

Eigen::Matrix4d dq3_by_dq1(const Eigen::Quaterniond &q1)
{
  Eigen::Matrix4d m;

  double  x = q1.x();
  double  y = q1.y();
  double  z = q1.z();
  double  w = q1.w();

  m << w, -x, -y, -z,
       x,  w, -z,  y,
       y,  z,  w, -x,
       z, -y,  x,  w;

  return m;
}

Eigen::Matrix4d dq3_by_dq2(const Eigen::Quaterniond &q2)
{
  Eigen::Matrix4d m;

  double  x = q2.x();
  double  y = q2.y();
  double  z = q2.z();
  double  w = q2.w();

  m << w, -x, -y, -z,
       x,  w,  z, -y,
       y, -z,  w,  x,
       z,  y, -x,  w;

  return m;
}

} // namespace SceneLib2
