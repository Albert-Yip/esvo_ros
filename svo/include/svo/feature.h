// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_FEATURE_H_
#define SVO_FEATURE_H_

#include <svo/frame.h>

namespace svo {

/// A salient image region that is tracked across frames.
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType {
    CORNER,
    EDGELET
  };

  int feature_ID;       //NOTE:self-added var
  FeatureType type;     //!< Type can be corner or edgelet.
  Frame* frame;         //!< Pointer to frame in which the feature was detected.
  Vector2d px;          //!< Coordinates in pixels on pyramid level 0.
  Vector3d f;           //!< Unit-bearing vector of the feature. NOTE: bearing vector是指FAST角点的方向向量（不对）， 由f(frame->cam_->cam2world(px))可知，是按照d=1把像素坐标转化为相机坐标系上的归一化坐标（P87），注意此处cam2world跟14讲不同，相当于px2cam（depth=1)
  int level;            //!< Image pyramid level where feature was extracted.
  Point* point;         //!< Pointer to 3D point which corresponds to the feature.
  Vector2d grad;        //!< Dominant gradient direction for edglets, normalized.

  Feature(Frame* _frame, const Vector2d& _px, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(frame->cam_->cam2world(px)),/// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {}

  Feature(Frame* _frame, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {}

  Feature(Frame* _frame, Point* _point, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(_point),
    grad(1.0,0.0)
  {}
  Feature(Frame* _frame, int _ft_ID, Point* _point, const Vector2d& _px, const Vector3d& _f) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(0),
    feature_ID(_ft_ID),
    point(_point),
    grad(1.0,0.0)
  {}

  Feature(Frame* _frame, int _ft_ID, const Vector2d& _px) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(frame->cam_->cam2world(px)),/// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
    level(0),
    feature_ID(_ft_ID),
    point(NULL),
    grad(1.0,0.0)
  {}
};

} // namespace svo

#endif // SVO_FEATURE_H_
