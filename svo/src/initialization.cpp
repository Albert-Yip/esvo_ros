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

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/initialization.h>
#include <svo/feature_detection.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>

namespace svo {
namespace initialization {

InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
  reset();
  detectFeatures(frame_ref, px_ref_, f_ref_);
  if(px_ref_.size() < 100)
  {
    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
    return FAILURE;
  }
  frame_ref_ = frame_ref;
  px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());//在指定位置loc前插入区间[start, end)的所有元素. 这里是把用于第二帧的current的特征点位置先初始化为ref第一帧的，随后用光流法更新
  return SUCCESS;
}

InitResult KltHomographyInit::addFirst_TFrame(FramePtr frame_ref)
{
  reset();
  feature_list_ref_.assign(frame_ref->feature_list_.begin(),frame_ref->feature_list_.end());
  frame_ref_ = frame_ref;
  SVO_INFO_STREAM("Add First Frame successfully, there are "<<feature_list_ref_.size()<<" features");
  return SUCCESS;
}

InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
  trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
  SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");

  if(disparities_.size() < Config::initMinTracked())
    return FAILURE;

  double disparity = vk::getMedian(disparities_);
  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
  if(disparity < Config::initMinDisparity())
    return NO_KEYFRAME;

  computeHomography(
      f_ref_, f_cur_,
      frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
      inliers_, xyz_in_cur_, T_cur_from_ref_);
  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");//triangulate all features and compute reprojection errors and inliers: output->inliers_, xyz_in_cur_, T_cur_from_ref_

  if(inliers_.size() < Config::initMinInliers())
  {
    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
    return FAILURE;
  }

  // Rescale the map such that the mean scene depth is equal to the specified scale
  vector<double> depth_vec;
  for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());
  double scene_depth_median = vk::getMedian(depth_vec);
  double scale = Config::mapScale()/scene_depth_median;
  frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
  frame_cur->T_f_w_.translation() =
      -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

  // For each inlier create 3D point and add feature in both frames
  SE3 T_world_cur = frame_cur->T_f_w_.inverse();
  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {
    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
      Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
      Point* new_point = new Point(pos);

      Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
      frame_cur->addFeature(ftr_cur);
      new_point->addFrameRef(ftr_cur);

      Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
      frame_ref_->addFeature(ftr_ref);
      new_point->addFrameRef(ftr_ref);
    }
  }
  return SUCCESS;
}

InitResult KltHomographyInit::addSecond_TFrame(FramePtr frame_cur)
{
  
  //NOTE:在这做的事情：KLT光流法track了features，triangulate all features and compute reprojection errors and inliers.
  //Output: inliers_(对应内点feature-id的vector), xyz_in_cur_(对应内点feature三维位置的vector), T_cur_from_ref_
  //因为已经做好tracking，而且ransac已经在tracking中完成，inliers设为全部重复出现特征的id，并估计T和三维位置
  feature_list_cur_.assign(frame_cur->feature_list_.begin(),frame_cur->feature_list_.end());
  vector<Vector3d> f_ref, f_cur;

  //temp fix for feature_list_ to px, NOTE:因为feature_list_与inliers对应的index不是一致的，inliers对应的是一一配对好的point_lists
  vector<cv::Point2f> points_ref;
  vector<cv::Point2f> points_cur;
  cv::Point2f temp_point;
  for(int i=0; i<feature_list_ref_.size(); i++)
  {
    for(int j=0; j<feature_list_cur_.size(); j++)
    {
      if(feature_list_ref_[i].id==feature_list_cur_[j].id)
      {
        ft_ID_list_.push_back(feature_list_cur_[j].id);
        temp_point.x = feature_list_ref_[i].x;
        temp_point.y = feature_list_ref_[i].y;
        points_ref.push_back(temp_point);
        temp_point.x = feature_list_cur_[j].x;
        temp_point.y = feature_list_cur_[j].y;
        points_cur.push_back(temp_point);
        break;
      }
    }
  }

  poseEstimate_triangulation(points_ref, points_cur, f_ref, f_cur, inliers_, xyz_in_cur_, T_cur_from_ref_);

  SVO_INFO_STREAM("Init: Epipolar RANSAC "<<inliers_.size()<<" inliers.");
  //triangulate all features and compute reprojection errors and inliers: output->inliers_, xyz_in_cur_, T_cur_from_ref_

  if(inliers_.size() < Config::initMinInliers())
  {
    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
    return FAILURE;
  }
  // Rescale the map such that the mean scene depth is equal to the specified scale
  vector<double> depth_vec;
  for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());
  double scene_depth_median = vk::getMedian(depth_vec);
  double scale = Config::mapScale()/scene_depth_median;//map_scale(1.0),
  frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
  frame_cur->T_f_w_.translation() =
      -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));


  // For each inlier create 3D point and add feature in both frames
  SE3 T_world_cur = frame_cur->T_f_w_.inverse();
  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {
    //NOTE:因为feature_list_与inliers对应的index不是一致的，inliers对应的是一一配对好的points_ref,cur
    // Vector2d px_cur(feature_list_cur_[*it].x, feature_list_cur_[*it].y);//Seg Fault,svo::Frame*]: Assertion `px != 0' failed.
    // Vector2d px_ref(feature_list_ref_[*it].x, feature_list_ref_[*it].y);
    Vector2d px_cur(points_cur[*it].x, points_cur[*it].y);
    Vector2d px_ref(points_ref[*it].x, points_ref[*it].y);
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
      Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
      Point* new_point = new Point(pos);

      Feature* ftr_cur(new Feature(frame_cur.get(), ft_ID_list_[*it], new_point, px_cur, f_cur[*it]));//Seg Fault
      frame_cur->addFeature(ftr_cur);
      new_point->addFrameRef(ftr_cur);

      Feature* ftr_ref(new Feature(frame_ref_.get(), ft_ID_list_[*it], new_point, px_ref, f_ref[*it]));
      frame_ref_->addFeature(ftr_ref);
      new_point->addFrameRef(ftr_ref);
    }
  }
  vector<int>().swap(ft_ID_list_);
  return SUCCESS;
}

void KltHomographyInit::reset()
{
  feature_list_cur_.clear();
  px_cur_.clear();
  frame_ref_.reset();
}

void detectFeatures(
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec)
{
  // Features new_features;
  // feature_detection::FastDetector detector(
  //     frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());
  // detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

  // // now for all maximum corners, initialize a new seed
  // px_vec.clear(); px_vec.reserve(new_features.size());
  // f_vec.clear(); f_vec.reserve(new_features.size());
  // std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){//NOTE: [&] capture all variables within scope by reference,这里的作用就是把new_features里的feature分别复制给ftr，用于后续pushback赋值)
  //   px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
  //   f_vec.push_back(ftr->f);
  //   delete ftr;
  // });
}

void trackKlt(
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities)
{
  const double klt_win_size = 30.0;
  const int klt_max_iter = 30;
  const double klt_eps = 0.001;
  vector<uchar> status;
  vector<float> error;
  vector<float> min_eig_vec;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
  cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
                           px_ref, px_cur,
                           status, error,
                           cv::Size2i(klt_win_size, klt_win_size),
                           4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

  vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
  vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
  vector<Vector3d>::iterator f_ref_it = f_ref.begin();
  f_cur.clear(); f_cur.reserve(px_cur.size());
  disparities.clear(); disparities.reserve(px_cur.size());
  for(size_t i=0; px_ref_it != px_ref.end(); ++i)
  {
    if(!status[i])
    {
      px_ref_it = px_ref.erase(px_ref_it);
      px_cur_it = px_cur.erase(px_cur_it);
      f_ref_it = f_ref.erase(f_ref_it);
      continue;
    }
    f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
    disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    ++px_ref_it;
    ++px_cur_it;
    ++f_ref_it;
  }
}

void computeHomography(
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref)
{
  vector<Vector2d, aligned_allocator<Vector2d> > uv_ref(f_ref.size());
  vector<Vector2d, aligned_allocator<Vector2d> > uv_cur(f_cur.size());
  for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
  {
    uv_ref[i] = vk::project2d(f_ref[i]);
    uv_cur[i] = vk::project2d(f_cur[i]);
  }
  vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
  Homography.computeSE3fromMatches();
  vector<int> outliers;
  vk::computeInliers(f_cur, f_ref,
                     Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);
  T_cur_from_ref = Homography.T_c2_from_c1;
}

void poseEstimate_triangulation(
    const vector<cv::Point2f> points1,     
    const vector<cv::Point2f> points2,
    vector<Vector3d>& f_ref, 
    vector<Vector3d>& f_cur,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref)
{
  // vector<cv::Point2f> points1;
  // vector<cv::Point2f> points2;
  // //-- 把匹配点转换为vector<Point2f>的形式
  // cv::Point2f temp_point;
  // int match_counter = 0;
  
  // for(int i=0; i<feature_list_ref_.size(); i++)
  // {
  //     for(int j=0; j<feature_list_cur_.size(); j++)
  //     {
  //         if(feature_list_ref_[i].id==feature_list_cur_[j].id)
  //         {
  //             temp_point.x = feature_list_ref_[i].x;
  //             temp_point.y = feature_list_ref_[i].y;
  //             points1.push_back(temp_point);
  //             temp_point.x = feature_list_cur_[j].x;
  //             temp_point.y = feature_list_cur_[j].y;
  //             points2.push_back(temp_point);
  //             break;
  //         }
  //     }
  // }

  //-- 计算本质矩阵
  cv::Point2d principal_point ( 170.2, 127.0 );	//相机光心, TUM dataset标定值
  double focal_length = 194.8;			//相机焦距, TUM dataset标定值
  cv::Mat essential_matrix;
  essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
  // cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;
  //-- 从本质矩阵中恢复旋转和平移信息.
  cv::Mat R, t;
  recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
  // cout<<"R is "<<endl<<R<<endl;
  // cout<<"t is "<<endl<<t<<endl;
  // SVO_INFO_STREAM("Trying to convert Mat to Eigen");
  // 将像素坐标转换至相机归一化坐标
  cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 194.8, 0, 170.2, 0, 194.8, 127.0, 0, 0, 1);

  for (int  i=0;i<points1.size();i++ )
  {
    Vector3d temp_point;
    temp_point[0] = (points1[i].x - K.at<double>(0,2)) / K.at<double>(0,0);
    temp_point[1] = (points1[i].y - K.at<double>(1,2)) / K.at<double>(1,1);
    temp_point[2] = 1;
    f_ref.push_back ( temp_point );
    temp_point[0] = (points2[i].x - K.at<double>(0,2)) / K.at<double>(0,0);
    temp_point[1] = (points2[i].y - K.at<double>(1,2)) / K.at<double>(1,1);
    temp_point[2] = 1;
    f_cur.push_back ( temp_point );
  }
  
  vector<int> outliers;
  //NOTE:这里有个问题是基于opencv的mat和基于Eigen的sophus的SE3的转换
  Matrix3d rotation_matrix;
  Vector3d translation_vec;
  cv::cv2eigen(R,rotation_matrix);
  cv::cv2eigen(t,translation_vec);
  vk::computeInliers(f_cur, f_ref,
                     rotation_matrix, translation_vec,
                     Config::poseOptimThresh(), focal_length,
                     xyz_in_cur, inliers, outliers);
  T_cur_from_ref=SE3(rotation_matrix, translation_vec);
}


} // namespace initialization
} // namespace svo
