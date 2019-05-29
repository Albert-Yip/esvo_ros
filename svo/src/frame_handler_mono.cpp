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
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/pose_optimizer.h>
#include <svo/sparse_img_align.h>
#include <vikit/performance_monitor.h>
#include <svo/depth_filter.h>
#ifdef USE_BUNDLE_ADJUSTMENT
#include <svo/bundle_adjustment.h>
#endif

namespace svo {

FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam) :
  FrameHandlerBase(),
  cam_(cam),
  reprojector_(cam_, map_),
  depth_filter_(NULL)
{
  initialize();
}

void FrameHandlerMono::initialize()
{
  feature_detection::DetectorPtr feature_detector(
      new feature_detection::FastDetector(
          cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels()));
  DepthFilter::callback_t depth_filter_cb = boost::bind(
      &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
  depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
  depth_filter_->startThread();
}

FrameHandlerMono::~FrameHandlerMono()
{
  delete depth_filter_;
}

void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)
{
  if(!startFrameProcessingCommon(timestamp))
    return;

  // some cleanup from last iteration, can't do before because of visualization
  core_kfs_.clear();
  overlap_kfs_.clear();

  // create new frame
  SVO_START_TIMER("pyramid_creation");
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  SVO_STOP_TIMER("pyramid_creation");

  // process frame
  UpdateResult res = RESULT_FAILURE;
  if(stage_ == STAGE_DEFAULT_FRAME)
    res = processFrame();
  else if(stage_ == STAGE_SECOND_FRAME)
    res = processSecondFrame();
  else if(stage_ == STAGE_FIRST_FRAME)
    res = processFirstFrame();
  else if(stage_ == STAGE_RELOCALIZING)
    res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
                          map_.getClosestKeyframe(last_frame_));

  // set last frame
  last_frame_ = new_frame_;
  new_frame_.reset();

  // finish processing
  finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
}

FramePtr FrameHandlerMono::lastFrame() { return last_frame_; }
const set<FramePtr>& FrameHandlerMono::coreKeyframes() { return core_kfs_; }
const vector<cv::Point2f>& FrameHandlerMono::initFeatureTrackRefPx() const { return klt_homography_init_.px_ref_; }
const vector<cv::Point2f>& FrameHandlerMono::initFeatureTrackCurPx() const { return klt_homography_init_.px_cur_; }
DepthFilter* FrameHandlerMono::depthFilter() const { return depth_filter_; }

FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
{
  new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());
  if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
    return RESULT_NO_KEYFRAME;
  new_frame_->setKeyframe();
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_SECOND_FRAME;
  SVO_INFO_STREAM("Init: Selected first frame.");
  return RESULT_IS_KEYFRAME;
}

FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
{
  initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);//NOTE:这里做了初始化位姿，利用的是计算单应矩阵H;同时利用三角测量计算深度
  if(res == initialization::FAILURE)
    return RESULT_FAILURE;
  else if(res == initialization::NO_KEYFRAME)
    return RESULT_NO_KEYFRAME;

  // two-frame bundle adjustment
#ifdef USE_BUNDLE_ADJUSTMENT
  ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
#endif

  new_frame_->setKeyframe();
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);//NOTE:这里初始化了深度地图，计算深度是之前的三角化，后续会有深度滤波器进行优化

  // add frame to map
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
  klt_homography_init_.reset();
  SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
  return RESULT_IS_KEYFRAME;
}

FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
{
  // Set initial pose TODO use prior
  new_frame_->T_f_w_ = last_frame_->T_f_w_;

  // sparse image align
  SVO_START_TIMER("sparse_img_align");
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);//NOTE: 第一步sparse image align，是基于class实现的，run是主要操作
  SVO_STOP_TIMER("sparse_img_align");
  SVO_LOG(img_align_n_tracked);
  SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

  // map reprojection & feature alignment
  SVO_START_TIMER("reproject");
  reprojector_.reprojectMap(new_frame_, overlap_kfs_);//NOTE: 第二步feature_align is also finished here
  SVO_STOP_TIMER("reproject");
  const size_t repr_n_new_references = reprojector_.n_matches_;
  const size_t repr_n_mps = reprojector_.n_trials_;
  SVO_LOG2(repr_n_mps, repr_n_new_references);
  SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);
  if(repr_n_new_references < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    tracking_quality_ = TRACKING_INSUFFICIENT;
    return RESULT_FAILURE;
  }

  //在这之前new_frame_已经有了：图像金字塔vec<mat> img_pyr_，检测或跟踪到的List<*Feature> fts_ 包含好2D粗略3D位置，粗略估计的位姿T_f_w_
  //Pose和Structure都是使用高斯牛顿法进行优化，参考十四讲P112、164
  // pose optimization
  SVO_START_TIMER("pose_optimizer");
  size_t sfba_n_edges_final;
  double sfba_thresh, sfba_error_init, sfba_error_final;
  pose_optimizer::optimizeGaussNewton(
      Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
      new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);//NOTE: 第三步pose optimization
  SVO_STOP_TIMER("pose_optimizer");
  SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
  if(sfba_n_edges_final < 20)
    return RESULT_FAILURE;

  // structure optimization
  SVO_START_TIMER("point_optimizer");
  optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
  SVO_STOP_TIMER("point_optimizer");

  //在此之后，new_frame_的feature的空间点new_frame_->fts_ （list）、以及位姿new_frame_->T_f_w_都经过优化

  // select keyframe
  core_kfs_.insert(new_frame_);
  setTrackingQuality(sfba_n_edges_final);
  if(tracking_quality_ == TRACKING_INSUFFICIENT)
  {
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    return RESULT_FAILURE;
  }
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);//遍历features，求已有三维坐标的深度d求最小值和中位数
  if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)//NOTE:判断是否需要新的关键帧
  {
    depth_filter_->addFrame(new_frame_);//NOTE:不是关键帧，更新深度滤波器
    return RESULT_NO_KEYFRAME;
  }
  new_frame_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");//NOTE:后边是需要插入新的关键帧相关操作

  // new keyframe selected
  for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
    if((*it)->point != NULL)
      (*it)->point->addFrameRef(*it);
  map_.point_candidates_.addCandidatePointToFrame(new_frame_);

  // optional bundle adjustment
#ifdef USE_BUNDLE_ADJUSTMENT
  if(Config::lobaNumIter() > 0)
  {
    SVO_START_TIMER("local_ba");
    setCoreKfs(Config::coreNKfs());
    size_t loba_n_erredges_init, loba_n_erredges_fin;
    double loba_err_init, loba_err_fin;
    ba::localBA(new_frame_.get(), &core_kfs_, &map_,
                loba_n_erredges_init, loba_n_erredges_fin,
                loba_err_init, loba_err_fin);
    SVO_STOP_TIMER("local_ba");
    SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
    SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                     "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
  }
#endif

  // init new depth-filters
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);//NOT:这里是深度滤波器插入关键帧的主要操作，init Seeds，把当前帧监测到的特征点用来初始化种子，已有feature直接占据像素网格，新feature做seeds_.push_back，均值为1.0/depth_mean，标准差为1.0/（6*depth_min）。
  //注意只在插入新的关键帧时会做feature detect，普通帧靠 光流跟踪（pose） 和 极线搜索(Depth Filter)

  // if limited number of keyframes, remove the one furthest apart
  if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
  {
    FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
    depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
    map_.safeDeleteFrame(furthest_frame);
  }

  // add keyframe to map
  map_.addKeyframe(new_frame_);

  return RESULT_IS_KEYFRAME;
}

FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
    const SE3& T_cur_ref,
    FramePtr ref_keyframe)
{
  SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");
  if(ref_keyframe == nullptr)
  {
    SVO_INFO_STREAM("No reference keyframe.");
    return RESULT_FAILURE;
  }
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);
  if(img_align_n_tracked > 30)
  {
    SE3 T_f_w_last = last_frame_->T_f_w_;
    last_frame_ = ref_keyframe;
    FrameHandlerMono::UpdateResult res = processFrame();
    if(res != RESULT_FAILURE)
    {
      stage_ = STAGE_DEFAULT_FRAME;
      SVO_INFO_STREAM("Relocalization successful.");
    }
    else
      new_frame_->T_f_w_ = T_f_w_last; // reset to last well localized pose
    return res;
  }
  return RESULT_FAILURE;
}

bool FrameHandlerMono::relocalizeFrameAtPose(
    const int keyframe_id,
    const SE3& T_f_kf,
    const cv::Mat& img,
    const double timestamp)
{
  FramePtr ref_keyframe;
  if(!map_.getKeyframeById(keyframe_id, ref_keyframe))
    return false;
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
  if(res != RESULT_FAILURE) {
    last_frame_ = new_frame_;
    return true;
  }
  return false;
}

void FrameHandlerMono::resetAll()
{
  resetCommon();
  last_frame_.reset();
  new_frame_.reset();
  core_kfs_.clear();
  overlap_kfs_.clear();
  // depth_filter_->reset();
}

void FrameHandlerMono::setFirstFrame(const FramePtr& first_frame)
{
  resetAll();
  last_frame_ = first_frame;
  last_frame_->setKeyframe();
  map_.addKeyframe(last_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
}

bool FrameHandlerMono::needNewKf(double scene_depth_mean)
{
  for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
  {
    Vector3d relpos = new_frame_->w2f(it->first->pos());
    if(fabs(relpos.x())/scene_depth_mean < Config::kfSelectMinDist() &&
       fabs(relpos.y())/scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
       fabs(relpos.z())/scene_depth_mean < Config::kfSelectMinDist()*1.3)
      return false;
  }
  return true;
}

void FrameHandlerMono::setCoreKfs(size_t n_closest)
{
  size_t n = min(n_closest, overlap_kfs_.size()-1);
  std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin()+n, overlap_kfs_.end(),
                    boost::bind(&pair<FramePtr, size_t>::second, _1) >
                    boost::bind(&pair<FramePtr, size_t>::second, _2));
  std::for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i){ core_kfs_.insert(i.first); });
}

///Self added functions

FrameHandlerMono::UpdateResult FrameHandlerMono::processFirst_TFrame()
{
  if(stage_ == STAGE_FIRST_FRAME)
    new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());
  else if(stage_ == STAGE_RELOCALIZING)
    new_frame_->T_f_w_ = last_frame_->T_f_w_;
  if(klt_homography_init_.addFirst_TFrame(new_frame_) == initialization::FAILURE)
    return RESULT_NO_KEYFRAME;
  new_frame_->setKeyframe();
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_SECOND_FRAME;
  SVO_INFO_STREAM("Init: Selected first frame.");
  return RESULT_IS_KEYFRAME;
}


FrameHandlerMono::UpdateResult FrameHandlerMono::processSecond_TFrame()
{
  initialization::InitResult res = klt_homography_init_.addSecond_TFrame(new_frame_);//NOTE:这里做了初始化位姿，利用的是计算本质矩阵E;同时利用三角测量计算深度
  if(res == initialization::FAILURE)
    return RESULT_FAILURE;
  else if(res == initialization::NO_KEYFRAME)
    return RESULT_NO_KEYFRAME;

#ifdef USE_BUNDLE_ADJUSTMENT
  ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
#endif

  new_frame_->setKeyframe();
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);//NOTE:这里初始化了深度地图，后续会有深度滤波器进行优化

  // add frame to map
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
  klt_homography_init_.reset();
  SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
  return RESULT_IS_KEYFRAME;    
}


FrameHandlerMono::UpdateResult FrameHandlerMono::process_TFrame()
{
  // Set initial pose TODO use prior
  new_frame_->T_f_w_ = last_frame_->T_f_w_;

  //给new_frame_的Features赋值，这里的features也不是前后帧11对应的
  FramePtr last_KF = map_.keyframes_.back();
  for(const auto& last_KF_ft : last_KF->fts_)
  {
    for(const auto& cur_tr_ft : new_frame_->feature_list_)
    {
      if(last_KF_ft->feature_ID == cur_tr_ft.id)
      {
        Vector2d px(cur_tr_ft.x,cur_tr_ft.y);
        Feature* ftr_cur(new Feature(new_frame_.get(),last_KF_ft->feature_ID, px));
        new_frame_->addFeature(ftr_cur);
        ftr_cur->point = last_KF_ft->point;//设置3维信息
        break;
      }
    }
  }

  //Pose和Structure都是使用高斯牛顿法进行优化，参考十四讲P112、164
  // pose optimization
  SVO_START_TIMER("pose_optimizer");
  size_t sfba_n_edges_final;
  double sfba_thresh, sfba_error_init, sfba_error_final;
  pose_optimizer::optimizeGaussNewton(
      Config::poseOptimThresh(), Config::poseOptimNumIter(), true,
      new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);//NOTE: 第三步pose optimization
  SVO_STOP_TIMER("pose_optimizer");
  SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
  if(sfba_n_edges_final < 20)
    return RESULT_FAILURE;

  // structure optimization
  SVO_START_TIMER("point_optimizer");
  optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
  SVO_STOP_TIMER("point_optimizer");

  //在此之后，new_frame_的feature的空间点new_frame_->fts_ （list）、以及位姿new_frame_->T_f_w_都经过优化
 
  // select keyframe
  core_kfs_.insert(new_frame_);
  setTrackingQuality(sfba_n_edges_final);
  if(tracking_quality_ == TRACKING_INSUFFICIENT)
  {
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    return RESULT_FAILURE;
  }

  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);//遍历features，求已有三维坐标的深度d求最小值和中位数
  // if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)//NOTE:判断是否需要新的关键帧
  if(!needNewKf_T(sfba_n_edges_final))
  {
    depth_filter_->addTFrame(new_frame_);//NOTE:不是关键帧，更新深度滤波器
    return RESULT_NO_KEYFRAME;
  }

  new_frame_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");//NOTE:后边是需要插入新的关键帧相关操作

  // new keyframe selected
  for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
    if((*it)->point != NULL)
      (*it)->point->addFrameRef(*it);
  map_.point_candidates_.addCandidatePointToFrame(new_frame_);

  // optional bundle adjustment
#ifdef USE_BUNDLE_ADJUSTMENT
  if(Config::lobaNumIter() > 0)
  {
    SVO_START_TIMER("local_ba");
    setCoreKfs(Config::coreNKfs());
    size_t loba_n_erredges_init, loba_n_erredges_fin;
    double loba_err_init, loba_err_fin;
    ba::localBA(new_frame_.get(), &core_kfs_, &map_,
                loba_n_erredges_init, loba_n_erredges_fin,
                loba_err_init, loba_err_fin);
    SVO_STOP_TIMER("local_ba");
    SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
    SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                     "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
  }
#endif

  // init new depth-filters
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);//NOT:这里是深度滤波器插入关键帧的主要操作，init Seeds，把当前帧监测到的特征点用来初始化种子，已有feature直接占据像素网格，新feature做seeds_.push_back，均值为1.0/depth_mean，标准差为1.0/（6*depth_min）。
  //注意只在插入新的关键帧时会做feature detect，普通帧靠 光流跟踪（pose） 和 极线搜索(Depth Filter)

  // if limited number of keyframes, remove the one furthest apart
  if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
  {
    FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
    depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
    map_.safeDeleteFrame(furthest_frame);
  }

  // add keyframe to map
  map_.addKeyframe(new_frame_);

  return RESULT_IS_KEYFRAME;
}



bool FrameHandlerMono::needNewKf_T(const size_t num_observations)
{
  int thres = 150;
  if(num_observations < thres)
  {
    SVO_WARN_STREAM("Tracking "<<num_observations<<" features, less than "<< thres <<" features! Trying to insert new Keyframe!");
    return true;
  }
  return false;
}


void FrameHandlerMono::testESVO(const vector<TrackedFeature> &feature_list, double timestamp)
{
  if(!startFrameProcessingCommon(timestamp))
    return;
  // some cleanup from last iteration, can't do before because of visualization
  core_kfs_.clear();
  overlap_kfs_.clear();

  // create new frame
  new_frame_.reset(new Frame(cam_, feature_list, timestamp));
  
  // process frame
  UpdateResult res = RESULT_FAILURE;
  if(stage_ == STAGE_DEFAULT_FRAME)
    res = process_TFrame();
  else if(stage_ == STAGE_SECOND_FRAME)
    res = processSecond_TFrame();
  else if(stage_ == STAGE_FIRST_FRAME || stage_ == STAGE_RELOCALIZING)
    res = processFirst_TFrame();
  // else if(stage_ == STAGE_RELOCALIZING)
  //   res = relocalize_TFrame();  

  // set last frame
  last_frame_ = new_frame_;
  new_frame_.reset();

  // finish processing
  std::cout<<"frame pointer = "<<last_frame_.get()<<std::endl;
  finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
}

} // namespace svo
