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

#include <stdexcept>
#include <svo/pose_optimizer.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <vikit/robust_cost.h>
#include <vikit/math_utils.h>

namespace svo {
namespace pose_optimizer {

void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs)
{
  // init
  double chi2(0.0);
  vector<double> chi2_vec_init, chi2_vec_final;
  vk::robust_cost::TukeyWeightFunction weight_function;
  SE3 T_old(frame->T_f_w_);
  Matrix6d A;
  Vector6d b;

  // compute the scale of the error for robust estimation
  std::cout << "NOTICE: Size of fts_ = " << frame->fts_.size() << std::endl;
  std::vector<float> errors; errors.reserve(frame->fts_.size());
  for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
      continue;//NOTE：判断是否已经有深度信息
    Vector2d e = vk::project2d((*it)->f)
               - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);//NOTE:这里就是重投影误差e=u-pi(T*P)
    e *= 1.0 / (1<<(*it)->level);//NOTE:(1<<(*it)->level)表示按照金字塔层数左移（也就是*2），可以省去
    errors.push_back(e.norm());
  }
  if(errors.empty())
    return;
  vk::robust_cost::MADScaleEstimator scale_estimator;
  estimated_scale = scale_estimator.compute(errors);//NOTE：返回errors的中位数

  num_obs = errors.size();
  std::cout << "NOTICE: Size of init_num_obs = " << num_obs << std::endl;
  chi2_vec_init.reserve(num_obs);
  chi2_vec_final.reserve(num_obs);
  double scale = estimated_scale;
  for(size_t iter=0; iter<n_iter; iter++)
  {
    // overwrite scale
    if(iter == 5)
      scale = 0.85/frame->cam_->errorMultiplier2();

    b.setZero();
    A.setZero();
    double new_chi2(0.0);

    // compute residual
    for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
      if((*it)->point == NULL)
        continue;
      Matrix26d J;
      Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);//NOTE：把三维点坐标系转换到当前帧 T*P
      Frame::jacobian_xyz2uv(xyz_f, J);//NOTE：计算重投影误差关于相机位姿李代数的一阶导数，即2*6的雅可比矩阵，见十四讲P164
      Vector2d e = vk::project2d((*it)->f) - vk::project2d(xyz_f);//NOTE:这里就是重投影误差e=u-pi(T*P)
      double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
      e *= sqrt_inv_cov;
      if(iter == 0)
        chi2_vec_init.push_back(e.squaredNorm()); // just for debug
      J *= sqrt_inv_cov;
      double weight = weight_function.value(e.norm()/scale);//NOTE:对于每个feature按照误差计算权重
      A.noalias() += J.transpose()*J*weight;//NOTE:高斯牛顿法里的H=Jt*J,十四讲P112
      b.noalias() -= J.transpose()*e*weight;//NOTE:高斯牛顿法里的g=-Jt*e,十四讲P112
      new_chi2 += e.squaredNorm()*weight;
    }

    // solve linear system
    const Vector6d dT(A.ldlt().solve(b));//NOTE:高斯牛顿法里的求解增量方程Delta x,也就是李代数增量

    // check if error increased
    if((iter > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dT[0]))
    {
      if(verbose)
        std::cout << "it " << iter
                  << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
      frame->T_f_w_ = T_old; // roll-back
      break;
    }

    // update the model
    SE3 T_new = SE3::exp(dT)*frame->T_f_w_;//NOTE:位姿增量更新
    T_old = frame->T_f_w_;
    frame->T_f_w_ = T_new;
    chi2 = new_chi2;
    if(verbose)
      std::cout << "it " << iter
                << "\t Success \t new_chi2 = " << new_chi2
                << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

    // stop when converged
    if(vk::norm_max(dT) <= EPS)//NOTE:更新量足够小，既达到收敛
      break;
  }

  // Set covariance as inverse information matrix. Optimistic estimator!
  const double pixel_variance=1.0;
  frame->Cov_ = pixel_variance*(A*std::pow(frame->cam_->errorMultiplier2(),2)).inverse();

  // Remove Measurements with too large reprojection error
  double reproj_thresh_scaled = reproj_thresh / frame->cam_->errorMultiplier2();
  size_t n_deleted_refs = 0;
  for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
    e *= sqrt_inv_cov;
    chi2_vec_final.push_back(e.squaredNorm());
    if(e.norm() > reproj_thresh_scaled)
    {
      // we don't need to delete a reference in the point since it was not created yet
      (*it)->point->n_failed_pose_estimate_++;
      if((*it)->point->n_failed_pose_estimate_ > 3)
      {
        (*it)->point = NULL;
        ++n_deleted_refs;
      }

    }
  }

  error_init=0.0;
  error_final=0.0;
  if(!chi2_vec_init.empty())
    error_init = sqrt(vk::getMedian(chi2_vec_init))*frame->cam_->errorMultiplier2();
  if(!chi2_vec_final.empty())
    error_final = sqrt(vk::getMedian(chi2_vec_final))*frame->cam_->errorMultiplier2();

  estimated_scale *= frame->cam_->errorMultiplier2();
  if(verbose)
    std::cout << "n deleted obs = " << n_deleted_refs
              << "\t scale = " << estimated_scale
              << "\t error init = " << error_init
              << "\t error end = " << error_final << std::endl;
  num_obs -= n_deleted_refs;
  std::cout << "NOTICE: Size of finial_num_obs = " << num_obs << std::endl;
}

} // namespace pose_optimizer
} // namespace svo
