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
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include "test_utils.h"

namespace svo {

class BenchmarkNode
{
  vk::AbstractCamera* cam_;
  svo::FrameHandlerMono* vo_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
  void runFromTrackingResult();
  istream &read_frame(istream &fin, vector<TrackedFeature> &feature_list, double &timestamp);
};

BenchmarkNode::BenchmarkNode()
{
  cam_ = new vk::PinholeCamera(346, 260, 194.8, 194.8, 170.2, 127.0);
  // //   projection_matrix:
  // - [194.8389461655774, 0.0, 170.20896993269332, 0.0]
  // - [0.0, 194.8389461655774, 127.00404928416845, 0.0]
  // - [0.0, 0.0, 1.0, 0.0]
  // //  resolution: [346, 260]
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

//read three lines from the output_result, i.e. one spatiotemporal window - or one frame.
istream & BenchmarkNode::read_frame(istream &fin, vector<TrackedFeature> &feature_list, double &timestamp)
{
    TrackedFeature temp_Feature;
    string line;
    // cout<<"Start reading frame (three lines)"<<endl;

    for(int line_counter=0;line_counter<3;line_counter++)
    {
        getline(fin, line);
        istringstream iss(line);
        if(line_counter%3 == 0)//id row
        {
            iss>>timestamp;
            while(iss>>temp_Feature.id)
            {
                // iss>>temp_Feature.id;
                feature_list.push_back(temp_Feature);
            }
        }
        else if(line_counter%3 == 1)//x row
        {
            double temp = -1;
            double temp_x = -1;
            int position = 0;
            iss>>temp;
            if(temp!=timestamp)
            {
                cout<<"ts="<<temp<<", timestamp is wrong!"<<endl;
                // return 0;
            }
            while(iss>>temp_x)
            {
                // iss>>temp_x;
                feature_list.at(position).x = temp_x;
                position++;
            }
        }
        else if(line_counter%3 == 2)//y row
        {
            double temp = -1;
            double temp_y = -1;
            int position = 0;
            iss>>temp;
            if(temp!=timestamp)
            {
                cout<<"ts="<<temp<<", timestamp is wrong!"<<endl;
                // return 0;
            }
            while(iss>>temp_y)
            {
                // iss>>temp_y;
                feature_list.at(position).y = temp_y;
                position++;
            }
        }
    }
    
    return fin;
}

void BenchmarkNode::runFromTrackingResult()
{
  //Read frame
  ifstream fin("/home/albert/workSpace/data/output_result_day_200_f.txt");
  // ifstream fin("/home/albert/workSpace/data/outdoors_night_result.txt");
  vector<TrackedFeature> feature_list;
  double timestamp = -1.0;
  while(read_frame(fin, feature_list, timestamp))
  {
    // cout<<"\nResult "<<":\n";
    for(auto iter=feature_list.begin(); iter!=feature_list.end(); iter++)
    {
        if((*iter).id == 0)
        {
            iter = feature_list.erase(iter);
            iter--;//erase删除后会返回下一个iter
            continue;
        }
        if((*iter).y >= 190)
        {
            iter = feature_list.erase(iter);
            iter--;//erase删除后会返回下一个iter
            continue;
        }
        // cout<<timestamp<<"  "<<(*iter).id<<"  "<<(*iter).x<<"  "<<(*iter).y<<endl;
    }
    // process frame
    vo_->testESVO(feature_list, timestamp);
    //set ts and vector to NULL for the new coming frame
    timestamp = -1.0;
    vector<TrackedFeature>().swap(feature_list);
    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
      std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \t"
                  << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n" << std::endl;

      // access the pose of the camera via vo_->lastFrame()->T_f_w_.
    }
  }
}

void BenchmarkNode::runFromFolder()
{
  for(int img_id = 2; img_id < 188; ++img_id)
  {
    // load image
    std::stringstream ss;
    ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
       << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";
    if(img_id == 2)
      std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());

    // process frame
    vo_->addImage(img, 0.01*img_id);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
    	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \t"
                  << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";

    	// access the pose of the camera via vo_->lastFrame()->T_f_w_.
    }
  }
}

} // namespace svo

int main(int argc, char** argv)
{
  {
    svo::BenchmarkNode benchmark;
    benchmark.runFromTrackingResult();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}

