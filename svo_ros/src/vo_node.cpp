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
#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

namespace svo {

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;
  bool quit_;
  VoNode();
  ~VoNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
  void runFromTrackingResult();
  istream &read_frame(istream &fin, vector<TrackedFeature> &feature_list, double &timestamp);
};

VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false)
{
  // Start user input thread in parallel thread that listens to console keys
  // if(vk::getParam<bool>("svo/accept_console_user_input", true))
  //   user_input_thread_ = boost::make_shared<vk::UserInputThread>();

  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 1.5708),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // Init VO and start
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

//read three lines from the output_result, i.e. one spatiotemporal window - or one frame.
istream & VoNode::read_frame(istream &fin, vector<TrackedFeature> &feature_list, double &timestamp)
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

void VoNode::runFromTrackingResult()
{
  //Read frame
  // ifstream fin("/home/albert/workSpace/data/output_result_day_200_f.txt");
  ifstream fin("/home/albert/workSpace/data/outdoors_night_result.txt");
  vector<TrackedFeature> feature_list;
  double timestamp = -1.0;
  ros::Rate loop_rate(5);
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
    cv::Mat img;
    visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, timestamp);
    if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
      visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
    if(publish_dense_input_)
      visualizer_.exportToDense(vo_->lastFrame());
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
    // ros::spinOnce();
    loop_rate.sleep();
  }
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  processUserActions();
  vo_->addImage(img, msg->header.stamp.toSec());
  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    // case 'q':
    //   quit_ = true;
    //   printf("SVO user input: QUIT\n");
    //   break;
    // case 'r':
    //   vo_->reset();
    //   printf("SVO user input: RESET\n");
    //   break;
    // case 's':
    //   vo_->start();
    //   printf("SVO user input: START\n");
    //   break;
    // default: ;
  }
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create vo_node" << std::endl;
  svo::VoNode vo_node;

  vo_node.runFromTrackingResult();

  // // subscribe to cam msgs
  // std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));
  // image_transport::ImageTransport it(nh);
  // image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);

  // // subscribe to remote input
  // vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);

  // // start processing callbacks
  // while(ros::ok() && !vo_node.quit_)
  // {
  //   ros::spinOnce();
  //   // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  // }

  printf("SVO terminated.\n");
  return 0;
}
