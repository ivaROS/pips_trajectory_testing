#ifndef TEST_TRAJECTORY_H
#define TEST_TRAJECTORY_H

#include <ros/ros.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::min
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <boost/foreach.hpp>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>
#include <kobuki_msgs/ButtonEvent.h>
#include "GenAndTest.h"

#define DEBUG false

class PipsAvoidance
{

  private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport* it_;
  image_transport::CameraSubscriber* sub_,depthsubit_;
  image_transport::Publisher* pub_,depthpub_,depthpub2_;
  message_filters::Subscriber<sensor_msgs::Image> depthsub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
  
  ros::Subscriber button_subscriber_;
  
  std::string base_frame_id_, odom_frame_id_;

  tf2_ros::Buffer* tfBuffer_;
  bool ready_;

  std::vector<cv::Point3d> co_offsets_;
  CollisionChecker* cc_;
  GenAndTest* traj_tester_;


    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo> image_sync_policy;
    typedef message_filters::Synchronizer<image_sync_policy> image_synchronizer;
    boost::shared_ptr<image_synchronizer> synced_images;

  void depthImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
               
  void buttonCB(const kobuki_msgs::ButtonEventPtr msg);
  
  public:
  TestTrajectory();
  void init(ros::NodeHandle nh);
  bool ready();
  bool PipsAvoidance::checkCurrentTrajectory(trajectory_generator::trajectory_points& currentTrajectory, size_t curr_index)



};

#endif //TEST_TRAJECTORY_H
