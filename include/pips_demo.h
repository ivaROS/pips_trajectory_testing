#ifndef TEST_TRAJECTORY_H
#define TEST_TRAJECTORY_H

#include "pips_trajectory_tester.h"


#include <iostream>     // std::cout
#include <algorithm>    // std::min
#include <chrono>

#include <ros/ros.h>
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



#define DEBUG false

class TestTrajectory
{

  private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport* it_;
  image_transport::CameraSubscriber* sub_,depthsubit_;
  image_transport::Publisher* pub_,depthpub_,depthpub2_;
  message_filters::Subscriber<sensor_msgs::Image> depthsub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
  
  ros::Subscriber trigger_sub_;
  
  std::string base_frame_id;

  tf2_ros::Buffer* tfBuffer_;
  tf2_ros::TransformListener* tf_listener_;
  bool firstDepthFrame_, generate;

  std::shared_ptr<HallucinatedRobotModel> robot_model_;
  CollisionChecker* cc_;
  GenAndTest* traj_tester_;


    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo> image_sync_policy;
    typedef message_filters::Synchronizer<image_sync_policy> image_synchronizer;
    boost::shared_ptr<image_synchronizer> synced_images;


  void trigger(const std_msgs::Empty& msg);
  void depthImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  
  public:
  TestTrajectory();
  void init(ros::NodeHandle nh);



};

#endif //TEST_TRAJECTORY_H
