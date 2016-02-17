#include <ros/ros.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::min


#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>




#include <collision_checker.h>
#include <trajectory_generator_ros_interface.h>




class GenAndTest
{

  std::vector<cv::Point3d> co_offsets_;
  geometry_msgs::TransformStampedConstPtr depth_base_transform_;

  CollisionChecker* cc_;
  TrajectoryGeneratorBridge traj_gen_bridge_;


public:
  GenAndTest(std::vector<cv::Point3d> co_offsets, geometry_msgs::TransformStampedConstPtr& depth_base_transform);

  void run(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg, const geometry_msgs::TransformStampedConstPtr& base_odom_transform);
               
  bool evaluateTrajectory(ni_trajectory& traj, geometry_msgs::TransformStamped base_odom_transform );


};




