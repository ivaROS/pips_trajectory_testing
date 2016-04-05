#ifndef GEN_AND_TEST_H
#define GEN_AND_TEST_H

#include <ros/ros.h>

//#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>

#include <collision_checker.h>
#include <trajectory_generator_ros_interface.h>




class GenAndTest
{
  ros::NodeHandle nh_;
  std::vector<cv::Point3d> co_offsets_;
  geometry_msgs::TransformStamped depth_base_transform_;

  CollisionChecker* cc_;
  
  ros::Publisher colliding_path_pub_, noncolliding_path_pub_, pose_array_pub_;
  int num_frames =0;
  bool parallelism_enabled_ = true;
public:
  GenAndTest();
  GenAndTest(std::vector<cv::Point3d> co_offsets, geometry_msgs::TransformStamped& depth_base_transform);

  void init(std::vector<cv::Point3d> co_offsets, geometry_msgs::TransformStamped& depth_base_transform);
  void setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  std::vector<ni_trajectory> run(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, std::string frame_id);
  std::vector<ni_trajectory> run(std::vector<traj_func*> trajectory_functions, std::string frame_id);

  bool evaluateTrajectory(ni_trajectory* traj);

  bool evaluateTrajectory(trajectory_generator::trajectory_points& trajectory);
  TrajectoryGeneratorBridge traj_gen_bridge_;
  
};



#endif //GEN_AND_TEST_H
