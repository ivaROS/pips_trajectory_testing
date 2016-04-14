#ifndef GEN_AND_TEST_H
#define GEN_AND_TEST_H

#include <collision_checker.h>
#include <trajectory_generator_ros_interface.h>

#include <ros/ros.h>

//#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <boost/shared_ptr.hpp>





class PipsTrajectory : public ni_trajectory
{
  int collision_ind_ = -1;
  
public:
  
  bool collides();
  ros::Time time_of_collision();
  void set_collision_ind(int ind);
  geometry_msgs::PointStamped get_collision_point();
  size_t num_states();
};


class GenAndTest
{
  ros::NodeHandle nh_;
  std::vector<cv::Point3d> co_offsets_;
  geometry_msgs::TransformStamped depth_base_transform_;

  CollisionChecker* cc_;
  
  ros::Publisher path_pub_, pose_array_pub_;
  int num_frames =0;
  std::string base_frame_id_ = "";
  std_msgs::Header header_;
  
  bool parallelism_enabled_ = true;
  std::string name_ = "GenAndTest";
  traj_params* params_;
  
public:
  GenAndTest();
  GenAndTest(std::vector<cv::Point3d>& co_offsets, geometry_msgs::TransformStamped& depth_base_transform);
  void constructor();
  
  void updateParams();
  
  void init(ros::NodeHandle& nh);
  void setRobotInfo(std::vector<cv::Point3d>& co_offsets, geometry_msgs::TransformStamped& depth_base_transform);
  void setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  
  std::vector<ni_trajectory*> run(std::vector<traj_func*>& trajectory_functions, const nav_msgs::OdometryPtr curr_odom);
  std::vector<ni_trajectory*> run(std::vector<traj_func*>& trajectory_functions);
  std::vector<ni_trajectory*> run(std::vector<traj_func*>& trajectory_functions, state_type& x0);
  std::vector<ni_trajectory*> run(std::vector<traj_func*>& trajectory_functions, state_type& x0, std_msgs::Header& header);
  
  int evaluateTrajectory(ni_trajectory* traj);
  void evaluateTrajectory(PipsTrajectory* traj);

  int evaluateTrajectory(trajectory_generator::trajectory_points& trajectory);
  TrajectoryGeneratorBridge traj_gen_bridge_;
  
  static std::vector<traj_func*> getDefaultTrajectoryFunctions();
  
};


#endif //GEN_AND_TEST_H
