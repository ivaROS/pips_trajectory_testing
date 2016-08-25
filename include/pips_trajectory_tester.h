#ifndef PIPS_TRAJECTORY_TESTER_H
#define PIPS_TRAJECTORY_TESTER_H

#include <collision_checker.h>
#include <trajectory_generator_ros_interface.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>

#include <memory>



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

typedef std::shared_ptr<PipsTrajectory> pips_trajectory_ptr;


typedef std::shared_ptr<TrajectoryGeneratorBridge> TrajectoryGeneratorBridge_ptr;
typedef std::shared_ptr<CollisionChecker> CollisionChecker_ptr;

class GenAndTest
{
  ros::NodeHandle nh_;
  std::vector<cv::Point3d> co_offsets_;
  geometry_msgs::TransformStamped depth_base_transform_;



  
  
  CollisionChecker_ptr cc_;
  
  ros::Publisher path_pub_, pose_array_pub_;
  int num_frames =0;
  std::string base_frame_id_ = "";
  std_msgs::Header header_;
  
  bool parallelism_enabled_ = true;
  std::string name_ = "GenAndTest";
  traj_params_ptr params_;
  
public:

  GenAndTest();
  GenAndTest(std::vector<cv::Point3d>& co_offsets, geometry_msgs::TransformStamped& depth_base_transform);
  void constructor();
  
  void updateParams();
  
  void init(ros::NodeHandle& nh);
  void setRobotInfo(std::vector<cv::Point3d>& co_offsets, geometry_msgs::TransformStamped& depth_base_transform);
  void setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const nav_msgs::Odometry::ConstPtr curr_odom);
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions);
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0);
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0, std_msgs::Header& header);
  
  int evaluateTrajectory(ni_trajectory_ptr& traj);
  void evaluateTrajectory(pips_trajectory_ptr& traj);

  int evaluateTrajectory(trajectory_generator::trajectory_points& trajectory);
  


  TrajectoryGeneratorBridge_ptr traj_gen_bridge_;
  
  static std::vector<traj_func_ptr> getDefaultTrajectoryFunctions();
  
};


#endif //PIPS_TRAJECTORY_TESTER_H