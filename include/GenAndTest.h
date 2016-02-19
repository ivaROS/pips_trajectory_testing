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
  TrajectoryGeneratorBridge traj_gen_bridge_;
  
  ros::Publisher colliding_path_pub_, noncolliding_path_pub_, pose_array_pub_;


public:
  GenAndTest(std::vector<cv::Point3d> co_offsets, geometry_msgs::TransformStamped& depth_base_transform);

  void run(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg, geometry_msgs::TransformStamped& base_odom_transform);
               
  bool evaluateTrajectory(ni_trajectory& traj);


};




