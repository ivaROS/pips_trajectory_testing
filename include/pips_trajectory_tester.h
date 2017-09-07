#ifndef PIPS_TRAJECTORY_TESTER_H
#define PIPS_TRAJECTORY_TESTER_H

#include <pips/collision_testing/collision_checker.h>
#include <trajectory_generator_ros_interface.h>
#include <pips_trajectory_testing/PipsTrajectoryTesterConfig.h>

//#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>

#include <fstream>
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

  //
  void get_collision_ind(int & ind);
  geometry_msgs::PointStamped get_check_point(const int ind);
};

typedef std::shared_ptr<PipsTrajectory> pips_trajectory_ptr;


typedef std::shared_ptr<TrajectoryGeneratorBridge> TrajectoryGeneratorBridge_ptr;
typedef std::shared_ptr<CollisionChecker> CollisionChecker_ptr;


//Generates a straight line trajectory with a given angle and speed
class angled_straight_traj_func : public traj_func{

    double dep_angle_;
    double v_;

public:
    angled_straight_traj_func( double dep_angle, double v ) : dep_angle_(dep_angle), v_(v) { }

    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[near_identity::XD_IND] = v_*cos( dep_angle_);
        dxdt[near_identity::YD_IND] = v_*sin( dep_angle_);
    }


};


class GenAndTest
{
  std::string name_ = "GenAndTest";
  ros::NodeHandle nh_, pnh_;
  geometry_msgs::TransformStamped depth_base_transform_;

  void configCB(pips_trajectory_testing::PipsTrajectoryTesterConfig &config, uint32_t level);

  typedef dynamic_reconfigure::Server<pips_trajectory_testing::PipsTrajectoryTesterConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  CollisionChecker_ptr cc_;

  ros::Publisher path_pub_, desired_path_pub_, pose_array_pub_;
  int num_frames =0;
  std::string base_frame_id_ = "";
  std_msgs::Header header_;

  bool parallelism_enabled_ = true;

  traj_params_ptr params_;

  // Actually, this should probably be a collision checker-specific thing...
  double min_dist = .05;// Check this distance first, then don't have to evaluate trajectories closer than that

public:

  GenAndTest(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  
  void init();
  void setCollisionChecker(CollisionChecker_ptr cc);
  
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const nav_msgs::Odometry::ConstPtr curr_odom);
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions);
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0);
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, double v0, std_msgs::Header& header);
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0, std_msgs::Header& header);
  std::vector<ni_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0, std_msgs::Header& header, traj_params_ptr params);
  
  int evaluateTrajectory(ni_trajectory_ptr& traj);
  void evaluateTrajectory(pips_trajectory_ptr& traj);

  int evaluateTrajectory(pips_trajectory_msgs::trajectory_points& trajectory);
  
 // std::vector<cv::Mat> generateDepthImages(const std::vector<traj_func_ptr>& trajectory_functions, const state_type& x0, const std_msgs::Header& header);
 // cv::Mat generateTrajectoryDepthImage(const pips_trajectory_ptr& traj);

  TrajectoryGeneratorBridge_ptr traj_gen_bridge_;
  
  static std::vector<traj_func_ptr> getDefaultTrajectoryFunctions();
  static std::vector<traj_func_ptr> getTrajectoryFunctions(const std::vector<double>& dep_angles, double velocity);
  static std::vector<traj_func_ptr> getTrajectoryFunctions(unsigned int num_paths, double velocity);


  static std::vector<traj_func_ptr> getDenseTrajectoryFunctions();
  
//  void saveCollisionCheckData(std::vector<traj_func_ptr>& trajectory_functions);
  
  std::fstream save_check_data_;
  
};


#endif //PIPS_TRAJECTORY_TESTER_H
