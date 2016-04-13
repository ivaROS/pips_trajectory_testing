#include "GenAndTest.h"

#include <collision_checker.h>
#include <trajectory_generator_ros_interface.h>

#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>


#define DEBUG true

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



  GenAndTest::GenAndTest()
  {
    GenAndTest::constructor();
  }
  
  GenAndTest::GenAndTest(std::vector<cv::Point3d>& co_offsets, geometry_msgs::TransformStamped& depth_base_transform)
  { 
    GenAndTest::constructor();
    GenAndTest::setRobotInfo(co_offsets, depth_base_transform);
  }
  
  void GenAndTest::constructor()
  {
      traj_gen_bridge_ = *(new TrajectoryGeneratorBridge);
      *params_ = new traj_params(traj_gen_bridge_.copyDefaultParams());
  }
  
  void GenAndTest::setRobotInfo(std::vector<cv::Point3d>& co_offsets, geometry_msgs::TransformStamped& depth_base_transform)
  {    
      cc_ = new CollisionChecker(depth_base_transform, co_offsets, false);
      base_frame_id_ = depth_base_transform.header.frame_id;
      header_.frame_id = base_frame_id_;
  }
  
  void GenAndTest::init(ros::NodeHandle& nh)
  {
  
    nh_ = ros::NodeHandle(nh, "GenAndTest");
    GenAndTest::constructor();

    //Create the various visualization publishers
    path_pub_ = nh_.advertise<nav_msgs::Path>("tested_paths", 5);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collision_points", 5);
    

    
    std::string key;

    if(ros::param::search("enable_parallel_loop", key))
    {
      ros::param::get(key, parallelism_enabled_); 
    }
    
    
    GenAndTest::updateParams();
  }
  
  void GenAndTest::updateParams()
  {
  //Should probably make this dynamnically reconfigurable, or at least cache the results
    nh_.param<double>("tf", (*params_)->tf, 10);
  }

  
  void GenAndTest::setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cc_->setImage(image_msg, info_msg);
    header_.stamp = image_msg->header.stamp;
  }



  std::vector<ni_trajectory*> GenAndTest::run(std::vector<traj_func*>& trajectory_functions)
  {
    state_type x0 = traj_gen_bridge_.initState();
    return GenAndTest::run(trajectory_functions, x0);
  }
  
  //OdometryPtr is not passed as a reference in this instance: we want a copy to be made of the boost::shared_ptr, so that this instance will be constant even if the calling function assigns a new message to curr_odom
  std::vector<ni_trajectory*> GenAndTest::run(std::vector<traj_func*>& trajectory_functions, const nav_msgs::OdometryPtr curr_odom)
  {
    state_type x0 = traj_gen_bridge_.initState(curr_odom);
    std_msgs::Header header;
    header.stamp = curr_odom->header.stamp;
    header.frame_id = curr_odom->child_frame_id;
    return GenAndTest::run(trajectory_functions, x0, header);
  }
  
  
  std::vector<ni_trajectory*> GenAndTest::run(std::vector<traj_func*>& trajectory_functions, state_type& x0)
  {
    return GenAndTest::run(trajectory_functions, x0, header_);
  }
  
  //This is the newest version, meant to be the most flexible
  std::vector<ni_trajectory*> GenAndTest::run(std::vector<traj_func*>& trajectory_functions, state_type& x0, std_msgs::Header& header)
  {
    num_frames=num_frames+1;
    
    ROS_DEBUG_STREAM("[" << name_ << "] Num frames: " << num_frames);
    
    
    ROS_DEBUG_STREAM("[" << name_ << "] Generating Trajectories");

    
    
    
    size_t num_paths = trajectory_functions.size();
    
    //Possibly use boost::shared_ptr to ensure these are properly released?
    std::vector<ni_trajectory*> trajectories(num_paths); //std::vector<boost::shared_ptr<PipsTrajectory*>>
    
    //Start timer
    auto t1 = std::chrono::high_resolution_clock::now();
 
    //Perform trajectory generation and collision detection in parallel if enabled
    //Vectors and arrays must be accessed by indicies to ensure thread safe behavior
    #pragma omp parallel for schedule(dynamic) if(parallelism_enabled_) //schedule(dynamic)
    for(size_t i = 0; i < num_paths; i++)
    {
      PipsTrajectory* traj = new PipsTrajectory();
      traj->header = header;
      traj->params = *params_;
      
      traj->trajpntr = trajectory_functions[i];
      traj->x0_ = x0;
      
      traj_gen_bridge_.generate_trajectory(traj);

      GenAndTest::evaluateTrajectory(traj);

      trajectories[i] = traj;
    }
    
    

    //End timer
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

    
    ROS_DEBUG_STREAM("[" << name_ << "] Generated " << num_paths << " trajectories in " << fp_ms.count() << " ms");
    

    for(size_t i = 0; i < trajectories.size(); i++)
    {
        nav_msgs::PathPtr path;

        path = trajectories[i]->toPathMsg();
        
        path_pub_.publish(path);
    }


    
    return trajectories;
  }
  
  
  void GenAndTest::evaluateTrajectory(PipsTrajectory* traj)
  {
    int collision_ind = GenAndTest::evaluateTrajectory((ni_trajectory*)traj);
    traj->set_collision_ind(collision_ind);
  }
  
  //Test whether trajectory collides
  int GenAndTest::evaluateTrajectory(ni_trajectory* traj)
  {

      for(size_t i = 0; i < traj->num_states(); i++)
      {

        geometry_msgs::Point pt = traj->getPoint(i);    
        double coords[3];
        coords[0] = pt.x;
        coords[1] = pt.y;
        coords[2] = pt.z;
        
        if(cc_->testCollision(coords))
        {
            return i;
            
        }
     
      }

      return -1;

  }

  int GenAndTest::evaluateTrajectory(trajectory_generator::trajectory_points& trajectory)
  {
      
      for(size_t i = 0; i < trajectory.points.size(); i++)
      {

        trajectory_generator::trajectory_point pt = trajectory.points[i];    
        double coords[3];
        coords[0] = pt.x;
        coords[1] = pt.y;
        coords[2] = 0;
        
        if(cc_->testCollision(coords))
        {
            return i;
        }

      }

      return -1;

  }


  std::vector<traj_func*> GenAndTest::getDefaultTrajectoryFunctions()
  {

    //Set trajectory departure angles and speed
    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4};
    double v = .25;

    size_t num_paths = dep_angles.size();
    
    std::vector<traj_func*> trajectory_functions(num_paths);
    
    for(size_t i = 0; i < num_paths; i++)
    {
      double dep_angle = dep_angles[i];
      traj_func* trajptr = new angled_straight_traj_func(dep_angle, v);
      trajectory_functions[i] = trajptr;
    }
    return trajectory_functions;
  }



  bool PipsTrajectory::collides()
  {
    return collision_ind_ >=0;
  }

  ros::Time PipsTrajectory::time_of_collision()
  {
    //ROS_WARN_STREAM(  //warn/close if no collision
    return header.stamp + ros::Duration(times[collision_ind_]);
  }
  
  void PipsTrajectory::set_collision_ind(int ind)
  {
    collision_ind_ = ind;
  }
  
  geometry_msgs::PointStamped PipsTrajectory::get_collision_point()
  {
      //ROS_WARN_STREAM(  //warn/close if no collision
    return ni_trajectory::getPointStamped(collision_ind_);
  }

  size_t PipsTrajectory::num_states()
  {
    if(PipsTrajectory::collides())
    {
      return collision_ind_;
    }
    else
    {
      return ni_trajectory::num_states();
    }
  }
