#include "pips_trajectory_tester.h"

#include <collision_checker.h>
#include <trajectory_generator_ros_interface.h>
#include <pips_trajectory_testing/PipsTrajectoryTesterConfig.h>

#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>

#include <memory>


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
      traj_gen_bridge_ = std::make_shared<TrajectoryGeneratorBridge>();
      params_ = std::make_shared<traj_params>(traj_gen_bridge_->getDefaultParams());
  }
  
  void GenAndTest::setRobotInfo(std::vector<cv::Point3d>& co_offsets, geometry_msgs::TransformStamped& depth_base_transform)
  {    
      cc_ = std::make_shared<CollisionChecker>(depth_base_transform, co_offsets, true);
      base_frame_id_ = depth_base_transform.child_frame_id; //.header.frame_id;
      header_.frame_id = base_frame_id_;
  }
  
  void GenAndTest::init(ros::NodeHandle& nh)
  {
  
    nh_ = ros::NodeHandle(nh, "GenAndTest");
    //pnh_ = ros::NodeHandle(nh_, "~");
    GenAndTest::constructor();

    //Create the various visualization publishers
    path_pub_ = nh_.advertise<nav_msgs::Path>("tested_paths", 5);
    desired_path_pub_ = nh_.advertise<nav_msgs::Path>("desired_paths", 5);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collision_points", 5);
    
    reconfigure_server_.reset( new ReconfigureServer(nh_));
    reconfigure_server_->setCallback(boost::bind(&GenAndTest::configCB, this, _1, _2));
    
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
    nh_.param<double>("tf", params_->tf, 10);
  }

  void GenAndTest::configCB(pips_trajectory_testing::PipsTrajectoryTesterConfig &config, uint32_t level) {
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request: tf=" << config.tf << ", parallelism=" << (config.parallelism?"True":"False")); 
    
    parallelism_enabled_ = config.parallelism;
    params_->tf = config.tf;
    params_->dt = config.dt;
    
    params_->cp = config.cp;
    params_->cd = config.cd;
    params_->cl = config.cl;
    params_->eps = config.eps;
    
    params_->a_max = config.a_max;
    params_->v_max = config.v_max;
    params_->w_max = config.w_max;
    params_->w_dot_max = config.w_dot_max;
    
  }
  
  void GenAndTest::setImage(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    cc_->setImage(image_msg, info_msg);
    header_.stamp = image_msg->header.stamp;
  }



  std::vector<ni_trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions)
  {
    state_type x0 = traj_gen_bridge_->initState();
    return GenAndTest::run(trajectory_functions, x0);
  }
  
  //OdometryPtr is not passed as a reference in this instance: we want a copy to be made of the boost::shared_ptr, so that this instance will be constant even if the calling function assigns a new message to curr_odom
  std::vector<ni_trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions, const nav_msgs::Odometry::ConstPtr curr_odom)
  {
    state_type x0 = traj_gen_bridge_->initState(curr_odom);
    std_msgs::Header header;
    header.stamp = curr_odom->header.stamp;
    header.frame_id = curr_odom->child_frame_id;
    return GenAndTest::run(trajectory_functions, x0, header);
  }
  
  
  std::vector<ni_trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0)
  {
    return GenAndTest::run(trajectory_functions, x0, header_);
  }
  
  //This is the lowest level version that is actually run; the rest are for convenience
  std::vector<ni_trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0, std_msgs::Header& header)
  {
    num_frames=num_frames+1;
    
    ROS_DEBUG_STREAM_NAMED(name_, "Num frames: " << num_frames);
    
    
    ROS_DEBUG_STREAM_NAMED(name_, "Generating Trajectories");

    
    
    
    size_t num_paths = trajectory_functions.size();
    
    std::vector<ni_trajectory_ptr> trajectories(num_paths); //std::vector<boost::shared_ptr<PipsTrajectory*>>
    
    //Start timer
    auto t1 = std::chrono::high_resolution_clock::now();
 
    double coords[3];
    coords[0] = min_dist;
    coords[1] = 0;
    coords[2] = 0;
 
    if(~cc_->testCollision(coords))
    {
 
        //Perform trajectory generation and collision detection in parallel if enabled
        //Vectors and arrays must be accessed by indicies to ensure thread safe behavior
        #pragma omp parallel for schedule(dynamic) if(parallelism_enabled_) //schedule(dynamic)
        for(size_t i = 0; i < num_paths; i++)
        {
          pips_trajectory_ptr traj = std::make_shared<PipsTrajectory>();
          traj->header = header;
          traj->params = params_;
          
          traj->trajpntr = trajectory_functions[i];
          traj->x0_ = x0;
          
          traj_gen_bridge_->generate_trajectory(traj);

          evaluateTrajectory(traj);

          trajectories[i] = traj;
        }
    }
    else
    {
        trajectories.resize(0);
    }
    

    //End timer
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

    
    ROS_DEBUG_STREAM_NAMED(name_, "Generated " << num_paths << " trajectories in " << fp_ms.count() << " ms");
    
    //traj_gen_bridge_->publishPaths(path_pub_, trajectories, 5); //hard coded for 5 trajectories

    for(size_t i = 0; i < trajectories.size(); i++)
    {
        nav_msgs::PathPtr path;

        path = trajectories[i]->toPathMsg();
        
        path_pub_.publish(path);
        desired_path_pub_.publish(trajectories[i]->getDesiredPathMsg());
    }


    
    return trajectories;
  }
  
  
  void GenAndTest::evaluateTrajectory(pips_trajectory_ptr& traj)
  {
    ni_trajectory_ptr ni_traj= std::static_pointer_cast<ni_trajectory>(traj);
    int collision_ind = GenAndTest::evaluateTrajectory(ni_traj);
    traj->set_collision_ind(collision_ind);
  }
  
  //Test whether trajectory collides
  int GenAndTest::evaluateTrajectory(ni_trajectory_ptr& traj)
  {

      for(size_t i = 0; i < traj->num_states(); i++)
      {

        geometry_msgs::Point pt = traj->getPoint(i);    

        if(pt.x > min_dist)
        {
        
          double coords[3];
          coords[0] = pt.x;
          coords[1] = pt.y;
          coords[2] = pt.z;
          
          if(cc_->testCollision(coords))
          {
              return i;
              
          }
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


  std::vector<traj_func_ptr> GenAndTest::getDefaultTrajectoryFunctions()
  {

    //Set trajectory departure angles and speed
    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4};
    double v = .25;

    size_t num_paths = dep_angles.size();
    
    std::vector<traj_func_ptr> trajectory_functions(num_paths);
    
    for(size_t i = 0; i < num_paths; i++)
    {
      double dep_angle = dep_angles[i];
      trajectory_functions[i] = std::make_shared<angled_straight_traj_func>(dep_angle, v);
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
