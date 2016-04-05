#include "GenAndTest.h"

#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>

#include <collision_checker.h>
#include <trajectory_generator_ros_interface.h>


#define DEBUG true

//Generates a straight line trajectory with a given angle and speed
class angled_straight_traj_func : public traj_func{

    double dep_angle_;
    double v_;

public:
    angled_straight_traj_func( double dep_angle, double v ) : dep_angle_(dep_angle), v_(v) { }
    
    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[XD_IND] = v_*cos( dep_angle_);
        dxdt[YD_IND] = v_*sin( dep_angle_);
    }
    
    
};



  GenAndTest::GenAndTest(std::vector<cv::Point3d> co_offsets, geometry_msgs::TransformStamped& depth_base_transform)
  {
    cc_ = new CollisionChecker(depth_base_transform, co_offsets, false);
    
    traj_gen_bridge_ = *(new TrajectoryGeneratorBridge);
    
    //Create the various visualization publishers
    colliding_path_pub_ = nh_.advertise<nav_msgs::Path>("colliding_paths", 5);
    noncolliding_path_pub_ = nh_.advertise<nav_msgs::Path>("noncolliding_paths", 5);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collision_points", 5);
    
    
    std::string key;

    if(ros::param::search("enable_parallel_loop", key))
    {
      ros::param::get(key, parallelism_enabled_); 
    }
      
  }
  
  GenAndTest::GenAndTest()
  {
    traj_gen_bridge_ = *(new TrajectoryGeneratorBridge);
  }
  
  void GenAndTest::init(std::vector<cv::Point3d> co_offsets, geometry_msgs::TransformStamped& depth_base_transform)
  {
    cc_ = new CollisionChecker(depth_base_transform, co_offsets, false);
    

    
    //Create the various visualization publishers
    colliding_path_pub_ = nh_.advertise<nav_msgs::Path>("colliding_paths", 5);
    noncolliding_path_pub_ = nh_.advertise<nav_msgs::Path>("noncolliding_paths", 5);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collision_points", 5);
    
    
    std::string key;

    if(ros::param::search("enable_parallel_loop", key))
    {
      ros::param::get(key, parallelism_enabled_); 
    }
      
  }



//Get the transform that takes point in base frame and transforms it to odom frame
  std::vector<ni_trajectory> GenAndTest::run(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg, std::string frame_id)

  {
    num_frames=num_frames+1;
    
            ROS_INFO_STREAM("Num frames: " << num_frames);
    
    if(DEBUG)
    {
        ROS_INFO_STREAM("Generating Trajectories");

    }
    
    //Update collision checker with new image and CameraInfo
    cc_->setImage(image_msg, info_msg);
    
    //Set trajectory departure angles and speed
    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4};
    double v = .25;
    
    size_t num_paths = dep_angles.size();
    
    std::vector<ni_trajectory*> trajectories(num_paths);
    bool collided[num_paths];
    
    //Start timer
    auto t1 = std::chrono::high_resolution_clock::now();
 
    //Perform trajectory generation and collision detection in parallel if enabled
    //Vectors and arrays must be accessed by indicies to ensure thread safe behavior
    #pragma omp parallel for schedule(dynamic) if(parallelism_enabled_) //schedule(dynamic)
    for(size_t i = 0; i < num_paths; i++)
    {
      double dep_angle = dep_angles[i];
      angled_straight_traj_func trajf(dep_angle, v);
  
      traj_func* trajpntr = &trajf;
      
      ROS_DEBUG_STREAM("Angle: " << dep_angle << '\n');
  
      ni_trajectory* traj = traj_gen_bridge_.generate_trajectory(trajpntr);
      traj->frame_id = frame_id;
      
      trajectories[i] = traj;
      
      collided[i] = GenAndTest::evaluateTrajectory(traj);
        
    }
    
    ROS_INFO_STREAM("Made it past generation");
    
    std::vector<ni_trajectory> colliding_trajectories;
    std::vector<ni_trajectory> noncolliding_trajectories;
    for(size_t i = 0; i < num_paths; i++)
    {
      if(collided[i])
      {
          colliding_trajectories.push_back(*trajectories[i]);
      }
      else
      {
          noncolliding_trajectories.push_back(*trajectories[i]);
      }
    }

    //End timer
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;


    if(DEBUG)
      ROS_INFO_STREAM("Generated " << colliding_trajectories.size() << " colliding and " << noncolliding_trajectories.size() << " noncolliding trajectories in " << fp_ms.count() << " ms");
        
    
    //Publish colliding and noncolliding paths
    if(true)
    {
      traj_gen_bridge_.publishPaths(colliding_path_pub_, colliding_trajectories, num_paths);
      traj_gen_bridge_.publishPaths(noncolliding_path_pub_, noncolliding_trajectories, num_paths);
    }
    

    return noncolliding_trajectories;
   
  }
  
  //Get the transform that takes point in base frame and transforms it to odom frame
  std::vector<ni_trajectory> GenAndTest::run(std::vector<traj_func*> trajectory_functions, std::string frame_id)
  {
    num_frames=num_frames+1;
    
            ROS_INFO_STREAM("Num frames: " << num_frames);
    
    if(DEBUG)
    {
        ROS_INFO_STREAM("Generating Trajectories");

    }
    
    //Update collision checker with new image and CameraInfo

    
    
    size_t num_paths = trajectory_functions.size();
    
    std::vector<ni_trajectory*> trajectories(num_paths);
    bool collided[num_paths];
    
    //Start timer
    auto t1 = std::chrono::high_resolution_clock::now();
 
    //Perform trajectory generation and collision detection in parallel if enabled
    //Vectors and arrays must be accessed by indicies to ensure thread safe behavior
    #pragma omp parallel for schedule(dynamic) if(parallelism_enabled_) //schedule(dynamic)
    for(size_t i = 0; i < num_paths; i++)
    {

      traj_func* trajpntr = trajectory_functions[i];
  
      ni_trajectory* traj = traj_gen_bridge_.generate_trajectory(trajpntr);
      traj->frame_id = frame_id;
      
      trajectories[i] = traj;
      
      collided[i] = GenAndTest::evaluateTrajectory(traj);
        
    }
    
    
    std::vector<ni_trajectory> colliding_trajectories;
    std::vector<ni_trajectory> noncolliding_trajectories;
    for(size_t i = 0; i < num_paths; i++)
    {
      if(collided[i])
      {
          colliding_trajectories.push_back(*trajectories[i]);
      }
      else
      {
          noncolliding_trajectories.push_back(*trajectories[i]);
      }
    }

    //End timer
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;


    if(DEBUG)
      ROS_INFO_STREAM("Generated " << colliding_trajectories.size() << " colliding and " << noncolliding_trajectories.size() << " noncolliding trajectories in " << fp_ms.count() << " ms");
        
    
    //Publish colliding and noncolliding paths
    if(true)
    {
      traj_gen_bridge_.publishPaths(colliding_path_pub_, colliding_trajectories, num_paths);
      traj_gen_bridge_.publishPaths(noncolliding_path_pub_, noncolliding_trajectories, num_paths);
    }
    

    return noncolliding_trajectories;
   
  }
  
  //Test whether trajectory collides
  bool GenAndTest::evaluateTrajectory(ni_trajectory* traj)
  {
      geometry_msgs::PoseArray collision_points;
      collision_points.header.frame_id = traj->frame_id;

      bool collided = false;
      for(size_t i = 0; !collided && i < traj->num_states(); i++)
      {

        geometry_msgs::Vector3 pt = traj->getPoint(i);    
        double coords[3];
        coords[0] = pt.x;
        coords[1] = pt.y;
        coords[2] = pt.z;
        
        if(cc_->testCollision(coords))
        {
            collided = true;
            geometry_msgs::Pose pose;
            pose.position.x = coords[0];
            pose.position.y = coords[1];
            pose.position.z = coords[2];
            collision_points.poses.push_back(pose);
        }
        else
        {
        
        }
        

      }
      
      //Note: this will actually only ever publish 1 point at a time...
      //Should accumulate points and publish at the end
      pose_array_pub_.publish(collision_points);
      
      return collided;

  }

  bool GenAndTest::evaluateTrajectory(trajectory_generator::trajectory_points& trajectory)
  {
      
      bool collided = false;
      for(size_t i = 0; !collided && i < trajectory.points.size(); i++)
      {

        trajectory_generator::trajectory_point pt = trajectory.points[i];    
        double coords[3];
        coords[0] = pt.x;
        coords[1] = pt.y;
        coords[2] = 0;
        
        if(cc_->testCollision(coords))
        {
            collided = true;
        }
        else
        {
        
        }
        

      }

      return collided;

  }




