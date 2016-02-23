
#include <chrono>

#include "GenAndTest.h"

#define DEBUG false


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
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
//]





  GenAndTest::GenAndTest(std::vector<cv::Point3d> co_offsets, geometry_msgs::TransformStamped& depth_base_transform)
    :co_offsets_(co_offsets),  depth_base_transform_(depth_base_transform)
  {
      cc_ = NULL;
      traj_gen_bridge_ = *(new TrajectoryGeneratorBridge);
      colliding_path_pub_ = nh_.advertise<nav_msgs::Path>("colliding_paths", 5);
      noncolliding_path_pub_ = nh_.advertise<nav_msgs::Path>("noncolliding_paths", 5);
      pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collision_points", 5);
  }



//Get the transform that takes point in base frame and transforms it to odom frame
//geometry_msgs::TransformStamped base_odom_transform = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0), timeout);

  void GenAndTest::run(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg, geometry_msgs::TransformStamped& base_odom_transform)
  {
    if(DEBUG)ROS_INFO_STREAM("Generating Trajectories");
    
    if(cc_ == NULL)
    {
        cc_ = new CollisionChecker(image_msg, info_msg, depth_base_transform_, co_offsets_, false);
    }
    else
    {
        cc_->init(image_msg);
    }
    
    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4};


    
    double v = .25;
    size_t num_paths = dep_angles.size();
    
    std::vector<ni_trajectory*> trajectories(num_paths);
    //trajectories.reserve(num_paths);

    
    bool collided[8];
    auto t1 = std::chrono::high_resolution_clock::now();
 
    //#pragma omp parallel //schedule(dynamic)
    for(size_t i = 0; i < num_paths; i++)
    {
        double dep_angle = dep_angles[i];
        angled_straight_traj_func trajf(dep_angle, v);
    
        traj_func* trajpntr = &trajf;
      
        ROS_DEBUG_STREAM("Angle: " << dep_angle << std::endl);

        ni_trajectory* traj = traj_gen_bridge_.generate_trajectory(trajpntr);
        traj->frame_id = base_odom_transform.child_frame_id;
        
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

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;


    if(DEBUG)ROS_INFO_STREAM("Generated " << colliding_trajectories.size() << " colliding and " << noncolliding_trajectories.size() << " noncolliding trajectories in " << fp_ms.count() << " ms");
        
    
    
    if(true)
    {
        traj_gen_bridge_.publishPaths(colliding_path_pub_, colliding_trajectories, num_paths);
        traj_gen_bridge_.publishPaths(noncolliding_path_pub_, noncolliding_trajectories, num_paths);
    }
    

    
   
  }
  
  
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
      
      pose_array_pub_.publish(collision_points);
      
      return collided;

  }






