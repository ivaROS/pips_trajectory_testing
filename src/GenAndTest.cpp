
#include <chrono>

#include "GenAndTest.h"


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class angled_straight_traj_func : public traj_func{

    double dep_angle_;
    double v_;
    double initial_heading_;

public:
    angled_straight_traj_func( double dep_angle, double v ) : dep_angle_(dep_angle), v_(v) { }

    void init ( const state_type &x0 )
    {
        initial_heading_ = x0[THETA_IND]; //theta
    }
    
    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[XD_IND] = v_*cos(initial_heading_ + dep_angle_);
        dxdt[YD_IND] = v_*sin(initial_heading_ + dep_angle_);
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
  }



//Get the transform that takes point in base frame and transforms it to odom frame
//geometry_msgs::TransformStamped base_odom_transform = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0), timeout);

  void GenAndTest::run(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg, geometry_msgs::TransformStamped& base_odom_transform)
  {
    ROS_INFO_STREAM("Generating Trajectories");
    
    if(cc_ == NULL)
    {
        cc_ = new CollisionChecker(image_msg, info_msg, depth_base_transform_, co_offsets_, false);
        cc_->setBaseTransform(base_odom_transform);        
    }
    else
    {
        cc_->init(image_msg, base_odom_transform);
    }
    
    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4};
    std::vector<ni_trajectory> colliding_trajectories;
    std::vector<ni_trajectory> noncolliding_trajectories;
    
    double v = .5;
    
    for(int i = 0; i < dep_angles.size(); i++)
    {
        double dep_angle = dep_angles[i];
        angled_straight_traj_func trajf(dep_angle, v);
    
        traj_func* trajpntr = &trajf;
    

        ni_trajectory traj = traj_gen_bridge_.generate_trajectory(base_odom_transform, trajpntr);
        
        if(GenAndTest::evaluateTrajectory(traj, base_odom_transform))
        {
            colliding_trajectories.push_back(traj);
        }
        else
        {
            noncolliding_trajectories.push_back(traj);
        }
        
        ROS_DEBUG_STREAM("Generated " << colliding_trajectories.size() << " colliding and " << noncolliding_trajectories.size() << " noncolliding trajectories");
        
        
    }
    
    
    
    if(true)
    {
        traj_gen_bridge_.publishPaths(colliding_path_pub_, colliding_trajectories);
        traj_gen_bridge_.publishPaths(noncolliding_path_pub_, noncolliding_trajectories);
    }
    

    
   
  }
  
  
  bool GenAndTest::evaluateTrajectory(ni_trajectory& traj, geometry_msgs::TransformStamped base_odom_transform )
  {
      bool collided = false;
      for(int i = 0; !collided && i < traj.num_states(); i++)
      {
        
        geometry_msgs::Vector3 pt = traj.getPoint(i);    
        double coords[3];
        coords[0] = pt.x;
        coords[1] = pt.y;
        coords[2] = pt.z;
        
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






