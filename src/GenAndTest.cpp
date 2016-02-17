
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
        initial_heading_ = x0[2]; //theta
    }
    
    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[6] = v_*cos(initial_heading_ + dep_angle_);
        dxdt[7] = v_*sin(initial_heading_ + dep_angle_);
    }
    
    
};
//]





  GenAndTest::GenAndTest(std::vector<cv::Point3d> co_offsets, geometry_msgs::TransformStamped& depth_base_transform)
    :co_offsets_(co_offsets),  depth_base_transform_(depth_base_transform)
  {
      traj_gen_bridge_ = *(new TrajectoryGeneratorBridge);
  }



//Get the transform that takes point in base frame and transforms it to odom frame
//geometry_msgs::TransformStamped base_odom_transform = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0), timeout);

  void GenAndTest::run(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg, geometry_msgs::TransformStamped& base_odom_transform)
  {
    std::cout << "Generating trajectories..." << std::endl;
    
    cc_ = new CollisionChecker(image_msg, info_msg, depth_base_transform_, co_offsets_, false);
    
    std::vector<double> dep_angles = {0};
    
    double v = .05;
    
    for(int i = 0; i < dep_angles.size(); i++)
    {
        double dep_angle = dep_angles[i];
        angled_straight_traj_func trajf(v,dep_angle);
    
        traj_func* trajpntr = &trajf;
    

        ni_trajectory traj = traj_gen_bridge_.generate_trajectory(base_odom_transform, trajpntr);
        
        evaluateTrajectory(traj, base_odom_transform);
        
        
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






