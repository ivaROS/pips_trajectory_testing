#ifndef PIPS_TRAJECTORY_TESTER_H
#define PIPS_TRAJECTORY_TESTER_H

#include <pips/collision_testing/collision_checker.h>
#include <trajectory_generator_ros_interface.h>
#include <pips_trajectory_testing/PipsTrajectoryTesterConfig.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <memory>
#include <chrono>

#include <omp.h>
#include <type_traits>



template<typename T>
class PipsTrajectory : public T
{
  int collision_ind_ = -1;

public:

  bool collides()
  {
    return collision_ind_ >=0;
  }
  
  ros::Time time_of_collision()
  {
    //ROS_WARN_STREAM(  //warn/close if no collision
    return T::header.stamp + ros::Duration(T::times[collision_ind_]);
  }
  
  void set_collision_ind(int ind)
  {
    collision_ind_ = ind;
  }
  
  geometry_msgs::PointStamped get_collision_point()
  {
    //ROS_WARN_STREAM(  //warn/close if no collision
    return T::getPointStamped(collision_ind_);
  }
  
  size_t num_states()
  {
    if(PipsTrajectory::collides())
    {
      return collision_ind_;
    }
    else
    {
      return T::num_states();
    }
  }
  
  //
  void get_collision_ind(int & ind)
  {
    ind = collision_ind_;
  }
  
  geometry_msgs::PoseStamped get_collision_pose()
  {
    geometry_msgs::PoseStamped pose;
    if(collides())
    {
      pose = T::getPoseStamped(collision_ind_);
    }
    else
    {
      ROS_ERROR("Error! Attempting to get colliding pose from trajectory that does not collide!");
    }
    return pose;
  }
  
  //What is this used for?
  geometry_msgs::PointStamped get_check_point(const int ind)
  {
    //ROS_WARN_STREAM(  //warn/close if no collision
    return T::getPointStamped(ind);
  }
  
  
};



typedef std::shared_ptr<CollisionChecker> CollisionChecker_ptr;



template<typename state_type, typename traj_func_t>
class GenAndTest
{
public:
  typedef trajectory_states<state_type, traj_func_t> TrajectoryStates;
  typedef std::shared_ptr<TrajectoryStates> trajectory_ptr;
  
  typedef PipsTrajectory<TrajectoryStates> pips_trajectory;
  typedef std::shared_ptr<pips_trajectory> pips_trajectory_ptr;
  
  //typedef pips_trajectory_ptr trajectory_ptr;
  typedef traj_func_t traj_func_type;
  
  typedef std::shared_ptr<traj_func_type> traj_func_ptr;
  
  typedef TrajectoryGeneratorBridge<state_type, traj_func_type> TrajBridge;
  typedef std::shared_ptr<TrajBridge> TrajectoryGeneratorBridge_ptr;
  
  typedef typename state_type::trajectory_msg_t trajectory_points;

  
private:
  
  TrajectoryGeneratorBridge_ptr traj_gen_bridge_;
  
  
  std::string name_ = "GenAndTest";
  ros::NodeHandle nh_, pnh_;
  geometry_msgs::TransformStamped depth_base_transform_;
  

  typedef dynamic_reconfigure::Server<pips_trajectory_testing::PipsTrajectoryTesterConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  CollisionChecker_ptr cc_, cc2_;
  
  CCOptions cc_options_;

  ros::Publisher path_pub_, desired_path_pub_, visualization_pub_;
  int num_frames =0;
  std::string base_frame_id_ = "";
  std_msgs::Header header_;

  bool parallelism_enabled_ = true;

  traj_params_ptr params_;
  
  bool initialized_ = false;

  // TODO: move this into the collision checker
  double min_dist_ = .05;// Check this distance first, then don't have to evaluate trajectories closer than that

public:
  
  void setParam_TF(double tf_) {
    params_->tf = tf_;
  }

  
  GenAndTest(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
  nh_(nh),
  pnh_(pnh, name_)
  {
    traj_gen_bridge_ = std::make_shared<TrajBridge>();
    params_ = std::make_shared<traj_params>(traj_gen_bridge_->getDefaultParams());  //NOTE: With dynamic reconfigure, this is probably next to pointless
  }
  
  GenAndTest(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) :
  name_(name),
  nh_(nh),
  pnh_(pnh, name_)
  {
    traj_gen_bridge_ = std::make_shared<TrajBridge>();
    params_ = std::make_shared<traj_params>(traj_gen_bridge_->getDefaultParams()); //NOTE: With dynamic reconfigure, this is probably next to pointless
  }


  void setBaseFrameId(const std::string base_frame_id)
  {
    base_frame_id_ = base_frame_id;
  }
  
  
//   std::vector<trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions)
//   {
//     state_type x0;
//     return GenAndTest::run(trajectory_functions, x0);
//   }
//

  
//   
//   //This is for cases where no header is available
//   std::vector<trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions, const state_type& x0)
//   {
//     std_msgs::Header header;
//     return GenAndTest::run(trajectory_functions, x0, header);
//   }
//   
//   std::vector<trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions, double v0, std_msgs::Header& header)
//   {
//     state_type x0 = traj_gen_bridge_->initState();
//     x0[near_identity::V_IND] = v0;
//     return GenAndTest::run(trajectory_functions, x0, header);
//   }
//   
  
  
  //This is the lowest level version that is actually run; the rest are for convenience
  std::vector<pips_trajectory_ptr> run(const std::vector<traj_func_ptr>& trajectory_functions, const state_type& x0, const std_msgs::Header& header, const traj_params_ptr params)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Generating Trajectories");
    size_t num_paths = trajectory_functions.size();
    
    std::vector<pips_trajectory_ptr> trajectories(num_paths); //std::vector<boost::shared_ptr<PipsTrajectory*>>
    
    //Start timer
    auto t1 = std::chrono::high_resolution_clock::now();
    
    //NOTE: preCheck currently does nothing
    if(!preCheck())
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Parallelism: " << parallelism_enabled_);
      //Perform trajectory generation and collision detection in parallel if enabled
      //Vectors and arrays must be accessed by indicies to ensure thread safe behavior
      
      
      
      //To aid in debugging, disable parallel loop completely in debug mode
      #ifdef NDEBUG 
      if (omp_get_dynamic())
        omp_set_dynamic(0);
      #pragma omp parallel if(parallelism_enabled_) //schedule(dynamic)
      #endif
      {
        #pragma omp single nowait
        for(size_t i = 0; i < num_paths; i++)
        {
          // NOTE
          // append shared(x0, header, trajectory_functions), according to:
          // https://stackoverflow.com/questions/43900638/openmp-task-cant-pass-argument-by-reference 
          //
          // #pragma omp task
          #pragma omp task shared(x0, header, trajectory_functions)
          {
            trajectories[i] = generateTraj(x0, header, params, trajectory_functions[i]);
          }
        }
      }
    }
    else
    {
      trajectories.resize(0);
    }
    
    
    //End timer
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    
    ROS_INFO_STREAM_NAMED(name_, "Generated " << num_paths << " trajectories in " << fp_ms.count() << " ms");
    
    return trajectories;
  }
  
  
  pips_trajectory_ptr generateTraj(const state_type& x0, const std_msgs::Header& header, traj_params_ptr params, traj_func_ptr traj_func)
  {
    pips_trajectory_ptr traj = std::make_shared<pips_trajectory>();
    traj->header = header;
    traj->params = params;
    
    traj->trajpntr = traj_func;
    traj->x0_ = x0;
    
    size_t steps = traj_gen_bridge_->generate_trajectory(traj);
    
    if(steps == 0)
    {
      ROS_ERROR_STREAM_NAMED(name_, "The trajectory has 0 states! Check initial conditions and parameters");
    }
    
    evaluateTrajectory(traj);
    return traj;
  }
  
  
  //This version is for standard online running
  std::vector<pips_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const state_type& x0, const std_msgs::Header& header)
  {
    std::vector<pips_trajectory_ptr> trajectories = run(trajectory_functions, x0, header, params_);
    
    //It is debateable whether path publishing belongs in this class...
    TrajBridge::publishPaths(path_pub_, trajectories);
    TrajBridge::publishDesiredPaths(desired_path_pub_, trajectories);
    
    
    visualization_msgs::Marker colliding_points;
    colliding_points.header = header;
    colliding_points.pose.orientation.w=1;
    colliding_points.ns = "collision_points";
    colliding_points.type = visualization_msgs::Marker::POINTS;
    colliding_points.action = visualization_msgs::Marker::ADD;
    colliding_points.scale.x = .05;
    colliding_points.scale.y = .05;
    colliding_points.color.a=1;
    colliding_points.color.r=1;
    
    for(auto traj : trajectories)
    {
      //pips_trajectory_ptr traj= std::static_pointer_cast<PipsTrajectory>(ni_traj);
      
      if(traj->collides())
      {
        colliding_points.points.push_back(traj->get_collision_pose().pose.position);
      }
    }
    visualization_pub_.publish(colliding_points);
    
    return trajectories;
  }
  
  
  template <typename M> //For any ros message with a header
  std::vector<pips_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const M& msg, const std::string& base_frame_id, const ros::Time& stamp)
  {
    state_type x0(msg);
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = base_frame_id;
    return GenAndTest::run(trajectory_functions, x0, header);
  }
  
  template <typename M>
  std::vector<pips_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const M& msg, const ros::Time& stamp)
  {
    return GenAndTest::run(trajectory_functions, msg, base_frame_id_, stamp);
  }
  
  template <typename M>
  std::vector<pips_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const M& msg, const std_msgs::Header& header)
  {
    const M& m = *msg;
    return GenAndTest::run(trajectory_functions, m, base_frame_id_, header.stamp);
  }
  
  //msg is not passed as a reference in this instance: we want a copy to be made of the boost::shared_ptr, so that this instance will be constant even if the calling function assigns a new message to curr_odom
  template <typename M> //For any ros message without a header
  std::vector<pips_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const typename M::ConstPtr msg, const std_msgs::Header& header)
  {
    const M& m = *msg;
    return GenAndTest::run(trajectory_functions, m, header);
  }
  
  // Overload for odometry message since it contains the base frame id
  std::vector<pips_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const nav_msgs::Odometry& msg, const std_msgs::Header& header)
  {
    return GenAndTest::run(trajectory_functions, msg, msg.child_frame_id, header.stamp);
  }
  

  
  template<typename T> struct is_shared_ptr : std::false_type {};
  template<typename T> struct is_shared_ptr<boost::shared_ptr<T>> : std::true_type {};
  
  template <typename M>
  std::vector<pips_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const M& msg, typename std::enable_if<!is_shared_ptr<M>::value>::type* dummy =0 )
  {
    return GenAndTest::run(trajectory_functions, msg, msg.header);
  }
  
  //For any ros message with a header
  template <typename M>
  std::vector<pips_trajectory_ptr> run(std::vector<traj_func_ptr>& trajectory_functions, const M& msg, typename std::enable_if<is_shared_ptr<M>::value>::type* dummy =0 )
  {
    return GenAndTest::run(trajectory_functions, *msg);
  }



  
  
  
  void setCollisionChecker(CollisionChecker_ptr cc)
  {
    cc_ = cc;
  }
  
  void setCollisionChecker(CollisionChecker_ptr cc, CollisionChecker_ptr cc2)
  {
    cc_ = cc;
    cc2_ = cc2;
  }
  
  void init()
  {
    if(!initialized_)
    {
      //Create the various visualization publishers
      path_pub_ = pnh_.advertise<pips_msgs::PathArray>("tested_paths", 5);
      //desired_path_pub_ = pnh_.advertise<pips_msgs::PathArray>("desired_paths", 5);
      visualization_pub_ = pnh_.advertise<visualization_msgs::Marker>("collision_points", 5);
      
      
      reconfigure_server_.reset( new ReconfigureServer(pnh_));
      reconfigure_server_->setCallback(boost::bind(&GenAndTest::configCB, this, _1, _2));
      
      initialized_ = true;
    }
  }
  
  
  void configCB(pips_trajectory_testing::PipsTrajectoryTesterConfig &config, uint32_t level) {
    //TODO: Need to add the rest of these
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request: tf=" << config.tf << ", parallelism=" << (config.parallelism?"True":"False") << ", detailed_collisions=" << (config.collision_details?"Enabled":"Disabled"));
    
    parallelism_enabled_ = config.parallelism;
    //TODO: add option set max number of threads, 0 corresponding to 'auto'
    
    cc_options_ = CCOptions(config.collision_details);
    
    
    params_->tf = config.tf;
    params_->dt = config.dt;
    
    //Turtlebot parameters, need to move to turtlebot trajectory generator
    //     params_->cp = config.cp;
    //     params_->cd = config.cd;
    //     params_->cl = config.cl;
    //     params_->eps = config.eps;
        
    //     params_->a_max = config.a_max;
    //     params_->v_max = config.v_max;
    //     params_->w_max = config.w_max;
    //     params_->w_dot_max = config.w_dot_max;
    
  }
  
  
  
  
  bool preCheck()
  {
    bool returnValue = false;
    
    if(cc_)
    {
      //returnValue = cc_->testCollision(min_dist_);
    }
    return returnValue;
  }
  
  void evaluateTrajectory(pips_trajectory_ptr& traj)
  {
    trajectory_ptr ni_traj= traj;
    int collision_ind = GenAndTest::evaluateTrajectory(ni_traj);
    traj->set_collision_ind(collision_ind);
  }
  
  //Test whether trajectory collides
  //Idea: move this to header, template it, and make the type of pose retrieved depend on the type desired 
  //That way, approaches without orientation info will end up calling the Point version directly (perhaps stamped?)
  int evaluateTrajectory(const trajectory_ptr& traj)
  {
    if(cc_)
    {
      if(cc2_)
      {
        return evaluateTrajectory2(traj);
      }
      
      size_t num_states = traj->num_states();
      
      //return evaluate(traj, num_states, 0);
      for(size_t i = 0; i < num_states; i++)
      {
        geometry_msgs::Pose pose = traj->getPose(i);
        
        if(cc_->testCollision(pose, cc_options_))
        {
          return i;
        }
      }
    }
    else
    {
      ROS_WARN_ONCE_NAMED(name_, "No collision checker has been created!");
    }
    
    return -1;
    
  }
  
  int evaluateTrajectory2(const trajectory_ptr& traj)
  {
    const size_t num_states = traj->num_states();
    
    size_t res1 = num_states;
    size_t res2 = num_states;
    
    std::vector<geometry_msgs::Pose> poses;
    for(size_t i = 0; i < num_states; i++)
    {
      poses.push_back(traj->getPose(i));
    }
    
    //bool early_cancel = omp_get_cancellation();
    
    
    #pragma omp taskgroup
    {
      size_t max_ind = num_states;
      #pragma omp task shared(res1, max_ind)
      {
        size_t i = 0;
        //for(int i = 0; i < num_states; i++)
        while(true)
        {
          size_t x;
          
          #pragma omp atomic read
          x = max_ind;
          
          if(i < x)
          {
            geometry_msgs::Pose pose = poses[i];
            
            if(cc_->testCollision(pose, cc_options_))
            {
              #pragma omp atomic write
              max_ind = i;
              
              {
                res1 = i;
                ROS_DEBUG_STREAM_NAMED(name_, "cc1 detected collision @ " << i);
              }
              break;
              //#pragma omp cancel taskgroup
            }
            else
            {
              ++i;
            }
          }
          else
          {
            break;
          }
          //#pragma omp cancellation point taskgroup
        }
      }
      
      #pragma omp task shared(res2, max_ind)
      {
        size_t i = 0;
        //for(int i = 0; i < num_states; i++)
        while(true)
        {
          size_t x;
          
          #pragma omp atomic read
          x = max_ind;
          
          if(i < x)
          {
            geometry_msgs::Pose pose = poses[i];
            
            if(cc2_->testCollision(pose, cc_options_))
            {
              #pragma omp atomic write
              max_ind = i;
              
              {
                res2 = i;
                ROS_DEBUG_STREAM_NAMED(name_, "cc1 detected collision @ " << i);
              }
              break;
              //#pragma omp cancel taskgroup
            }
            else
            {
              ++i;
            }
          }
          else
          {
            break;
          }
          //#pragma omp cancellation point taskgroup
        }
      }
    }
   
   
   size_t min_res = std::min(res1,res2);
   
   int res;
   
   if(min_res < num_states)
   {
     res= min_res;
   }
   else
   {
     res= -1;
   }
    
    ROS_DEBUG_STREAM_NAMED(name_, "results: res1=" << res1 << ", res2=" << res2 << ", min_res=" << min_res << ", res=" << res);
    
    return res;
  }
  
//   int evaluate(const trajectory_ptr& traj, size_t num_states, int ind)
//   {
//     if(ind == num_states)
//     {
//       return -1;
//     }
//     
//     int ret;
//     
//     
//     #pragma omp task shared(traj)
//     {
//       geometry_msgs::Pose pose = traj->getPose(ind);
//       
//       if(cc_->testCollision(pose, cc_options_))
//       {
//         ret = ind;
//       }
//       else
//       {
//         ret = evaluate(traj, num_states, ind+1);
//       }
//     }
//     
//     return ret;
//   }
  
  int evaluateTrajectory(const trajectory_points& trajectory)
  {
    if(cc_)
    {
      for(size_t i = 0; i < trajectory.points.size(); i++)
      {
        //TODO: move conversion somewhere else so we can just pass trajectory_point to collision checker
        auto pt = trajectory.points[i]; 
        
        geometry_msgs::Pose pose;
        pose.position.x = pt.x;
        pose.position.y = pt.y;
        TrajBridge::yawToQuaternion(pt.theta);
        if(cc_->testCollision(pose, cc_options_))
        {
          return i;
        }
        
      }
    }
    else
    {
      ROS_WARN_ONCE_NAMED(name_, "No collision checker has been created!");
    }
    
    
    return -1;
    
  }
  
 // std::vector<cv::Mat> generateDepthImages(const std::vector<traj_func_ptr>& trajectory_functions, const state_type& x0, const std_msgs::Header& header);
 // cv::Mat generateTrajectoryDepthImage(const pips_trajectory_ptr& traj);


  
//  void saveCollisionCheckData(std::vector<traj_func_ptr>& trajectory_functions);
  
//  std::fstream save_check_data_;
    
};


#endif //PIPS_TRAJECTORY_TESTER_H
