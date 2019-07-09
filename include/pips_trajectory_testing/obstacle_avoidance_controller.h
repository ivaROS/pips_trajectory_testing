
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef OBSTACLE_AVOIDANCE_CONTROLLER_H_
#define OBSTACLE_AVOIDANCE_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
//#include <trajectory_controller.h>
#include <pips_trajectory_testing/pips_trajectory_tester.h>
#include <pips_trajectory_testing/rate_tracker.h>
#include <pips_trajectory_testing/PipsControllerConfig.h>
#include <tf_utils/odom_to_tf.h>


//#include <sensor_msgs/Image.h>
//#include <image_transport/subscriber_filter.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
//#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

#include <memory>


namespace pips_trajectory_testing
{



/**
 * @ brief 
 *
 * High level obstacle avoidance controller. Uses platform-specific controller and trajectory tester
 */
template<typename Controller, typename GenAndTest>
class ObstacleAvoidanceController : public Controller
{
  
public:
  
  typedef typename GenAndTest::trajectory_ptr trajectory_ptr;
  typedef typename GenAndTest::traj_func_type traj_func_type;
  typedef typename GenAndTest::traj_func_ptr traj_func_ptr;
  typedef typename GenAndTest::trajectory_points trajectory_points;
  typedef typename GenAndTest::TrajBridge TrajBridge;

  typedef std::shared_ptr<GenAndTest> GenAndTest_ptr;

protected:
  virtual void setupPublishersSubscribers()=0;
  std::string name_;
  
  
private:
  static constexpr const char* DEFAULT_NAME="ObstacleAvoidanceController";
  
protected:
  ros::NodeHandle pnh_;

  
  bool wander_,idle_eval_,ready_;
  

  ros::Publisher commanded_trajectory_publisher_;
  

  GenAndTest_ptr traj_tester_, traj_tester2_;
  CollisionChecker_ptr cc_;
  ros::Duration min_ttc_;
  ros::Duration min_tte_;
  
  rate_tracker image_rate;
  
  typedef dynamic_reconfigure::Server<pips_trajectory_testing::PipsControllerConfig> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;

  int num_paths_;
  double v_des_, path_limits_;
  
protected:
  void configCB(pips_trajectory_testing::PipsControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request:\n\tMin_ttc =\t" << config.min_ttc << "\n\tMin_tte =\t"<< config.min_tte <<"\n\tWander =\t" << (config.wander?"True":"False") << "\n\tv_des =\t" << config.v_des); //<< config.num_paths 
    min_ttc_ = ros::Duration(config.min_ttc);
    min_tte_ = ros::Duration(config.min_tte);
    
    
    wander_ = config.wander;
    idle_eval_ = config.idle_eval;
    num_paths_ = config.num_paths;
    v_des_ = config.v_des;
    path_limits_ = config.path_limits;
    
  }
  


  
  
  void sensorCb(const std_msgs::Header& header)
  {
    
    if(header.stamp == ros::Time(0))  // Gazebo occasionally publishes Image and CameraInfo messages with time=0
    {
      ROS_WARN_STREAM_NAMED( name_,"Bad timestamp");
      return;
    }
    
    ROS_DEBUG_STREAM_NAMED(name_, "SensorCB time: " << header.stamp );
    
    image_rate.addTime(header);
    
    ROS_DEBUG_STREAM_THROTTLE_NAMED(2, name_,"Sensor Callback rate: " << image_rate.getRate() << " (" << image_rate.getNumSamples() << " samples). Current delay: " << image_rate.getLastDelay() << "s; Average delay: " << image_rate.getAverageDelay() << "s.");
    
    nav_msgs::Odometry::ConstPtr odom = Controller::curr_odom_;
    
    tf_utils::AddToBuffer(odom, Controller::tfBuffer_, "odometry_msg");
    
    std_msgs::Header base_header = odom->header;
    base_header.frame_id = odom->child_frame_id;
    
    if(Controller::isReady(header) && isReady(base_header))
    {
      
      if(wander_)
      {
        
        bool replan = false;
        //check if current path is still clear
        if(Controller::executing_)
        {
          ROS_DEBUG_STREAM_NAMED(name_, "Executing: Checking if current path clear");
          if(ObstacleAvoidanceController::checkCurrentTrajectory(header))
          {
            replan = true;
          }
          
        }
        
        //Generate trajectories and assign best
        if(!Controller::executing_ || replan)
        {    
          ROS_DEBUG_STREAM_COND_NAMED(!Controller::executing_, name_, "Not currently executing, test new trajectories");
          ROS_DEBUG_STREAM_COND_NAMED(replan, name_, "Time to replan");
          
//           std::vector<traj_func_ptr> trajectory_functions = getTrajectoryFunctions();
//           auto valid_trajs = traj_tester_->run(trajectory_functions, odom);
//           
//           ROS_DEBUG_STREAM_NAMED(name_, "Found " << valid_trajs.size() << " non colliding  trajectories");
          
          auto chosen_traj = findPath(odom);
          
          bool foundPath = false;
          if(chosen_traj && chosen_traj->getDuration() > min_ttc_)
          {
            foundPath = true;
            trajectory_points msg = chosen_traj->toMsg();
            commanded_trajectory_publisher_.publish(msg);
          }
          else
          {
            ROS_WARN_STREAM_NAMED(name_, "The longest found trajectory is shorter than the required minimum time to collision (min_ttc) (" << min_ttc_ << ")" );
          }
          
          //If no satisfactory trajectory was found, then command a halt.
          if(!foundPath)
          {
            ROS_WARN_STREAM_NAMED(name_, "No path found, halting." );
            Controller::stop();
          }
        }
        
        
      }
      //If we're not wandering, then calculate trajectories constantly
      else if(idle_eval_)
      {
        std::vector<traj_func_ptr> trajectory_functions = getTrajectoryFunctions();
        auto valid_trajs = traj_tester_->run(trajectory_functions, odom);  //TODO: unless all controllers use odometry message, this should change
      }
      
      
      
      
    }
  }
  
  virtual typename TrajBridge::trajectory_ptr findPath(nav_msgs::Odometry::ConstPtr odom )
  {
    std::vector<traj_func_ptr> trajectory_functions = getTrajectoryFunctions();
    auto valid_trajs = traj_tester_->run(trajectory_functions, odom);
    
    ROS_DEBUG_STREAM_NAMED(name_, "Found " << valid_trajs.size() << " non colliding  trajectories");
    
    if(valid_trajs.size() >0)
    {
      auto chosen_traj = TrajBridge::getCenterLongestTrajectory(valid_trajs);
      
      ROS_INFO_STREAM_NAMED(name_, "Length of longest trajectory: " << chosen_traj->getDuration());

      return chosen_traj;
    }
    
    return nullptr;
  }
  
  
  virtual bool isReady(const std_msgs::Header& header)
  {
    return Controller::isReady(header);
  }
  

  //Internal methods can pass by reference safely
  bool getCurrentTrajectoryStats(const std_msgs::Header& header, ros::Duration& ttc, ros::Duration& tte)
  {
    trajectory_points trimmed_trajectory;
    
    
    //Lock trajectory mutex while copying
    {
      boost::mutex::scoped_lock lock(Controller::trajectory_mutex_);
      
      trimmed_trajectory.header = Controller::desired_trajectory_.header;
      trimmed_trajectory.header.stamp = header.stamp; //This isn't ideal, but without it the difference in time gets too big and the trajectory can't be transformed to base frame. Only works because desired_trajectory_ is in 'odom' frame, which is continuous
      trimmed_trajectory.points.insert(trimmed_trajectory.points.begin(), Controller::desired_trajectory_.points.begin() + Controller::curr_index_, Controller::desired_trajectory_.points.end());
    }
    
    trajectory_points localTrajectory;
    try
    {
      localTrajectory = Controller::tfBuffer_->transform(trimmed_trajectory, Controller::base_frame_id_, ros::Duration(.1)); // This was where the transform exceptions were actually coming from!
      
      ROS_DEBUG_STREAM_NAMED(name_, "Successfully transformed current trajectory from frame [" << trimmed_trajectory.header.frame_id << "] to frame [" << localTrajectory.header.frame_id << "] at time " << localTrajectory.header.stamp);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN_NAMED(name_, "Unable to transform trajectory: %s",ex.what());
      return true;  //If we can't verify that the current trajectory is safe, better act as though it isn't
    }
    
    
    ros::WallTime start = ros::WallTime::now();
    int collision_ind = traj_tester2_->evaluateTrajectory(localTrajectory);
    ROS_DEBUG_STREAM_NAMED(name_ + "timing", "Current trajectory evaluated in " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
    
    if(collision_ind >=0)
    {
      ttc = (localTrajectory.points[collision_ind].time - localTrajectory.points.front().time); 
    }
    tte = localTrajectory.points.back().time - localTrajectory.points.front().time;
    
    return collision_ind >=0;
  }
  
  //Internal methods can pass by reference safely
  bool checkCurrentTrajectory(const std_msgs::Header& header)
  {
    ros::Duration ttc, tte;
    bool collides = getCurrentTrajectoryStats(header, ttc, tte);
    
    bool retval;
    if(ttc >=ros::Duration(0) && ttc < min_ttc_)
    {
      ROS_WARN_STREAM_NAMED(name_, "Current trajectory collides!");
      retval = true;
    }
    else if(tte < min_tte_)
    {
      ROS_WARN_NAMED(name_, "No imminent collision, but close to end of trajectory"); //should be debug, but for now making more obvious
      retval = true;
    }
    else
    {
      retval = false;
    }
    
/*    
    if(collides)
    {
      localTrajectory.points.resize(collision_ind);
    }*/
    //TODO: publish the noncolliding current trajectory message (probably as visualization msgs, with different colors for colliding/noncolliding
    
    return retval;
  }
  
  virtual void setupTrajectoryTesters()=0;
  
  virtual std::vector<traj_func_ptr> getTrajectoryFunctions()=0;  
  
  
  
public:
  
  ObstacleAvoidanceController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME) : 
    Controller(nh, pnh), 
    pnh_(pnh, name),
    wander_(false), 
    idle_eval_(false),
    ready_(false)
  {
    
  };
  ~ObstacleAvoidanceController(){};
  
  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  virtual bool init()
  {
    Controller::init();
    
    setupTrajectoryTesters();
    
    reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
    reconfigure_server_->setCallback(boost::bind(&ObstacleAvoidanceController::configCB, this, _1, _2));
    
    setupPublishersSubscribers();
    
    
    return true;
  }

};

} //ns pips_trajectory_testing

#endif /* PIPS_TRAJECTORY_CONTROLLER_H_ */

