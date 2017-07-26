
/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "obstacle_avoidance_controller.h"
#include "pips_trajectory_tester.h"
#include <trajectory_controller.h>
#include <pips_trajectory_testing/PipsControllerConfig.h> //Dynamic Reconfigure for high level pips controller functions
#include <tf2_pips/tf2_trajectory.h>

#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/BumperEvent.h>
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

namespace kobuki
{

  ObstacleAvoidanceController::ObstacleAvoidanceController(ros::NodeHandle& nh, ros::NodeHandle& pnh) : 
      kobuki::TrajectoryController(nh, pnh), 
      pnh_(pnh),
      wander_(false), 
      idle_eval_(false),
      ready_(false)
  {
    traj_tester_ = std::make_shared<GenAndTest>(nh_, pnh_);
    
  };
  
  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool ObstacleAvoidanceController::init()
  {
    kobuki::TrajectoryController::init();
    traj_tester_->init();

    //Using pointer
    reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
    reconfigure_server_->setCallback(boost::bind(&ObstacleAvoidanceController::configCB, this, _1, _2));
    

    ObstacleAvoidanceController::setupPublishersSubscribers();
    

    return true;
  }

  void ObstacleAvoidanceController::configCB(pips_trajectory_testing::PipsControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request:\n\tMin_ttc =\t" << config.min_ttc << "\n\tMin_tte =\t"<< config.min_tte <<"\n\tWander =\t" << (config.wander?"True":"False") << "\n\tv_des =\t" << config.v_des); //<< config.num_paths 
    min_ttc_ = ros::Duration(config.min_ttc);
    min_tte_ = ros::Duration(config.min_tte);
    

    wander_ = config.wander;
    idle_eval_ = config.idle_eval;
    num_paths_ = config.num_paths;
    v_des_ = config.v_des;
    
    
  }
  
  bool ObstacleAvoidanceController::isReady(const std_msgs::Header& header)
  {
    if(!curr_odom_)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(5 , name_,  "No odometry received!");
      return false;
    }
    else
    {
      ros::Duration delta_t = curr_odom_->header.stamp - header.stamp;
      ROS_DEBUG_STREAM_NAMED(name_, "Odometry is " << delta_t << " newer than current sensor data");
    }
    return true;
  }
  
  void ObstacleAvoidanceController::setupPublishersSubscribers()
  {
    ROS_DEBUG_STREAM_NAMED(name_,  "Setting up publishers and subscribers");
    
    button_sub_ = nh_.subscribe("/mobile_base/events/button", 10, &ObstacleAvoidanceController::buttonCB, this);
    bumper_sub_ = nh_.subscribe("/mobile_base/events/bumper", 10, &ObstacleAvoidanceController::bumperCB, this);
    
    commanded_trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/desired_trajectory", 1, true);
  }
  
  //Pressing button 0 activates wander mode
  void ObstacleAvoidanceController::buttonCB(const kobuki_msgs::ButtonEvent::ConstPtr& msg)
  {
    if (msg->button == kobuki_msgs::ButtonEvent::Button0 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
    {
      wander_ = true;
      ROS_INFO_STREAM_NAMED(name_,  "Activating Wander");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_,  "Non-handled Button event");
    }
  };
  
  //Hitting the bumper deactivates wander mode
  void ObstacleAvoidanceController::bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED )
    {
      stop();
      wander_ = false;
      ROS_INFO_STREAM_NAMED(name_,  "Robot collided with obstacle! Deactivating Wander");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_,  "Non-handled Bumper event");
    }
  };
  
  
  void ObstacleAvoidanceController::sensorCb(const std_msgs::Header& header)
  {
    
    if(header.stamp == ros::Time(0))  // Gazebo occasionally publishes Image and CameraInfo messages with time=0
    {
      ROS_WARN_STREAM_NAMED( name_,"Bad timestamp");
      return;
    }
    

    
    image_rate.addTime(header);
    
    ROS_WARN_STREAM_THROTTLE_NAMED(2, name_,"Image rate: " << image_rate.getRate() << " (" << image_rate.getNumSamples() << " samples). Current delay: " << image_rate.getLastDelay() << "s; Average delay: " << image_rate.getAverageDelay() << "s.");

    if(isReady(header));
    {
      
      if(wander_)
      {

        bool replan = false;
        //check if current path is still clear
        if(executing_)
        {
          ROS_DEBUG_STREAM_NAMED(name_, "Executing: Checking if current path clear");
          if(ObstacleAvoidanceController::checkCurrentTrajectory(header))
          {
            replan = true;
          }

        }
        
        //Generate trajectories and assign best
        if(!executing_ || replan)
        {    
          ROS_DEBUG_STREAM_COND_NAMED(!executing_, name_, "Not currently executing, test new trajectories");
          ROS_DEBUG_STREAM_COND_NAMED(replan, name_, "Time to replan");
          std::vector<traj_func_ptr> trajectory_functions = ObstacleAvoidanceController::getTrajectoryFunctions();
          std::vector<ni_trajectory_ptr> valid_trajs = traj_tester_->run(trajectory_functions, curr_odom_);
          
          ROS_DEBUG_STREAM_NAMED(name_, "Found " << valid_trajs.size() << " non colliding  trajectories");
          
          bool foundPath = false;
          if(valid_trajs.size() >0)
          {
            ni_trajectory_ptr chosen_traj = TrajectoryGeneratorBridge::getCenterLongestTrajectory(valid_trajs);

            ROS_INFO_STREAM_NAMED(name_, "Length of longest trajectory: " << chosen_traj->getDuration());

            if(chosen_traj->getDuration() > min_ttc_)
            {
              foundPath = true;
              pips_trajectory_msgs::trajectory_points msg = chosen_traj->toTrajectoryMsg();
              commanded_trajectory_publisher_.publish(msg);
            }
            else
            {
              ROS_WARN_STREAM_NAMED(name_, "The longest found trajectory is shorter than the required minimum time to collision (min_ttc) (" << min_ttc_ << ")" );
            }

           }
          
          //If no satisfactory trajectory was found, then command a halt.
          if(!foundPath)
          {
              ROS_WARN_STREAM_NAMED(name_, "No path found, halting." );
            stop();
          }
        }
    

      }
      //If we're not wandering, then calculate trajectories constantly
      else if(idle_eval_)
      {
        std::vector<traj_func_ptr> trajectory_functions = ObstacleAvoidanceController::getTrajectoryFunctions();
        std::vector<ni_trajectory_ptr> valid_trajs = traj_tester_->run(trajectory_functions, curr_odom_);
      }
                  

    
   
    }
  }


  bool ObstacleAvoidanceController::checkCurrentTrajectory(const std_msgs::Header& header)
  {
    pips_trajectory_msgs::trajectory_points trimmed_trajectory;

    
    //Lock trajectory mutex while copying
    {
      boost::mutex::scoped_lock lock(trajectory_mutex_);
      
      trimmed_trajectory.header = desired_trajectory_.header;
      trimmed_trajectory.points = std::vector<pips_trajectory_msgs::trajectory_point>(desired_trajectory_.points.begin() + curr_index_, desired_trajectory_.points.end());
    }
    
    pips_trajectory_msgs::trajectory_points localTrajectory;
    try
    {
      localTrajectory = tfBuffer_->transform(trimmed_trajectory, base_frame_id_, ros::Duration(.1)); // This was where the transform exceptions were actually coming from!
    
      ROS_DEBUG_STREAM_NAMED(name_, "Successfully transformed current trajectory from frame [" << trimmed_trajectory.header.frame_id << "] to frame [" << localTrajectory.header.frame_id << "] at time " << localTrajectory.header.stamp);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_NAMED(name_, "Unable to transform trajectory: %s",ex.what());
        return true;  //If we can't verify that the current trajectory is safe, better act as though it isn't
    }
    

    
    int collision_ind = traj_tester_->evaluateTrajectory(localTrajectory);
    
    if((collision_ind >=0) && (localTrajectory.points[collision_ind].time - localTrajectory.points.front().time) < min_ttc_)
    {
      ROS_WARN_STREAM_NAMED(name_, "Current trajectory collides!");
      return true;
    }
    else if((localTrajectory.points.back().time - localTrajectory.points.front().time) < min_tte_)
    {
      ROS_WARN_NAMED(name_, "No imminent collision, but close to end of trajectory"); //should be debug, but for now making more obvious
      return true;
    }
    else
    {
      return false;
    }
  }
  
  
  std::vector<traj_func_ptr> ObstacleAvoidanceController::getTrajectoryFunctions()
  {

    //Set trajectory departure angles and speed
    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4}; //,.6,.8,1,1.2,1.6,2,2.4};
    double v = v_des_;  //.25;

    size_t num_paths = dep_angles.size(); // = num_paths_; 
    
    std::vector<traj_func_ptr> trajectory_functions(num_paths);
    
    for(size_t i = 0; i < num_paths; i++)
    {
      trajectory_functions[i] = std::make_shared<angled_straight_traj_func>(dep_angles[i], v);

    }
    return trajectory_functions;
  }




} // namespace kobuki
// %EndTag(FULLTEXT)%

