/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /kobuki_controller_tutorial/include/kobuki_controller_tutorial/bump_blink_controller.hpp
 *
 * @brief "Bump-Blink"-controller for the Kobuki controller tutorial
 *
 * A simple nodelet-based controller for Kobuki, which makes one of Kobuki's LEDs blink, when a bumper is pressed.
 *
 * @author Marcus Liebhardt, Yujin Robot
 **/


/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "pips_controller.h"
#include "pips_trajectory_tester.h"
#include <trajectory_controller.h>
#include <pips_trajectory_testing/PipsControllerConfig.h> //Dynamic Reconfigure for high level pips controller functions

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

  PipsTrajectoryController::PipsTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string& name) : 
      kobuki::TrajectoryController(nh, pnh, name), 
      wander_(false), 
      ready_(false)
  {
    traj_tester_ = std::make_shared<GenAndTest>();
    
  };
  
  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool PipsTrajectoryController::init()
  {
    kobuki::TrajectoryController::init();
    traj_tester_->init(nh_);

    //Using pointer
    reconfigure_server_.reset( new ReconfigureServer(pnh_));
    reconfigure_server_->setCallback(boost::bind(&PipsTrajectoryController::configCB, this, _1, _2));
    
    //If not using pointer:
    //param_server_.setCallback(boost::bind(&PipsTrajectoryController::configCB, this, _1, _2));
  
    PipsTrajectoryController::setupPublishersSubscribers();
    
    
    //these next 2 lines are just for initial testing! Although perhaps wander should be on by default in any case...
    //wander_ = true;
    //this->disable();
    
    return true;
  }

  void PipsTrajectoryController::configCB(pips_trajectory_testing::PipsControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request:\n\tMin_ttc =\t" << config.min_ttc << "\n\tMin_tte =\t"<< config.min_tte <<"\n\tWander =\t" << (config.wander?"True":"False") << "\n\tv_des =\t" << config.v_des); //<< config.num_paths 
    min_ttc_ = ros::Duration(config.min_ttc);
    min_tte_ = ros::Duration(config.min_tte);
    

    wander_ = config.wander;
    num_paths_ = config.num_paths;
    v_des_ = config.v_des;
    
  }
  
  
  void PipsTrajectoryController::setupPublishersSubscribers()
  {
    std::string depth_image_topic = "/camera/depth/image_raw";
    std::string depth_info_topic = "/camera/depth/camera_info";

    ROS_DEBUG_STREAM_NAMED(name_,  "Setting up publishers and subscribers");

    depthsub_.subscribe(nh_, depth_image_topic, 10);
    depth_info_sub_.subscribe(nh_, depth_info_topic, 10);
    synced_images.reset(new image_synchronizer(image_synchronizer(10), depthsub_, depth_info_sub_) );
    synced_images->registerCallback(bind(&PipsTrajectoryController::depthImageCb, this, _1, _2));
    
    button_sub_ = nh_.subscribe("/mobile_base/events/button", 10, &PipsTrajectoryController::buttonCB, this);
    bumper_sub_ = nh_.subscribe("/mobile_base/events/bumper", 10, &PipsTrajectoryController::bumperCB, this);
    
    commanded_trajectory_publisher_ = nh_.advertise< trajectory_generator::trajectory_points >("/desired_trajectory", 1, true);
  }
  
  //Pressing button 0 activates wander mode
  void PipsTrajectoryController::buttonCB(const kobuki_msgs::ButtonEvent::ConstPtr& msg)
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
  void PipsTrajectoryController::bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    if (wander_ && msg->state == kobuki_msgs::BumperEvent::PRESSED )
    {
      wander_ = false;
      ROS_INFO_STREAM_NAMED(name_,  "Robot collided with obstacle! Deactivating Wander");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_,  "Non-handled Bumper event");
    }
  };
  
  //Note: Is it really necessary to get both image and camera info? More importantly, does it slow things much to do a synchronized callback like this?
  //If it does, then should have separate callbacks- this one would just get the image, and the other would check if camerainfo changes, and if so update it. That does sound messy though. On the other hand, if the only thing that changes is the size, then the image msg has that anyway so it would be easy.
  void PipsTrajectoryController::depthImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
               const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    if(info_msg->header.stamp == ros::Time(0))  // Gazebo occasionally publishes Image and CameraInfo messages with time=0
      return;
      
    ros::Duration timeout(0);
    
    image_rate.addTime(info_msg->header);
    
    ROS_WARN_STREAM_THROTTLE_NAMED(2, name_,"Image rate: " << image_rate.getRate() << " (" << image_rate.getNumSamples() << " samples). Current delay: " << image_rate.getLastDelay() << "s; Average delay: " << image_rate.getAverageDelay() << "s.");


    if(!ready_) {
        ROS_DEBUG_STREAM_ONCE_NAMED(name_, "Not ready, check for transform...");
        try
        {
          //Get the transform that takes a point in the base frame and transforms it to the depth optical
          geometry_msgs::TransformStamped depth_base_transform = tfBuffer_->lookupTransform(info_msg->header.frame_id, base_frame_id_, ros::Time(0));
          traj_tester_->setRobotInfo(depth_base_transform);
         
          ready_ = true;

          ROS_DEBUG_STREAM_NAMED(name_,  "Transform found! Initializing trajectory testing with robot info");

        }
        catch (tf2::TransformException &ex) {
          ROS_WARN_STREAM_THROTTLE_NAMED(5, name_, "Problem finding transform:\n" <<ex.what());
          return;
        }
    }
    
    ROS_DEBUG_STREAM_NAMED(name_, "Updating collision checker image");
    //Update tester with new image/camera info
    traj_tester_->setImage(image_msg, info_msg);
    
    if(ready_)
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Ready");
      
      if(!curr_odom_)
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(5 , name_,  "No odometry received!");
        return;
      }
      else
      {
        ros::Duration delta_t = curr_odom_->header.stamp - info_msg->header.stamp;
        ROS_DEBUG_STREAM_NAMED(name_, "Odometry is " << delta_t << " newer than current image");
      }
      
    

      if(wander_)
      {

        bool replan = false;
        //check if current path is still clear
        if(executing_)
        {
          ROS_DEBUG_STREAM_NAMED(name_, "Executing: Checking if current path clear");
          if(PipsTrajectoryController::checkCurrentTrajectory(info_msg->header))
          {
            replan = true;
          }

        }
        
        //Generate trajectories and assign best
        if(!executing_ || replan)
        {    
          ROS_DEBUG_STREAM_COND_NAMED(!executing_, name_, "Not currently executing, test new trajectories");
          ROS_DEBUG_STREAM_COND_NAMED(replan, name_, "Time to replan");
          std::vector<traj_func_ptr> trajectory_functions = PipsTrajectoryController::getTrajectoryFunctions();
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
              trajectory_generator::trajectory_points msg = chosen_traj->toTrajectoryMsg();
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
            stop();
          }
        }
    

      }

    else
    {
      std::vector<traj_func_ptr> trajectory_functions = PipsTrajectoryController::getTrajectoryFunctions();
      std::vector<ni_trajectory_ptr> valid_trajs = traj_tester_->run(trajectory_functions, curr_odom_);
    }
                

    
   
    }
  }


  bool PipsTrajectoryController::checkCurrentTrajectory(const std_msgs::Header& header)
  {
    trajectory_generator::trajectory_points trimmed_trajectory;

    
    //Lock trajectory mutex while copying
    {
      boost::mutex::scoped_lock lock(trajectory_mutex_);
      
      trimmed_trajectory.header = desired_trajectory_.header;
      trimmed_trajectory.points = std::vector<trajectory_generator::trajectory_point>(desired_trajectory_.points.begin() + curr_index_, desired_trajectory_.points.end());
    }
    
    trajectory_generator::trajectory_points localTrajectory;
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
  
  
  std::vector<traj_func_ptr> PipsTrajectoryController::getTrajectoryFunctions()
  {

    //Set trajectory departure angles and speed
    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4};
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

