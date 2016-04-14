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
#include "pips_trajectory_controller.h"
#include "GenAndTest.h"
#include <trajectory_controller.h>

#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <kobuki_msgs/ButtonEvent.h>
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


  PipsTrajectoryController::PipsTrajectoryController(ros::NodeHandle& nh, std::string& name) : kobuki::TrajectoryController(nh, name) 
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
    
    double radius = .178;
    double height = .48;
    double floor_tolerance = .05;
    double safety_expansion = .05;

    cv::Point3d topr(radius+safety_expansion,-height,radius+safety_expansion);
    cv::Point3d topl(-radius-safety_expansion,-height,radius+safety_expansion);
    cv::Point3d bottomr(radius+safety_expansion,-floor_tolerance,radius+safety_expansion);
    cv::Point3d bottoml(-radius-safety_expansion,-floor_tolerance,radius+safety_expansion);


    cv::Point3d offsets[] = {topr,topl,bottoml,bottomr};
    std::vector<cv::Point3d> co_offsets(offsets, offsets + sizeof(offsets) / sizeof(cv::Point3d) );
    co_offsets_ = co_offsets;
    
    traj_tester_->init(nh_);
    
    wander_ = false;
    ready_ = false;
    
    
    //these next 2 lines are just for initial testing!
    wander_ = true;
    this->disable();
    
    return true;
  }


  void PipsTrajectoryController::setupParams()
  {
    TrajectoryController::setupParams();
    

    //params_ = new traj_params(traj_tester_->traj_gen_bridge_.copyDefaultParams());
    
    //nh_.param<double>("tf", params_->tf, 5);

  }
  
  
  void PipsTrajectoryController::setupPublishersSubscribers()
  {
    TrajectoryController::setupPublishersSubscribers();
    std::string depth_image_topic = "/camera/depth/image_raw";
    std::string depth_info_topic = "/camera/depth/camera_info";

    ROS_DEBUG_STREAM("[" << name_ << "] Setting up publishers and subscribers");

    depthsub_.subscribe(nh_, depth_image_topic, 10);
    depth_info_sub_.subscribe(nh_, depth_info_topic, 10);
    synced_images.reset(new image_synchronizer(image_synchronizer(10), depthsub_, depth_info_sub_) );
    synced_images->registerCallback(bind(&PipsTrajectoryController::depthImageCb, this, _1, _2));
    
    button_sub_ = nh_.subscribe("/mobile_base/events/button", 10, &PipsTrajectoryController::buttonCB, this);
    
    //Currently not using this; would it be effective with multithreaded spinner?
    commanded_trajectory_publisher_ = nh_.advertise< trajectory_generator::trajectory_points >("/desired_trajectory", 1);
  }
  

  void PipsTrajectoryController::buttonCB(const kobuki_msgs::ButtonEventPtr msg)
  {
    if (msg->button == kobuki_msgs::ButtonEvent::Button0 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
    {
      wander_ = true;
      ROS_INFO_STREAM("[" << name_ <<"] Activating Wander");
    }
    else
    {
      ROS_INFO_STREAM("[" << name_ <<"] Button event");
    }
  };
  
  void PipsTrajectoryController::depthImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {

    ros::Duration timeout(0);
    


    if(!ready_) {
        ROS_DEBUG_STREAM("[" << name_ << "] Not ready, check for transform");
        try
        {
          //Get the transform that takes a point in the base frame and transforms it to the depth optical
          geometry_msgs::TransformStamped depth_base_transform = tfBuffer_->lookupTransform(info_msg->header.frame_id, base_frame_id_, ros::Time(0));
          
          traj_tester_->setRobotInfo(co_offsets_, depth_base_transform);
          
          ready_ = true;

          ROS_DEBUG_STREAM("[" << name_ << "] Created trajectory testing instance");

        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          return;
        }
    }
    
    if(ready_)
    {
      ROS_DEBUG_STREAM("[" << name_ << "] Ready (check for transform)");
      
      if(curr_odom_ == NULL)
      {
        ROS_ERROR_STREAM("[" << name_ << "] No odometry received!");
        return;
      }
      else
      {
        ros::Duration delta_t = curr_odom_->header.stamp - info_msg->header.stamp;
        ROS_DEBUG_STREAM("[" << name_ << "] Odometry is " << delta_t << " newer than current image");
      }
      
      if(wander_)
      {
      
        ROS_DEBUG_STREAM("[" << name_ << "] Updating collision checker image");
        //Update tester with new image/camera info
        traj_tester_->setImage(image_msg, info_msg);
        
        
        //check if current path is still clear
        if(executing_)
        {
          ROS_DEBUG_STREAM("[" << name_ << "] Executing: Checking if current path clear");
          if(PipsTrajectoryController::checkCurrentTrajectory())
          {
            ROS_WARN_STREAM("[" << name_ << "] Current trajectory collides!");
            executing_ = false;
          }
        }
        
        //Generate trajectories and assign best
        if(!executing_)
        {    
          ROS_DEBUG_STREAM("[" << name_ << "] Not currently executing, test new trajectories");
          std::vector<traj_func*> trajectory_functions = PipsTrajectoryController::getTrajectoryFunctions();
          std::vector<ni_trajectory*> valid_trajs = traj_tester_->run(trajectory_functions, curr_odom_);
          
          ROS_DEBUG_STREAM("[" << name_ << "] Found " << valid_trajs.size() << " non colliding  trajectories");
          if(valid_trajs.size() >0)
          {
            ni_trajectory* chosen_traj = TrajectoryGeneratorBridge::getLongestTrajectory(valid_trajs);
            //executeTrajectory

            trajectory_generator::trajectory_points msg = chosen_traj->toTrajectoryMsg();
            PipsTrajectoryController::TrajectoryCB(msg);

          }
        }
    

      }
            

    
   
    }
  }

  bool PipsTrajectoryController::checkCurrentTrajectory()
  {
    trajectory_generator::trajectory_points trimmed_trajectory;
    trimmed_trajectory.header = desired_trajectory_.header;
    trimmed_trajectory.points = std::vector<trajectory_generator::trajectory_point>(desired_trajectory_.points.begin() + curr_index_, desired_trajectory_.points.end());
      
    trajectory_generator::trajectory_points localTrajectory = tfBuffer_->transform(trimmed_trajectory, base_frame_id_);
    
    return traj_tester_->evaluateTrajectory(localTrajectory);
  }
  
  std::vector<traj_func*> PipsTrajectoryController::getTrajectoryFunctions()
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




} // namespace kobuki
// %EndTag(FULLTEXT)%

