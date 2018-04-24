#include "pips_controller.h"
#include <pips_trajectory_testing/depth_image_cc_wrapper.h>


 namespace kobuki
{
  PipsTrajectoryController::PipsTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    ObstacleAvoidanceController(nh, pnh), nh_(nh, name_), pnh_(pnh, name_)
  {
      cc_wrapper_ = std::make_shared<pips_trajectory_testing::DepthImageCCWrapper>(nh_, pnh_,tfBuffer_);
      traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
      
  }




  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool PipsTrajectoryController::isReady(const std_msgs::Header& header)
  {
    if(!ObstacleAvoidanceController::isReady(header))
    {
      return false;
    }

    else
    {
        cc_wrapper_->update();
      return cc_wrapper_->isReady(header);
    }
  }
  
  void PipsTrajectoryController::generateTrajectories()
  {
      std_msgs::Header header = cc_wrapper_->getCurrentHeader();
      ObstacleAvoidanceController::sensorCb(header);
  }
  
  bool PipsTrajectoryController::init()
  {
    ObstacleAvoidanceController::init();
    
    cc_wrapper_->init();
    cc_wrapper_->setCallback(boost::bind(&PipsTrajectoryController::generateTrajectories, this));
    
    return true;
  }
  
}
