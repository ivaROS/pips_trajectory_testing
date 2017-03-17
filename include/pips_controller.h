
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PIPS_CONTROLLER_H_
#define PIPS_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "obstacle_avoidance_controller.h"
#include "pips_trajectory_tester.h"
//#include <pips_trajectory_testing/PipsControllerConfig.h>


#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <dynamic_reconfigure/server.h>

#include <memory>

#include <ros/callback_queue.h>


namespace kobuki
{




/**
 * @ brief 
 *
 * A simple nodelet-based controller intended to avoid obstacles using PIPS.
 */
class PipsTrajectoryController : public kobuki::ObstacleAvoidanceController
{
public:
  PipsTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string& name);
  ~PipsTrajectoryController(){};

  virtual bool init();

protected:
  virtual bool isReady(const std_msgs::Header& header);

  

  CollisionChecker_ptr cc_;
  
private:
  
  bool hasTransform_ = false;
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo> image_sync_policy;
  typedef message_filters::Synchronizer<image_sync_policy> image_synchronizer;
  boost::shared_ptr<image_synchronizer> synced_images;
  
  message_filters::Subscriber<sensor_msgs::Image> depthsub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
  
  void depthImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
               const sensor_msgs::CameraInfo::ConstPtr& info_msg);
               
               


};

} //ns kobuki

#endif /* PIPS_CONTROLLER_H_ */

