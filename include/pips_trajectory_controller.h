
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PIPS_TRAJECTORY_CONTROLLER_H_
#define PIPS_TRAJECTORY_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <trajectory_controller.h>
#include "GenAndTest.h"
#include "rate_tracker.h"

#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>


#include <kobuki_msgs/ButtonEvent.h>


namespace kobuki
{

/**
 * @ brief 
 *
 * A simple nodelet-based controller intended to avoid obstacles using PIPS.
 */
class PipsTrajectoryController : public kobuki::TrajectoryController
{
public:
  PipsTrajectoryController(ros::NodeHandle& nh, std::string& name);
  ~PipsTrajectoryController(){};

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool init();
  
  typedef std::shared_ptr<GenAndTest> GenAndTest_ptr;

protected:
  void setupParams();
  void setupPublishersSubscribers();
  void OdomCB(const nav_msgs::Odometry::ConstPtr& msg);

  
private:
  bool wander_,ready_;
  
  message_filters::Subscriber<sensor_msgs::Image> depthsub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
  
  ros::Subscriber button_sub_;
  ros::Publisher commanded_trajectory_publisher_;
  
  std::vector<cv::Point3d> co_offsets_;
  GenAndTest_ptr traj_tester_;
  ros::Duration min_ttc_;
  ros::Duration min_tte_;
  
  rate_tracker odom_rate;
  rate_tracker image_rate;
  
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo> image_sync_policy;
  typedef message_filters::Synchronizer<image_sync_policy> image_synchronizer;
  boost::shared_ptr<image_synchronizer> synced_images;
    
  
  void buttonCB(const kobuki_msgs::ButtonEvent::ConstPtr& msg);
  void depthImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
               const sensor_msgs::CameraInfo::ConstPtr& info_msg);
  bool checkCurrentTrajectory(const std_msgs::Header& header);
  std::vector<traj_func_ptr> getTrajectoryFunctions();
};

} //ns kobuki

#endif /* PIPS_TRAJECTORY_CONTROLLER_H_ */

