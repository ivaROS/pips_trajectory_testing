
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

#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


#include "GenAndTest.h"
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

protected:
  void setupParams();
  void setupPublishersSubscribers();
  
private:
  bool wander_,ready_;
  
  message_filters::Subscriber<sensor_msgs::Image> depthsub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
  
  ros::Subscriber button_sub_;
  ros::Publisher commanded_trajectory_publisher_;
  
  std::vector<cv::Point3d> co_offsets_;
  GenAndTest* traj_tester_;
  traj_params* params_;
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo> image_sync_policy;
  typedef message_filters::Synchronizer<image_sync_policy> image_synchronizer;
  boost::shared_ptr<image_synchronizer> synced_images;
    
  
  void buttonCB(const kobuki_msgs::ButtonEventPtr msg);
  void depthImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  bool checkCurrentTrajectory();
  std::vector<traj_func*> getTrajectoryFunctions();
};

} //ns kobuki

#endif /* PIPS_TRAJECTORY_CONTROLLER_H_ */

