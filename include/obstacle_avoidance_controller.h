
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef OBSTACLE_AVOIDANCE_CONTROLLER_H_
#define OBSTACLE_AVOIDANCE_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <trajectory_controller.h>
#include "pips_trajectory_tester.h"
#include <pips_trajectory_testing/PipsControllerConfig.h>


#include <sensor_msgs/Image.h>
//#include <image_transport/subscriber_filter.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
//#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

#include <memory>

#include <ros/callback_queue.h>

#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/BumperEvent.h>


namespace kobuki
{



/**
 * @ brief 
 *
 * A simple nodelet-based controller intended to avoid obstacles using PIPS.
 */
class ObstacleAvoidanceController : public kobuki::TrajectoryController
{
public:
  ObstacleAvoidanceController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ObstacleAvoidanceController(){};

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  virtual bool init();
  
  typedef std::shared_ptr<GenAndTest> GenAndTest_ptr;

protected:
  void setupPublishersSubscribers();

private:
  ros::NodeHandle pnh_;
  std::string name_ = "ObstacleAvoidanceController";
  
protected:

  
  bool wander_,idle_eval_,ready_;
  

  ros::Subscriber button_sub_, bumper_sub_;
  ros::Publisher commanded_trajectory_publisher_;
  

  GenAndTest_ptr traj_tester_;
  CollisionChecker_ptr cc_;
  ros::Duration min_ttc_;
  ros::Duration min_tte_;
  
  rate_tracker image_rate;
  
  typedef dynamic_reconfigure::Server<pips_trajectory_testing::PipsControllerConfig> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;

  int num_paths_;
  double v_des_;
  
  

    
  void buttonCB(const kobuki_msgs::ButtonEvent::ConstPtr& msg);
  void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);

  void sensorCb(const std_msgs::Header& header);
  void configCB(pips_trajectory_testing::PipsControllerConfig &config, uint32_t level);
  
  virtual bool isReady(const std_msgs::Header& header);

  //Internal methods can pass by reference safely
  bool checkCurrentTrajectory(const std_msgs::Header& header);
  virtual std::vector<traj_func_ptr> getTrajectoryFunctions();
  virtual std::vector<double> getDepartureAngles();

};

} //ns kobuki

#endif /* PIPS_TRAJECTORY_CONTROLLER_H_ */

