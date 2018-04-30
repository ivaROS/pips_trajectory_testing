
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

#include <pips_trajectory_testing/pips_cc_wrapper.h>


//#include <dynamic_reconfigure/server.h>

#include <memory>


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
  PipsTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);
  ~PipsTrajectoryController(){};

  virtual bool init();
  
  static constexpr const char* DEFAULT_NAME="PipsController";


protected:
  bool isReady(const std_msgs::Header& header);
  
  void sensorCb(const std_msgs::Header& header);
  
  void generateTrajectories();
  
  virtual void setupTrajectoryTesters();
 


  
private:
  std::string name_;
  ros::NodeHandle pnh_;
  
  std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;

};

} //ns kobuki

#endif /* PIPS_CONTROLLER_H_ */

