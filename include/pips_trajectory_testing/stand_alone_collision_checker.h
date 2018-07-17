#ifndef STAND_ALONE_COLLISION_CHECKER_H
#define STAND_ALONE_COLLISION_CHECKER_H

#include <ros/ros.h>

namespace pips_trajectory_testing
{
  
  template <typename T>
  class StandAloneCollisionChecker
  {

  public:
    StandAloneCollisionChecker(int argc, char** argv)
    {
      ros::init(argc, argv, "stand_alone_collision_checker");
      
      ros::NodeHandle nh;
      ros::NodeHandle pnh("~");
      T s(nh, pnh);
      s.init();
      s.autoUpdate();
      ros::spin();
      
    }
    

  };

}

#endif //STAND_ALONE_COLLISION_CHECKER_H
