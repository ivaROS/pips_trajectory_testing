#include "pips_demo.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pips_demo");
  TestTrajectory tester;
  ros::NodeHandle nh("/camera");
  tester.init(nh);
  ros::spin();
}
