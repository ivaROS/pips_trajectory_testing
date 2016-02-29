#include "TestTrajectory.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectory");
  TestTrajectory tester;
  ros::NodeHandle nh = getNodeHandle();
  tester.init(nh);
  ros::spin();
}
