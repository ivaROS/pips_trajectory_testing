#include "nodelet.h"
#include <pluginlib/class_list_macros.h>


PipsTrajectoryNodelet::PipsTrajectoryNodelet()
{
    tester = new TestTrajectory;
}

PipsTrajectoryNodelet::~PipsTrajectoryNodelet()
{
}

void PipsTrajectoryNodelet::onInit()
{
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle pnh = getMTPrivateNodeHandle();
    tester->init(nh);
}

PLUGINLIB_EXPORT_CLASS(PipsTrajectoryNodelet, nodelet::Nodelet) 
