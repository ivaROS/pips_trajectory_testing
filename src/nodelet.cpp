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
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    tester->init(nh);
}

PLUGINLIB_EXPORT_CLASS(PipsTrajectoryNodelet, nodelet::Nodelet) 
