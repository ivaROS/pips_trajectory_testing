#include "pips_trajectory_tester.h"

#include <pips/collision_testing/collision_checker.h>
#include <trajectory_generator_ros_interface.h>
#include <pips_trajectory_testing/PipsTrajectoryTesterConfig.h>

#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>

#include <fstream>
#include <memory>
#include <chrono>

#include <omp.h>







bool PipsTrajectory::collides()
{
    return collision_ind_ >=0;
}

ros::Time PipsTrajectory::time_of_collision()
{
    //ROS_WARN_STREAM(  //warn/close if no collision
    return header.stamp + ros::Duration(times[collision_ind_]);
}

void PipsTrajectory::set_collision_ind(int ind)
{
    collision_ind_ = ind;
}

geometry_msgs::PointStamped PipsTrajectory::get_collision_point()
{
    //ROS_WARN_STREAM(  //warn/close if no collision
    return ni_trajectory::getPointStamped(collision_ind_);
}

size_t PipsTrajectory::num_states()
{
    if(PipsTrajectory::collides())
    {
        return collision_ind_;
    }
    else
    {
        return ni_trajectory::num_states();
    }
}

//
void PipsTrajectory::get_collision_ind(int & ind)
{
    ind = collision_ind_;
}

geometry_msgs::PoseStamped PipsTrajectory::get_collision_pose()
{
    geometry_msgs::PoseStamped pose;
    if(collides())
    {
        pose = ni_trajectory::getPoseStamped(collision_ind_);
    }
    else
    {
	ROS_ERROR("Error! Attempting to get colliding pose from trajectory that does not collide!");
    }
    return pose;
}

geometry_msgs::PointStamped PipsTrajectory::get_check_point(const int ind)
{
    //ROS_WARN_STREAM(  //warn/close if no collision
    return ni_trajectory::getPointStamped(ind);
}
