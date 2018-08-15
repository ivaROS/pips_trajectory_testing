#ifndef TF_UTILS_ODOM_TO_TF_H
#define TF_UTILS_ODOM_TO_TF_H

#include <nav_msgs/Odometry.h>
#include <tf2/buffer_core.h>
//#include <tf2_ros/buffer.h>

namespace tf_utils
{
  inline
  geometry_msgs::TransformStamped toTransform(const nav_msgs::Odometry::ConstPtr& odom)
  {
    geometry_msgs::TransformStamped transform;
    transform.header = odom->header;
    transform.child_frame_id = odom->child_frame_id;
    
    geometry_msgs::Vector3& trans = transform.transform.translation;
    const geometry_msgs::Point& pos = odom->pose.pose.position;
    
    trans.x = pos.x;
    trans.y = pos.y;
    trans.z = pos.z;
    
    transform.transform.rotation = odom->pose.pose.orientation;
    
    return transform;
  }
  
  inline
  bool AddToBuffer(const nav_msgs::Odometry::ConstPtr odom, tf2::BufferCore* buffer, const std::string& authority)
  {
    geometry_msgs::TransformStamped transform = toTransform(odom);
    
    try
    {
      return buffer->setTransform(transform, authority, false);
    }
    catch (tf2::TransformException& ex)
    {
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", transform.child_frame_id.c_str(), transform.header.frame_id.c_str(), temp.c_str());
      return false;
    }
  }
  
  inline
  bool AddToBuffer(const nav_msgs::Odometry::ConstPtr odom, std::shared_ptr<tf2::BufferCore> buffer, const std::string& authority)
  {
    geometry_msgs::TransformStamped transform = toTransform(odom);
    
    try
    {
      return buffer->setTransform(transform, authority, false);
    }
    catch (tf2::TransformException& ex)
    {
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", transform.child_frame_id.c_str(), transform.header.frame_id.c_str(), temp.c_str());
      return false;
    }
  }
  
/*  
  bool AddToBuffer(const nav_msgs::Odometry::ConstPtr odom, std::shared_ptr<tf2::BufferCore>& buffer, const std::string& authority)
  {
    return AddToBuffer(odom, *buffer, authority);
  }*/

}

#endif //TF_UTILS_ODOM_TO_TF_H
