#ifndef PIPS_CC_WRAPPER_H
#define PIPS_CC_WRAPPER_H

#include <pips/collision_testing/pips_collision_checker.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/message_filter.h>

namespace pips_trajectory_testing
{
  
  class PipsCCWrapper
  {
  private:
    bool hasTransform_=false;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string base_frame_id_ = "base_footprint";
    
  public:
    
    PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name);
    PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, std::shared_ptr<tf2_ros::Buffer>& tf_buffer);
    
    
    virtual bool init()=0;
    
    virtual void update()=0;
    
    virtual std::shared_ptr<PipsCollisionChecker> getCC()=0;
    
    bool isReady ( const std_msgs::Header& header );
    
  protected:
    ros::NodeHandle nh_, pnh_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    

  private:
    

  };

}

#endif //PIPS_CC_WRAPPER_H