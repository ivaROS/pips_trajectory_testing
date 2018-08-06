#ifndef PIPS_CC_WRAPPER_H
#define PIPS_CC_WRAPPER_H

#include <pips/collision_testing/transforming_collision_checker.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/message_filter.h>

namespace pips_trajectory_testing
{
  typedef boost::function<void() > Callback;

  class PipsCCWrapper
  {
  private:
    bool hasTransform_=false;
    
    
    std::string base_frame_id_ = "base_footprint";
    std::string fixed_frame_id_ = "odom";
    
    Callback cb_=0;
    

    
  public:
    
    //PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name);
    PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::make_shared<tf2_ros::Buffer>());
    
    
    virtual bool init();
    
    virtual void update()=0;
    
    virtual void setBaseFrame(const std::string& base_frame_id);

    
    virtual std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()=0;
    
    virtual bool isReady();
    virtual bool isReady(const std_msgs::Header& header);
    
    
    virtual bool isReadyImpl() { return true;}
    
    bool transformReady ( const std_msgs::Header& header);
    bool transformReady ( const std_msgs::Header& target_header, const std_msgs::Header& source_header);
    
    void setCallback(Callback cb=0);
    void autoUpdate();
    
    virtual std_msgs::Header getCurrentHeader()=0;

    
  protected:
    ros::NodeHandle nh_, pnh_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    void doCallback();
    
    void defaultCallback();
    
    bool inited_ = false;


  private:
    

  };

}

#endif //PIPS_CC_WRAPPER_H
