#ifndef PIPS_CC_WRAPPER_H
#define PIPS_CC_WRAPPER_H

#include <pips/collision_testing/transforming_collision_checker.h>
#include <tf/transform_datatypes.h>
#include <tf2_utils/transform_manager.h>

#include <tf2_ros/message_filter.h>
#include <pips_trajectory_testing/TestCollisionStamped.h>

namespace pips_trajectory_testing
{

  class PipsCCWrapper
  {
  public:
    typedef boost::function<void() > Callback;
      
  private:
    bool hasTransform_=false;
    ros::ServiceServer collision_testing_service_;
    
    Callback cb_=0;
    

    
  public:
    
    //PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name);
    PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tf_buffer = tf2_utils::TransformManager(false));
    
    
    virtual bool init();
    
    virtual void update()=0;
    
    virtual void setBaseFrame(const std::string& base_frame_id);

    
    virtual std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()=0;
    
    virtual bool isReady();
    virtual bool isReady(const std_msgs::Header& header);
    virtual bool testCollisionSrv(pips_trajectory_testing::TestCollisionStamped::Request &req, pips_trajectory_testing::TestCollisionStamped::Response &res);
    
    
    virtual bool isReadyImpl() { return true;}
    
    bool transformReady ( const std_msgs::Header& header);
    bool transformReady ( const std_msgs::Header& target_header, const std_msgs::Header& source_header);
    
    void setCallback(Callback cb=0);
    void autoUpdate();
    
    virtual std_msgs::Header getCurrentHeader()=0;

    std::string getName() const {return name_;}
    
    virtual bool publishersExist() {return true;}
    
  protected:
    ros::NodeHandle nh_, pnh_;
    std::string name_;
    tf2_utils::TransformManager tfm_;
    
    void doCallback();
    
    void defaultCallback();
    
    bool inited_ = false;
    
    boost::mutex callback_mutex_;
    using Lock = boost::mutex::scoped_lock;
    

    std::string base_frame_id_;
    std::string fixed_frame_id_;

  };

}

#endif //PIPS_CC_WRAPPER_H
