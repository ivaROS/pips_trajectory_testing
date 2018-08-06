#ifndef DEPTH_IMAGE_CC_WRAPPER_H
#define DEPTH_IMAGE_CC_WRAPPER_H

#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <pips/collision_testing/depth_image_collision_checker.h>
#include <sensor_msgs/Image.h>
//#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <boost/thread/mutex.hpp>


namespace pips_trajectory_testing
{

class DepthImageCCWrapper : public pips_trajectory_testing::PipsCCWrapper
{

  
private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> image_sync_policy;
  typedef message_filters::Synchronizer<image_sync_policy> time_synchronizer;
    
  typedef tf2_ros::MessageFilter<sensor_msgs::CameraInfo> tf_filter;
  
  
  boost::shared_ptr<tf_filter> info_tf_filter_;
  boost::shared_ptr<time_synchronizer> image_synchronizer_;
    
  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
  
  sensor_msgs::Image::ConstPtr current_image;
  sensor_msgs::CameraInfo::ConstPtr current_camInfo;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  static constexpr const int MAGIC_NUMBER = -17;  //The sole purpose of this is to ensure that the constructor with the 'optional' tfbuffer argument is not accidentally passed a tfbuffer object
  
  
  std::shared_ptr<pips::collision_testing::DepthImageCollisionChecker> cc_;
  
  typedef boost::mutex::scoped_lock Lock;
  boost::mutex mutex_;
  
  void depthImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
                    const sensor_msgs::CameraInfo::ConstPtr& info_msg);
  
  
  
public:
  // Use this constructor if you don't have a tfbuffer
  DepthImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, int tamper_prevention = MAGIC_NUMBER, std::shared_ptr<tf2_ros::Buffer> tf_buffer=std::make_shared<tf2_ros::Buffer>());
  
  // Use this constructor if you already have a tfbuffer
  DepthImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string& name=DEFAULT_NAME);
  
  
  bool init();
  
  void update();
  
  bool isReadyImpl();

  
  static constexpr const char* DEFAULT_NAME="depth_image_cc_wrapper";
  

  
  std_msgs::Header getCurrentHeader();
  
  std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
  {
    return cc_;
  }
  

};

}

#endif //DEPTH_IMAGE_CC_WRAPPER_H
