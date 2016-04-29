#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pluginlib/class_list_macros.h>

namespace pips
{

struct RosbagPublisher {
public:
  RosbagPublisher() {

  }
  
  void CameraCB(sensor_msgs::Image image_msg, sensor_msgs::CameraInfo info_msg)
  {
    sensor_msgs::Image::Ptr img_ptr = sensor_msgs::Image::Ptr(new sensor_msgs::Image(image_msg));
    sensor_msgs::CameraInfo::Ptr info_ptr = sensor_msgs::CameraInfo::Ptr(new sensor_msgs::CameraInfo(info_msg));    
    
    img_ptr->header.stamp = ros::Time::now();
    info_ptr->header.stamp = img_ptr->header.stamp;
    
    image_pub_.publish(img_ptr);
    info_pub_.publish(info_ptr);
    
  }
  
  void OdomCB(nav_msgs::Odometry msg)
  {
    odom_pub_.publish(msg);
  }

  bool init(ros::NodeHandle& nh)
  {
    nh_ = nh;
    boost::shared_ptr<sensor_msgs::Image const> sharedPtr;
    sensor_msgs::Image img;
    

    
    std::string in_image_topic = "/rosbag/image";
    std::string in_info_topic = "/rosbag/info";
    std::string in_odom_topic = "/rosbag/odom";
    
    std::string out_image_topic = "/camera/depth/image_raw";
    std::string out_info_topic = "/camera/depth/camera_info";
    std::string out_odom_topic = "/odom";
    

    
    depthsub_.subscribe(nh_, in_image_topic, 10);
    depth_info_sub_.subscribe(nh_, in_info_topic, 10);
    synced_images.reset(new image_synchronizer(image_synchronizer(10), depthsub_, depth_info_sub_) );
    synced_images->registerCallback(bind(&RosbagPublisher::CameraCB, this, _1, _2));
    
    odom_sub_ = nh_.subscribe(in_odom_topic, 10, &RosbagPublisher::OdomCB, this);
    
    
    image_pub_ = nh_.advertise< sensor_msgs::Image >(out_image_topic, 10);
    info_pub_ = nh_.advertise< sensor_msgs::CameraInfo >(out_info_topic, 10);
    odom_pub_ = nh_.advertise< nav_msgs::Odometry >(out_odom_topic, 10);
    
    ROS_INFO("Started subscribers/publishers");
    return true;
  }
  
private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  
  ros::Publisher odom_pub_, image_pub_, info_pub_;
  
  message_filters::Subscriber<sensor_msgs::Image> depthsub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo> image_sync_policy;
  typedef message_filters::Synchronizer<image_sync_policy> image_synchronizer;
  boost::shared_ptr<image_synchronizer> synced_images;

};


class RosbagPublisherNodelet : public nodelet::Nodelet
{
public:
  RosbagPublisherNodelet(){};
  ~RosbagPublisherNodelet(){}

  /**
   * @brief Initialise the nodelet
   *
   * This function is called, when the nodelet manager loads the nodelet.
   */
  virtual void onInit()
  {
    ros::NodeHandle nh = this->getPrivateNodeHandle();

    // resolve node(let) name
    std::string name = nh.getUnresolvedNamespace();
    int pos = name.find_last_of('/');
    name = name.substr(pos + 1);

    NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
    publisher_.reset(new RosbagPublisher());

    // Initialises the controller
    if (publisher_->init(nh))
    {
      NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
    }
    else
    {
      NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
    }
  }
private:
  boost::shared_ptr<RosbagPublisher> publisher_;
};
}



PLUGINLIB_EXPORT_CLASS(pips::RosbagPublisherNodelet, 
                       nodelet::Nodelet);
