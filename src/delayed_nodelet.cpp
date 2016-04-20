#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdlib.h>
#include <sensor_msgs/Image.h>

struct delayed_nodelet {
public:
  delayed_nodelet(std::string& topic, std::string& command) : topic(topic), command(command){

  }
  
  void init(ros::NodeHandle& nh)
  {
    boost::shared_ptr<sensor_msgs::Image const> sharedPtr;
    sensor_msgs::Image img;

    sharedPtr  = ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh);
    ROS_INFO("Message received, executing command");
    //if (sharedPtr == NULL)
    //    ROS_ERROR();
    nodelet_thread_ = boost::thread(&delayed_nodelet::start_nodelet, this, command);

  }
  
  ~delayed_nodelet()
  {
    nodelet_thread_.join();
  }

  void start_nodelet(std::string command)
  {
    int ret = system(command.c_str());
  }

private:
  boost::thread nodelet_thread_;
  std::string topic, command;

};

int main(int argc, char **argv)
{
    std::string name= "delayed_nodelet";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;

    std::string topic = "/camera/depth/image_raw";
    std::string command = "roslaunch odroid throttle_images.launch";


    delayed_nodelet delayer(topic, command);
    delayer.init(nh);
    ros::spin();
    ros::shutdown();
}
