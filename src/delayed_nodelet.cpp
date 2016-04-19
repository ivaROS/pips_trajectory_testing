#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdlib.h>
#include <sensor_msgs/Image.h>

struct delayed_nodelet {
public:
  delayed_nodelet()  {
    // advertise, subscribe or whatever using the my_queue_ CallbackQueue

    

  }
  
  void init(ros::NodeHandle& nh)
  {
    nh_ = nh;
    trigger_subscriber_ = nh_.subscribe("/camera/depth/image_raw", 1, &delayed_nodelet::trigger_callback_, this);
    command1 = "roslaunch odroid throttle_images.launch";
  }
  
  ~delayed_nodelet()
  {
    nh_.shutdown();
    nodelet_thread1_.join();
  }



  void trigger_callback_(sensor_msgs::Image msg)
  {
    nodelet_thread1_ = boost::thread(&delayed_nodelet::start_nodelet, this, command1);
    trigger_subscriber_.shutdown();
  }

  void start_nodelet(std::string command)
  {
    int ret = system(command.c_str());
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber trigger_subscriber_;
  boost::thread nodelet_thread1_;
  std::string command1;
};

int main(int argc, char **argv)
{
    std::string name= "delayed_nodelet";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;

    delayed_nodelet delayer;
    delayer.init(nh);
    ros::spin();
}
