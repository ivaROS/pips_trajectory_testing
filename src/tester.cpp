#include <ros/ros.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::min
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>
#include "collision_checker.h"



class TestTrajectory
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_,depthsubit_;
  image_transport::Publisher pub_,depthpub_,depthpub2_;
  message_filters::Subscriber<sensor_msgs::Image> depthsub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  CvFont font_;
  bool generate,firstDepthFrame_;

  tf::StampedTransform starting_frame_transform_,depth_starting_frame_transform_;
  tf2_ros::StaticTransformBroadcaster br;
  ros::Timer timer, depth_timer;
  std::vector<cv::Point3d> co_offsets_;
  CollisionChecker* cc_;
  GenAndTest* traj_tester_;


    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo> image_sync_policy;
    typedef message_filters::Synchronizer<image_sync_policy> image_synchronizer;
    boost::shared_ptr<image_synchronizer> synced_images;

public:
  TestTrajectory()
    : it_(nh_), tf_listener_(tfBuffer_), firstDepthFrame_(true), generate(false)
  {


    std::string depth_image_topic = nh_.resolveName("depth_image");
    std::string depth_info_topic = nh_.resolveName("depth_info");
    
    //depthsubit_ = it_.subscribeCamera(depth_image_topic, 10, &FrameDrawer::depthImageCb, this);

    depthsub_.subscribe(nh_, depth_image_topic, 10);
    depth_info_sub_.subscribe(nh_, depth_info_topic, 10);
    synced_images.reset(new image_synchronizer(image_synchronizer(10), depthsub_, depth_info_sub_) );
    synced_images->registerCallback(bind(&TestTrajectory::depthImageCb, this, _1, _2));


    double radius = .25;
    double height = .7;
    double clearance = .05;

    cv::Point3d topr(radius,-height,radius);
    cv::Point3d topl(-radius,-height,radius);
    cv::Point3d bottomr(radius,-clearance,radius);
    cv::Point3d bottoml(-radius,-clearance,radius);


    cv::Point3d offsets[] = {topr,topl,bottoml,bottomr};
    std::vector<cv::Point3d> co_offsets(offsets, offsets + sizeof(offsets) / sizeof(cv::Point3d) );
    co_offsets_ = co_offsets;
  }



  void depthImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {

    std::cout << "depth callback" << std::endl;

    std::string stationary_frame("odom");
    std::string tracking_frame_id("base_link");
    ros::Duration timeout(1.0 / 30);


    if(firstDepthFrame_) {

        try
        {
          //Get the transform that takes a point in the base frame and transforms it to the depth optical
          geometry_msgs::TransformStamped depth_base_transform = tfBuffer_.lookupTransform(info_msg->header.frame_id, "base_link", ros::Time(0), timeout);
          
          traj_tester_ = new GenAndTest(co_offsets_, depth_base_transform);
          
          firstDepthFrame_ = false;

          ROS_INFO("Created trajectory testing instance");

        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          return;
        }
    }
    
    if(generate)
    {
        generate = false;
        try
        {
          //Get the transform that takes point in base frame and transforms it to odom frame
          geometry_msgs::TransformStamped base_transform = tfBuffer_.lookupTransform("odom", "base_link", info_msg->header.stamp, timeout);
          
          const geometry_msgs::TransformStampedPtr base_transformPtr = geometry_msgs::TransformStampedPtr(new geometry_msgs::TransformStamped(base_transform));
          
          traj_tester_->run(image_msg, info_msg, base_transformPtr);
          
          
          
          

        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("[draw_frames] TF exception:\n%s",ex.what());
          return;
        }

 
    }
    
    
   
  }
  
   void depthImageInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {

    std::cout << "depth info callback" << std::endl;

    std::string stationary_frame("odom");
    std::string tracking_frame_id("base_link");
    std::string starting_frame_id("depth_start_frame");
    ros::Duration timeout(1.0 / 30);




        try
        {
          //Get the transform that takes a point in the base frame and transforms it to the depth optical
          geometry_msgs::TransformStamped depth_base_transform = tfBuffer_.lookupTransform(info_msg->header.frame_id, "base_link", ros::Time(0), timeout);
          
          traj_tester_ = new GenAndTest(co_offsets_, depth_base_transform);
          
          cc_ = new CollisionChecker(image_msg, info_msg, depth_base_transform, co_offsets_, false);
          
          ROS_INFO("Saved first depth frame");

        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          return;
        }



    
      
      geometry_msgs::TransformStamped transform;
        try
        {
          ros::Time acquisition_time = info_msg->header.stamp;
          transform = tfBuffer_.lookupTransform("depth_start_frame", ros::Time(0), "base_link", ros::Time(0), "odom", timeout);
          
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("[draw_frames] TF exception:\n%s",ex.what());
          return;
        }


      geometry_msgs::Vector3 pt = transform.transform.translation;    
      double coords[3];
      coords[0] = pt.x;
      coords[1] = pt.y;
      coords[2] = pt.z;
      
      cc_->testCollision(coords);
    
    
   
  }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectory");
  TestTrajectory tester();
  ros::spin();
}
