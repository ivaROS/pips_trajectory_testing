
#include "TestTrajectory.h"






  TestTrajectory::TestTrajectory() :
      firstDepthFrame_(true),
      generate(true)
  {
   
  }

  void TestTrajectory::init(ros::NodeHandle& nh)
  {
    nh_ = nh;
    it_ = new image_transport::ImageTransport(nh_);
    tfBuffer_ = new tf2_ros::Buffer; //optional parameter: ros::Duration(cache time) (default=10)
    tf_listener_ = new tf2_ros::TransformListener(*tfBuffer_);

  

    std::string depth_image_topic = "camera/depth/image_raw"; //nh_.resolveName("depth_image");
    std::string depth_info_topic = "camera/depth/camera_info"; //nh_.resolveName("depth_info");
    
    //depthsubit_ = it_.subscribeCamera(depth_image_topic, 10, &TestTrajectory::depthImageCb, this);

    trigger_sub_ = nh_.subscribe("enable", 10, &TestTrajectory::trigger, this);


    ROS_INFO_STREAM("Initializing node. Depth image topic: " << depth_image_topic << "; depth info topic: " << depth_info_topic);
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
  

  void TestTrajectory::trigger(const std_msgs::Empty& msg)
  {
      generate = true;
  
  }

  void TestTrajectory::depthImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {

    if(DEBUG)std::cout << "depth callback" << std::endl;

    std::string stationary_frame("odom");
    std::string base_frame_id("base_link");
    ros::Duration timeout(1.0 / 30);


    if(firstDepthFrame_) {

        try
        {
          //Get the transform that takes a point in the base frame and transforms it to the depth optical
          geometry_msgs::TransformStamped depth_base_transform = tfBuffer_->lookupTransform(info_msg->header.frame_id, base_frame_id, ros::Time(0), timeout);
          
          traj_tester_ = new GenAndTest(co_offsets_, depth_base_transform);
          
          firstDepthFrame_ = false;

          if(DEBUG)ROS_INFO("Created trajectory testing instance");

        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          return;
        }
    }
    else
    {
      if(generate)
      {
          //generate = false;
          try
          {
            //Get the transform that takes point in base frame and transforms it to odom frame
            geometry_msgs::TransformStamped base_transform = tfBuffer_->lookupTransform("odom", "base_link", info_msg->header.stamp, timeout);
            
            ROS_DEBUG_STREAM("base_transform: " << base_transform << std::endl);
            
            //const geometry_msgs::TransformStampedPtr base_transformPtr = geometry_msgs::TransformStampedPtr(new geometry_msgs::TransformStamped(base_transform));
            
            
            auto t1 = std::chrono::high_resolution_clock::now();

            traj_tester_->run(image_msg, info_msg, base_transform);
            
            auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
            ROS_INFO_STREAM("Trajectory gen/test took " << fp_ms.count() << " ms" << std::endl);
            

          }
          catch (tf2::TransformException &ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s",ex.what());
            return;
          }

   
      }
    
    
   
    }
  }
  


