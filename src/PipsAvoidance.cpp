
#include "PipsAvoidance.h"






  PipsAvoidance::PipsAvoidance() :
      ready_(false)
  {
   
  }



  void PipsAvoidance::init(ros::NodeHandle nh, tf2_ros::Buffer* tfBuffer, base_frame_id, odom_frame_id)
  {
    nh_ = nh;
    tfBuffer_ = tfBuffer;
    
    base_frame_id_ = base_frame_id;
    odom_frame_id_ = odom_frame_id;
    
    std::string depth_image_topic = "depth/image_raw";
    std::string depth_info_topic = "depth/camera_info";

    depthsub_.subscribe(nh_, depth_image_topic, 10);
    depth_info_sub_.subscribe(nh_, depth_info_topic, 10);
    synced_images.reset(new image_synchronizer(image_synchronizer(10), depthsub_, depth_info_sub_) );
    synced_images->registerCallback(bind(&PipsAvoidance::depthImageCb, this, _1, _2));


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
  
  bool PipsAvoidance::ready()
  {
    return ready_;
  }
  


  
  void PipsAvoidance::depthImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {

    if(DEBUG)std::cout << "depth callback" << std::endl;

    ros::Duration timeout(0);


    if(!ready_) {

        try
        {
          //Get the transform that takes a point in the base frame and transforms it to the depth optical
          geometry_msgs::TransformStamped depth_base_transform = tfBuffer_->lookupTransform(info_msg->header.frame_id, base_frame_id, ros::Time(0));
          
          traj_tester_ = new GenAndTest(co_offsets_, depth_base_transform);
          
          ready_ = true;

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
            //Note: replace odom and base_link with odom_frame_id and base_frame_id
            geometry_msgs::TransformStamped base_transform = tfBuffer_->lookupTransform("odom", "base_link", info_msg->header.stamp, timeout);
            
            ROS_DEBUG_STREAM("base_transform: " << base_transform << std::endl);
            
            //const geometry_msgs::TransformStampedPtr base_transformPtr = geometry_msgs::TransformStampedPtr(new geometry_msgs::TransformStamped(base_transform));
            
            
            auto t1 = std::chrono::high_resolution_clock::now();
            
            //base_transform is actually only being used here to know which frame the trajectory is in...
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
      else
      {
          traj_tester_->test
      }
    
    
   
    }
  }
  


