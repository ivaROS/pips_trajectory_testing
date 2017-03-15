#include pips_controller_impl.h
 
 namespace kobuki
{


  void PipsTrajectoryController::depthImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
               const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    DepthDataPtr depthData = std::make_shared<DepthData>(image_msg, info_msg);
    sensorCb(depthData);
 }



  void PipsTrajectoryController::setupPublishersSubscribers()
  {
    ObstacleAvoidanceController::setupPublishersSubscribers();
    
    std::string depth_image_topic = "/camera/depth/image_raw";
    std::string depth_info_topic = "/camera/depth/camera_info";

    ROS_DEBUG_STREAM_NAMED(name_,  "Setting up publishers and subscribers");

    depthsub_.subscribe(nh_, depth_image_topic, 10);
    depth_info_sub_.subscribe(nh_, depth_info_topic, 10);
    synced_images.reset(new image_synchronizer(image_synchronizer(10), depthsub_, depth_info_sub_) );
    synced_images->registerCallback(bind(&ObstacleAvoidanceController::depthImageCb, this, _1, _2));
  }
  
}
