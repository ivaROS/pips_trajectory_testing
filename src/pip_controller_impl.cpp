#include pips_controller_impl.h
 
 namespace kobuki
{

  // Could separate the image and camera_info callbacks to allow non synchronized messages
  void PipsTrajectoryController::depthImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
               const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    if(image_msg->header.stamp == ros::Time(0) || info_msg->header.stamp == ros::Time(0))  // Gazebo occasionally publishes Image and CameraInfo messages with time=0
    {
      ROS_WARN_STREAM_NAMED( name_,"Bad timestamp");
      return;
    }
    
    
    //Update tester with new data
    ROS_DEBUG_STREAM_NAMED(name_, "Updating collision checker image");
    cc_->setImage(image_msg, info_msg);
    
    //Let controller know that we have new sensor data
    sensorCb(image_msg->header.stamp);
 }

  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool isReady(const std_msgs::Header& header)
  {
    ROS_DEBUG_STREAM_ONCE_NAMED(name_, "Not ready, check for transform...");
    try
    {
      //Get the transform that takes a point in the base frame and transforms it to the depth optical
      geometry_msgs::TransformStamped sensor_base_transform = tfBuffer_->lookupTransform(header.frame_id, base_frame_id_, ros::Time(0));
      cc_->setTransform(sensor_base_transform);

      ROS_DEBUG_STREAM_NAMED(name_,  "Transform found! Passing transform to collision checker");

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM_THROTTLE_NAMED(5, name_, "Problem finding transform:\n" <<ex.what());
      return false;
    }
    return true;
  }
  
  // This will need to generate a derived collision checker class
  CollisionChecker_ptr getCollisionChecker()
  {
    cc_ = std::make_shared<CollisionChecker>();
    return cc_;
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
