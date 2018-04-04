#include "pips_controller.h"

 namespace kobuki
{
  PipsTrajectoryController::PipsTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    ObstacleAvoidanceController(nh, pnh), nh_(nh, name_), pnh_(pnh, name_)
  {
      cc_ = std::make_shared<pips::collision_testing::DepthImageCollisionChecker>(nh_, pnh_);
      traj_tester_->setCollisionChecker(cc_);
      
  }





  // Could separate the image and camera_info callbacks to allow non synchronized messages
  void PipsTrajectoryController::depthImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
               const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    if(image_msg->header.stamp == ros::Time(0) || info_msg->header.stamp == ros::Time(0))  // Gazebo occasionally publishes Image and CameraInfo messages with time=0
    {
      ROS_WARN_STREAM_NAMED( name_,"Bad timestamp");
      return;
    }
    
    
    if(isReady(image_msg->header))
    {
      //Update tester with new data
      ROS_DEBUG_STREAM_NAMED(name_, "Updating collision checker image");
      cc_->setImage(image_msg, info_msg);
      
      //Let controller know that we have new sensor data
      ObstacleAvoidanceController::sensorCb(image_msg->header);
    }
 }

  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool PipsTrajectoryController::isReady(const std_msgs::Header& header)
  {
    if(!ObstacleAvoidanceController::isReady(header))
    {
      return false;
    }

    ros::Duration timeout(0); //Could be used for transform lookup?
    
    if(!hasTransform_)
    {
      ROS_DEBUG_STREAM_ONCE_NAMED(name_, "Not ready, check for transform...");
      try
      {
        //Get the transform that takes a point in the base frame and transforms it to the depth optical
        geometry_msgs::TransformStamped sensor_base_transform = tfBuffer_->lookupTransform(header.frame_id, base_frame_id_, ros::Time(0));
        
        //cc_ = std::make_shared<CollisionChecker>(nh_, pnh_);  // it appears that the first reconfigure call happens as part of the constructor in some way, meaning that it happened before the next line here
        cc_->setTransform(sensor_base_transform);
        //traj_tester_->setCollisionChecker(cc_);

        cc_->init();

        ROS_DEBUG_STREAM_NAMED(name_,  "Transform found! Passing transform to collision checker");
        hasTransform_ = true;

      }
      catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM_THROTTLE_NAMED(5, name_, "Problem finding transform:\n" <<ex.what());
        return false;
      }
    }
    return true;
  }
  
  
  bool PipsTrajectoryController::init()
  {
    ObstacleAvoidanceController::init();
    
    //TODO: change these to just 'image' and 'camera_info' to facilitate easier remapping
    std::string depth_image_topic = "/image";
    std::string depth_info_topic = "/camera_info";

    ROS_DEBUG_STREAM_NAMED(name_,  "Setting up publishers and subscribers");

    depthsub_.subscribe(ObstacleAvoidanceController::nh_, depth_image_topic, 1);
    depth_info_sub_.subscribe(ObstacleAvoidanceController::nh_, depth_info_topic, 1);
    synced_images.reset(new image_synchronizer(image_synchronizer(10), depthsub_, depth_info_sub_) );
    synced_images->registerCallback(bind(&PipsTrajectoryController::depthImageCb, this, _1, _2));
    
    return true;
  }
  
}
