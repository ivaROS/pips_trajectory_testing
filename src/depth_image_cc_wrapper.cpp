#include <pips_trajectory_testing/depth_image_cc_wrapper.h>

namespace pips_trajectory_testing
{

DepthImageCCWrapper::DepthImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) :
    PipsCCWrapper(nh,pnh,name,tfm)
{
    cc_ = std::make_shared<pips::collision_testing::DepthImageCollisionChecker>(nh, pnh_);
}


DepthImageCCWrapper::DepthImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) :
    PipsCCWrapper(nh,pnh,name, tfm)
{
    cc_ = std::make_shared<pips::collision_testing::DepthImageCollisionChecker>(nh, pnh_);
}

bool DepthImageCCWrapper::init()
{
  if(!inited_)
  {
    PipsCCWrapper::init();
  
    // Get topic names
    std::string depth_image_topic="/camera/depth/image_raw", depth_info_topic= "/camera/depth/camera_info";
    
    pnh_.getParam("depth_image_topic", depth_image_topic );
    pnh_.getParam("depth_info_topic", depth_info_topic );
    
    // The idea here is to set the parameter on the parameter server to the default value to make it easier to see what it is.
    pnh_.setParam("depth_image_topic", depth_image_topic);
    pnh_.setParam("depth_info_topic", depth_info_topic );

    //TODO: Replace this with the 'fixed_frame_id_' variable from PipsCCWrapper
    std::string odom_frame_id = "odom";

    //NOTE: It isn't clear to me whether it matters if I use pnh_ or nh_
    std::string key;
    if (pnh_.searchParam("odom_frame_id", key))
    {
      pnh_.getParam(key, odom_frame_id );
      pnh_.setParam(key, odom_frame_id );
    }
    
    ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );

    image_transport::ImageTransport it ( nh_ );
    
    
    //pub_trajectoryProjection = it.advertise ( "trajectoryProjection", 1 );
    
    // Setup subscribers
    depth_sub_.subscribe(it, depth_image_topic, 3);
    depth_info_sub_.subscribe ( nh_, depth_info_topic, 3 );    
    
    // Ensure that CameraInfo is transformable
    info_tf_filter_ = boost::make_shared<tf_filter>(depth_info_sub_, *tfm_.getBuffer(), PipsCCWrapper::fixed_frame_id_, 2,nh_);
    
    // Synchronize Image and CameraInfo callbacks
    image_synchronizer_ = boost::make_shared<time_synchronizer>(time_synchronizer(10),depth_sub_, *info_tf_filter_);
    image_synchronizer_->registerCallback(boost::bind(&DepthImageCCWrapper::depthImageCb, this, _1, _2));
    
    inited_ = true;
  }
    return true;

}


void DepthImageCCWrapper::update()
{
    ROS_DEBUG_STREAM_NAMED ( name_, "Updating collision checker image" );
    
    Lock(mutex_);
    cc_->setImage ( current_image, current_camInfo );
}

bool DepthImageCCWrapper::isReadyImpl()
{
    if(current_camInfo && current_image)
    {
      return true;
    }
    else
    {
      ROS_ERROR_STREAM_COND_NAMED(!current_camInfo, name_, "No current CameraInfo message!");
      ROS_ERROR_STREAM_COND_NAMED(!current_image, name_, "No current Image message!");
    }
    return false;
}

    
// Could separate the image and camera_info callbacks to allow non synchronized messages
void DepthImageCCWrapper::depthImageCb ( const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg )
{
    if ( image_msg->header.stamp == ros::Time ( 0 ) || info_msg->header.stamp == ros::Time ( 0 ) ) { // Gazebo occasionally publishes Image and CameraInfo messages with time=0
        ROS_WARN_STREAM_NAMED ( name_,"Bad timestamp" );
        return;
    }

    {
      Lock(mutex_);
      current_image = image_msg;
      current_camInfo = info_msg;
    }

    doCallback();
}

std_msgs::Header DepthImageCCWrapper::getCurrentHeader()
{
    if(current_camInfo)
    {
        return current_camInfo->header;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(name_, "Trying to get current header, but none exists!");
        return std_msgs::Header();
    }
}


}

