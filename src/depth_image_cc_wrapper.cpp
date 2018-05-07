#include <pips_trajectory_testing/depth_image_cc_wrapper.h>

namespace pips_trajectory_testing
{

DepthImageCCWrapper::DepthImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    PipsCCWrapper(nh,pnh,"depth_image_cc_wrapper")
{
    cc_ = std::make_shared<pips::collision_testing::DepthImageCollisionChecker>(nh, pnh_);
}


DepthImageCCWrapper::DepthImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string& name) :
    PipsCCWrapper(nh,pnh,name, tf_buffer)
{
    cc_ = std::make_shared<pips::collision_testing::DepthImageCollisionChecker>(nh, pnh_);
}

bool DepthImageCCWrapper::init()
{    
    // Get topic names
    std::string depth_image_topic="/camera/depth/image_raw", depth_info_topic= "/camera/depth/camera_info";
    
    pnh_.getParam("depth_image_topic", depth_image_topic );
    pnh_.getParam("depth_info_topic", depth_info_topic );
    
    // The idea here is to set the parameter on the parameter server to the default value to make it easier to see what it is.
    pnh_.setParam("depth_image_topic", depth_image_topic);
    pnh_.setParam("depth_info_topic", depth_info_topic );

    std::string odom_frame_id = "odom";
    nh_.getParam("odom_frame_id", odom_frame_id);
    nh_.setParam("odom_frame_id", odom_frame_id);
    //   nh_.setParam("odom_frame_id_t", odom_frame_id);

    // TODO: use parameters for base_frame_id and odom_frame_id
    
    ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );

    image_transport::ImageTransport it ( nh_ );
    
    
    //pub_trajectoryProjection = it.advertise ( "trajectoryProjection", 1 );
    
    // Setup subscribers
    depth_sub_.subscribe(it, depth_image_topic, 3);
    depth_info_sub_.subscribe ( nh_, depth_info_topic, 3 );    
    
    // Ensure that CameraInfo is transformable
    info_tf_filter_ = boost::make_shared<tf_filter>(depth_info_sub_, *tf_buffer_, odom_frame_id, 2,nh_);
    
    // Synchronize Image and CameraInfo callbacks
    image_synchronizer_ = boost::make_shared<time_synchronizer>(time_synchronizer(10),depth_sub_, *info_tf_filter_);
    image_synchronizer_->registerCallback(boost::bind(&DepthImageCCWrapper::depthImageCb, this, _1, _2));

    return true;

}


void DepthImageCCWrapper::update()
{
    ROS_DEBUG_STREAM_NAMED ( name_, "Updating collision checker image" );
    cc_->setImage ( current_image, current_camInfo );
}

    
bool DepthImageCCWrapper::isReady(const std_msgs::Header& header)
{
    return current_camInfo && current_image && PipsCCWrapper::isReady(current_camInfo->header);
}
    
// TODO: Add a mutex to coordinate 'update' and 'depthImageCb'
// Could separate the image and camera_info callbacks to allow non synchronized messages
void DepthImageCCWrapper::depthImageCb ( const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg )
{
    if ( image_msg->header.stamp == ros::Time ( 0 ) || info_msg->header.stamp == ros::Time ( 0 ) ) { // Gazebo occasionally publishes Image and CameraInfo messages with time=0
        ROS_WARN_STREAM_NAMED ( name_,"Bad timestamp" );
        return;
    }


    current_image = image_msg;
    current_camInfo = info_msg;

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
        return std_msgs::Header();
    }
}


}

