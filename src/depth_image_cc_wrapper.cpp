#include <pips_trajectory_testing/depth_image_cc_wrapper.h>


DepthImageCCWrapper::DepthImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
  PipsCCWrapper(nh,pnh,"DepthImageCCWrapper")
{
  cc_ = std::make_shared<pips::collision_testing::DepthImageCollisionChecker>(nh, pnh);
  
}


DepthImageCCWrapper::DepthImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<tf2_ros::Buffer>& tf_buffer) :
  PipsCCWrapper(nh,pnh,"DepthImageCCWrapper", tf_buffer)
{
  cc_ = std::make_shared<pips::collision_testing::DepthImageCollisionChecker>(nh, pnh);
  
}

bool DepthImageCCWrapper::init()
{
    std::string depth_image_topic = "camera/depth/image_raw";
    std::string depth_info_topic = "camera/depth/camera_info";
    ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );

    image_transport::ImageTransport it ( nh_ );
    
    
    //pub_trajectoryProjection = it.advertise ( "trajectoryProjection", 1 );
    
    // Setup subscribers
    depth_sub_.subscribe(it, depth_image_topic, 3);
    depth_info_sub_.subscribe ( nh_, depth_info_topic, 3 );    
    
    // Ensure that CameraInfo is transformable
    info_tf_filter_ = boost::make_shared<tf_filter>(depth_info_sub_, *tf_buffer_, "odom", 2,nh_);
    
    // Synchronize Image and CameraInfo callbacks
    image_synchronizer_ = boost::make_shared<time_synchronizer>(time_synchronizer(10),depth_sub_, *info_tf_filter_);
    image_synchronizer_->registerCallback(boost::bind(&DepthImageCCWrapper::depthImageCb, this, _1, _2));

    return true;

}


void DepthImageCCWrapper::update()
{
    if ( isReady ( current_image->header ) ) {
        cc_->setImage ( current_image, current_camInfo );
        /*
        if ( _visualize ) {
            boost::mutex::scoped_lock l ( copy_mutex );
            imageVisualize = current_raw_image.clone();
            trajectoryProjectionHeader = current_image->header;
            visualizeReady = true;
        }
        */
    }
}

    
    
    
    
// Could separate the image and camera_info callbacks to allow non synchronized messages
void DepthImageCCWrapper::depthImageCb ( const sensor_msgs::Image::ConstPtr& image_msg,
        const sensor_msgs::CameraInfo::ConstPtr& info_msg
        )
{
    if ( image_msg->header.stamp == ros::Time ( 0 ) || info_msg->header.stamp == ros::Time ( 0 ) ) { // Gazebo occasionally publishes Image and CameraInfo messages with time=0
        ROS_WARN_STREAM_NAMED ( name_,"Bad timestamp" );
        return;
    }


    if ( isReady ( image_msg->header ) ) {

        //Update tester with new data
        ROS_DEBUG_STREAM_NAMED ( name_, "Updating collision checker image" );
        current_image = image_msg;
        current_camInfo = info_msg;
        /*
        if ( _visualize ) {
            try {
                boost::shared_ptr<void const> tracked_object_disp;
                boost::mutex::scoped_lock l ( copy_mutex );
                current_raw_image = cv_bridge::toCvCopy ( *raw_image_msg, sensor_msgs::image_encodings::BGR8 )->image;
            } catch ( ... ) {
                ROS_ERROR ( "Exception within cost map callback function\n" );
            }
        }
        */


    }
}

