#include <pips_trajectory_testing/pips_cc_wrapper.h>

namespace pips_trajectory_testing
{

  
  PipsCCWrapper::PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) :
    nh_(nh),
    pnh_(pnh),
    name_(name)
    
    {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(); //optional parameter: ros::Duration(cache time) (default=10) (though it doesn't seem to accept it!)
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
  
  PipsCCWrapper::PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, std::shared_ptr<tf2_ros::Buffer>& tf_buffer) :
    nh_(nh),
    pnh_(pnh),
    name_(name),
    tf_buffer_(tf_buffer)
  {
  }
  
  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool PipsCCWrapper::isReady ( const std_msgs::Header& header )
  {

      ros::Duration timeout ( 0 ); //Could be used for transform lookup?

      if ( !hasTransform_ ) {
          ROS_DEBUG_STREAM_ONCE_NAMED ( name_, "Not ready, check for transform..." );
          try {
              //Get the transform that takes a point in the base frame and transforms it to the depth optical
              auto sensor_base_transform = tf_buffer_->lookupTransform ( header.frame_id, base_frame_id_, ros::Time ( 0 ) );
              getCC()->setTransform ( sensor_base_transform );

              getCC()->init();

              ROS_DEBUG_STREAM_NAMED ( name_,  "Transform found! Passing transform to collision checker" );
              hasTransform_ = true;

          } catch ( tf2::TransformException &ex ) {
              ROS_WARN_STREAM_THROTTLE_NAMED ( 5, name_, "Problem finding transform:\n" <<ex.what() );
              return false;
          }
      }
      return true;
  }

}