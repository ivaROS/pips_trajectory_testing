#include <pips_trajectory_testing/pips_cc_wrapper.h>

namespace pips_trajectory_testing
{

  /*
  PipsCCWrapper::PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) :
    nh_(nh),
    pnh_(pnh, name),
    name_(name)
    {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(); //optional parameter: ros::Duration(cache time) (default=10) (though it doesn't seem to accept it!)
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    */
  
  PipsCCWrapper::PipsCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) :
    nh_(nh),
    pnh_(pnh, name),
    name_(name),
    tfm_(tfm, nh)
  {

  }
  
  bool PipsCCWrapper::init()
  {
    //NOTE: It isn't clear to me whether it matters if I use pnh_ or nh_
    std::string key;
    if (pnh_.searchParam("base_frame_id", key))
    {
      pnh_.getParam(key, base_frame_id_ );
      //nh_.setParam(key, base_frame_id_ );
    }
    else
    {
      ROS_WARN_STREAM_NAMED(name_, "Warning, no entry found on parameter server for 'base_frame_id'! Using default value: '" << base_frame_id_ << "'");
    }
    
    if (pnh_.searchParam("fixed_frame_id", key))
    {
      pnh_.getParam(key, fixed_frame_id_ );
      //nh_.setParam(key, base_frame_id_ );
    }
    else
    {
      ROS_WARN_STREAM_NAMED(name_, "Warning, no entry found on parameter server for 'fixed_frame_id'! Using default value: '" << fixed_frame_id_ << "'");
    }
    
    return true;
  }
  
  void PipsCCWrapper::setBaseFrame(const std::string& base_frame_id)
  {
      base_frame_id_ = base_frame_id;
  }
  
  bool PipsCCWrapper::isReady()
  {
    if(isReadyImpl())
    {
      return transformReady(getCurrentHeader());
    }
    else
    {
      return false;
    }
  }
  
  bool PipsCCWrapper::isReady(const std_msgs::Header& header)
  {
    if(isReadyImpl())
    {
      return transformReady(getCurrentHeader(), header);
    }
    else
    {
      return false;
    }
  }
  
  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool PipsCCWrapper::transformReady ( const std_msgs::Header& header )
  {

      ros::Duration timeout ( 0 ); //Could be used for transform lookup?

      if ( !hasTransform_ ) {
          ROS_DEBUG_STREAM_THROTTLE_NAMED (1, name_, "Not ready, check for transform..." );
          try {
              //Get the transform that takes a point in the base frame and transforms it to the depth optical
              geometry_msgs::TransformStamped sensor_base_transform = tfm_.getBuffer()->lookupTransform ( header.frame_id, base_frame_id_, ros::Time ( 0 ) );
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
  
  bool PipsCCWrapper::transformReady ( const std_msgs::Header& target_header, const std_msgs::Header& source_header)
  {
    
    ros::Duration timeout ( 0 ); //Could be used for transform lookup?
    
    ROS_DEBUG_STREAM_ONCE_NAMED ( name_, "Not ready, check for transform..." );
    try {
      //Get the transform that takes a point in the base frame and transforms it to the depth optical
      geometry_msgs::TransformStamped sensor_base_transform = tfm_.getBuffer()->lookupTransform ( target_header.frame_id, target_header.stamp, source_header.frame_id, source_header.stamp, fixed_frame_id_, timeout);
      getCC()->setTransform ( sensor_base_transform );
      
      getCC()->init();
      
      ROS_DEBUG_STREAM_NAMED(name_+".transform", "transform: from [" << target_header.frame_id << "] at " << target_header.stamp << " to [" << source_header.frame_id << "] at " << source_header.stamp << ": " << toString(sensor_base_transform));
      
      
      ROS_DEBUG_STREAM_NAMED ( name_,  "Transform found! Passing transform to collision checker" );
      
    } catch ( tf2::TransformException &ex ) {
      ROS_WARN_STREAM_THROTTLE_NAMED ( 5, name_, "Problem finding transform:\n" <<ex.what() );
      return false;
    }
    
    return true;
  }
  
  void PipsCCWrapper::defaultCallback()
  {
      if(isReady())
      {
        update();
      }
  }
  
  void PipsCCWrapper::autoUpdate()
  {
    setCallback(boost::bind(&PipsCCWrapper::defaultCallback, this));
  }
  
  void PipsCCWrapper::setCallback(Callback cb)
  {
      cb_ = cb;
  }
  
  void PipsCCWrapper::doCallback()
  {
      if(cb_)
      {
          cb_();
      }
  }

}
