#include "pips_trajectory_tester.h"

#include <pips/collision_testing/collision_checker.h>
#include <trajectory_generator_ros_interface.h>
#include <pips_trajectory_testing/PipsTrajectoryTesterConfig.h>

#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>

#include <fstream>
#include <memory>
#include <chrono>

#include <omp.h>

//Generates a straight line trajectory with a given angle and speed
class angled_straight_traj_func : public traj_func{

    double dep_angle_;
    double v_;

public:
    angled_straight_traj_func( double dep_angle, double v ) : dep_angle_(dep_angle), v_(v) { }
    
    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[near_identity::XD_IND] = v_*cos( dep_angle_);
        dxdt[near_identity::YD_IND] = v_*sin( dep_angle_);
    }
    
    
};



GenAndTest::GenAndTest(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
nh_(nh, name_), pnh_(pnh, name_)
{
    traj_gen_bridge_ = std::make_shared<TrajectoryGeneratorBridge>();
    params_ = std::make_shared<traj_params>(traj_gen_bridge_->getDefaultParams());
}


void GenAndTest::setCollisionChecker(CollisionChecker_ptr cc)
{
  cc_ = cc;
}

void GenAndTest::init()
{

    //Create the various visualization publishers
    path_pub_ = nh_.advertise<pips_msgs::PathArray>("tested_paths", 5);
    desired_path_pub_ = nh_.advertise<pips_msgs::PathArray>("desired_paths", 5);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collision_points", 5);

    
    reconfigure_server_.reset( new ReconfigureServer(pnh_));
    reconfigure_server_->setCallback(boost::bind(&GenAndTest::configCB, this, _1, _2));
    /*
    std::string key;

    if(ros::param::search("enable_parallel_loop", key))
    {
      ros::param::get(key, parallelism_enabled_);
    }
    
    
    GenAndTest::updateParams();
    
    */
}


void GenAndTest::configCB(pips_trajectory_testing::PipsTrajectoryTesterConfig &config, uint32_t level) {
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request: tf=" << config.tf << ", parallelism=" << (config.parallelism?"True":"False"));
    
    parallelism_enabled_ = config.parallelism;
    params_->tf = config.tf;
    params_->dt = config.dt;
    
    params_->cp = config.cp;
    params_->cd = config.cd;
    params_->cl = config.cl;
    params_->eps = config.eps;
    
    params_->a_max = config.a_max;
    params_->v_max = config.v_max;
    params_->w_max = config.w_max;
    params_->w_dot_max = config.w_dot_max;
    
}


std::vector<ni_trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions)
{
    state_type x0 = traj_gen_bridge_->initState();
    return GenAndTest::run(trajectory_functions, x0);
}

//OdometryPtr is not passed as a reference in this instance: we want a copy to be made of the boost::shared_ptr, so that this instance will be constant even if the calling function assigns a new message to curr_odom
std::vector<ni_trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions, const nav_msgs::Odometry::ConstPtr curr_odom)
{
    state_type x0 = traj_gen_bridge_->initState(curr_odom);
    std_msgs::Header header;
    header.stamp = curr_odom->header.stamp;
    header.frame_id = curr_odom->child_frame_id;
    return GenAndTest::run(trajectory_functions, x0, header);
}


std::vector<ni_trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0)
{
    return GenAndTest::run(trajectory_functions, x0, header_);
}

//This is the lowest level version that is actually run; the rest are for convenience
std::vector<ni_trajectory_ptr> GenAndTest::run(std::vector<traj_func_ptr>& trajectory_functions, state_type& x0, std_msgs::Header& header)
{
    num_frames=num_frames+1;
    
    ROS_DEBUG_STREAM_NAMED(name_, "Num frames: " << num_frames);
    
    
    ROS_DEBUG_STREAM_NAMED(name_, "Generating Trajectories");

    if(num_frames == 1)
    {
        /* Should use a service call to trigger the generation+publication */
        //generateDepthImages(trajectory_functions, x0, header);
    }
    
    
    size_t num_paths = trajectory_functions.size();
    
    std::vector<ni_trajectory_ptr> trajectories(num_paths); //std::vector<boost::shared_ptr<PipsTrajectory*>>
    
    //Start timer
    auto t1 = std::chrono::high_resolution_clock::now();

    double coords[3];
    coords[0] = min_dist;
    coords[1] = 0;
    coords[2] = 0;

    if(~cc_->testCollision(coords))
    {
        ROS_DEBUG_STREAM_NAMED(name_, "Parallelism: " << parallelism_enabled_);
        //Perform trajectory generation and collision detection in parallel if enabled
        //Vectors and arrays must be accessed by indicies to ensure thread safe behavior
#pragma omp parallel for schedule(dynamic) if(parallelism_enabled_) //schedule(dynamic)
        for(size_t i = 0; i < num_paths; i++)
        {

            pips_trajectory_ptr traj = std::make_shared<PipsTrajectory>();
            traj->header = header;
            traj->params = params_;

            traj->trajpntr = trajectory_functions[i];
            traj->x0_ = x0;

            traj_gen_bridge_->generate_trajectory(traj);

            evaluateTrajectory(traj);

            trajectories[i] = traj;


            if(omp_in_parallel())
            {
              int thread_id = omp_get_thread_num();
              auto t2 = std::chrono::high_resolution_clock::now();
              std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

              ROS_DEBUG_STREAM_NAMED(name_,"OpenMP active! Thread # " << thread_id << " completed in " << fp_ms.count() << "ms");
            
             }

        }
    }
    else
    {
        trajectories.resize(0);
    }
    

    //End timer
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

    
    ROS_DEBUG_STREAM_NAMED(name_, "Generated " << num_paths << " trajectories in " << fp_ms.count() << " ms");

    TrajectoryGeneratorBridge::publishPaths(path_pub_, trajectories);
    TrajectoryGeneratorBridge::publishDesiredPaths(desired_path_pub_, trajectories);

    return trajectories;
}


void GenAndTest::evaluateTrajectory(pips_trajectory_ptr& traj)
{
    ni_trajectory_ptr ni_traj= std::static_pointer_cast<ni_trajectory>(traj);
    int collision_ind = GenAndTest::evaluateTrajectory(ni_traj);
    traj->set_collision_ind(collision_ind);
}

//Test whether trajectory collides
//Idea: move this to header, template it, and make the type of pose retrieved depend on the type desired 
//That way, approaches without orientation info will end up calling the Point version directly (perhaps stamped?)
int GenAndTest::evaluateTrajectory(ni_trajectory_ptr& traj)
{

    for(size_t i = 0; i < traj->num_states(); i++)
    {

        geometry_msgs::Pose pose = traj->getPose(i);

        if(pose.position.x > min_dist)
        {

            if(cc_->testCollision(pose))
            {
                return i;

            }
        }

    }

    return -1;

}

int GenAndTest::evaluateTrajectory(pips_trajectory_msgs::trajectory_points& trajectory)
{

    for(size_t i = 0; i < trajectory.points.size(); i++)
    {
        //TODO: make sure collision_checker can handle input of type pips_trajectory_msgs::trajectory_point then remove this conversion
        pips_trajectory_msgs::trajectory_point pt = trajectory.points[i]; 
        double coords[3];
        coords[0] = pt.x;
        coords[1] = pt.y;
        coords[2] = 0;
        
        if(cc_->testCollision(coords))
        {
            return i;
        }

    }

    return -1;

}


std::vector<traj_func_ptr> GenAndTest::getDefaultTrajectoryFunctions()
{

    //Set trajectory departure angles and speed
    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4};
    double v = .25;

    size_t num_paths = dep_angles.size();
    
    std::vector<traj_func_ptr> trajectory_functions(num_paths);
    
    for(size_t i = 0; i < num_paths; i++)
    {
        double dep_angle = dep_angles[i];
        trajectory_functions[i] = std::make_shared<angled_straight_traj_func>(dep_angle, v);
    }
    return trajectory_functions;
}


// additional function for generate dense trajectory functions
std::vector<traj_func_ptr> GenAndTest::getDenseTrajectoryFunctions()
{

    //Set trajectory departure angles and speed
    // angle range:
    // [-k : k]*interval_ang
    double interval_ang = 0.04;
    int k = 10;
    std::vector<double> dep_angles;
    for (int i = -k; i < k; ++ i) {
        dep_angles.push_back(double(i)*interval_ang);
    }
    //    std::vector<double> dep_angles = {-.4,-.2,0,.2,.4};
    double v = .25;

    size_t num_paths = dep_angles.size();

    std::vector<traj_func_ptr> trajectory_functions(num_paths);

    for(size_t i = 0; i < num_paths; i++)
    {
        double dep_angle = dep_angles[i];
        trajectory_functions[i] = std::make_shared<angled_straight_traj_func>(dep_angle, v);
    }
    return trajectory_functions;
}


/* TODO: The following 2 methods are really the only ones that are specific to depth space checking; 
 * they should probably go into derived classes or something
 */
/*
std::vector<cv::Mat> GenAndTest::generateDepthImages(const std::vector<traj_func_ptr>& trajectory_functions, const state_type& x0, const std_msgs::Header& header)
{
    
    ROS_DEBUG_STREAM_NAMED(name_, "Generating Depth Images");
    
    size_t num_paths = trajectory_functions.size();
    
    std::vector<cv::Mat> trajectoryImages(num_paths);
    
    //Start timer
    auto t1 = std::chrono::high_resolution_clock::now();


    //Perform trajectory generation and collision detection in parallel if enabled
    //Vectors and arrays must be accessed by indicies to ensure thread safe behavior
    //#pragma omp parallel for schedule(dynamic) if(parallelism_enabled_) //schedule(dynamic)
    for(size_t i = 0; i < num_paths; i++)
    {
        pips_trajectory_ptr traj = std::make_shared<PipsTrajectory>();
        traj->header = header;
        traj->params = params_;

        traj->trajpntr = trajectory_functions[i];
        traj->x0_ = x0;

        traj_gen_bridge_->generate_trajectory(traj);

        cv::Mat image = generateTrajectoryDepthImage(traj);

        trajectoryImages[i] = image;
    }


    //End timer
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;


    ROS_DEBUG_STREAM_NAMED(name_, "Generated depth images for " << num_paths << " trajectories in " << fp_ms.count() << " ms");


    return trajectoryImages;
  }
  
  
  
  cv::Mat GenAndTest::generateTrajectoryDepthImage(const pips_trajectory_ptr& traj)
  {

      cv::Mat result;
      for(size_t i = 0; i < traj->num_states(); i++)
      {
        geometry_msgs::Point pt = traj->getPoint(i);    

        double coords[3];
        coords[0] = pt.x;
        coords[1] = pt.y;
        coords[2] = pt.z;

        cv::Mat newIm = cc_->generateDepthImage(coords);
        
        {
          double min;
          double max;
          cv::minMaxIdx(result, &min, &max);
          cv::Mat adjMap;
          cv::convertScaleAbs(newIm, adjMap, 255 / (max-min), -min);
          
          
          cv::imshow("image", adjMap);
          cv::waitKey(0);
        }
        
        if(result.empty())
        {
            result =  newIm;
        }
        else
        {
            result = cv::max(result, newIm);
        }
        
        {
            double min;
            double max;
            cv::minMaxIdx(result, &min, &max);
            cv::Mat adjMap;
            cv::convertScaleAbs(result, adjMap, 255 / (max-min), -min);

            cv::imshow("composite image", adjMap);
            cv::waitKey(0);
        }


    }

    return result;

}
*/

/*
//
void GenAndTest::saveCollisionCheckData(std::vector<traj_func_ptr>& trajectory_functions)
{
    state_type x0 = traj_gen_bridge_->initState();

    save_check_data_.open ("collision_check.txt", std::fstream::out | std::fstream::app);
    //    save_check_data_ << "Writing this to a file.\n";
    size_t num_paths = trajectory_functions.size();
    //Perform trajectory generation and collision detection in parallel if enabled
    //Vectors and arrays must be accessed by indicies to ensure thread safe behavior
    //#pragma omp parallel for schedule(dynamic) if(parallelism_enabled_) //schedule(dynamic)
    for(size_t i = 0; i < num_paths; i++)
    {
        pips_trajectory_ptr traj = std::make_shared<PipsTrajectory>();
        traj->header = header_;
        traj->params = params_;

        traj->trajpntr = trajectory_functions[i];
        traj->x0_ = x0;

        traj_gen_bridge_->generate_trajectory(traj);

        double coords[3];
        double pixels[2];
        geometry_msgs::Point pt;
        int collision_idx;
        traj->get_collision_ind(collision_idx);
        // for non-collision check points
        for(size_t j = 0; j < collision_idx; j++)
        {
            pt = traj->getPoint(j);
            coords[0] = pt.x;
            coords[1] = pt.y;
            coords[2] = pt.z;

            cc_->generateImageCoord(coords, pixels);
            save_check_data_ << pixels[0] << " " << pixels[1] << " " << 0 << std::endl;
        }
        // for collision point
        pt = traj->getPoint(collision_idx);
        coords[0] = pt.x;
        coords[1] = pt.y;
        coords[2] = pt.z;

        cc_->generateImageCoord(coords, pixels);
        save_check_data_ << pixels[0] << " " << pixels[1] << " " << 1 << std::endl;
    }

    save_check_data_.close();
}

*/


bool PipsTrajectory::collides()
{
    return collision_ind_ >=0;
}

ros::Time PipsTrajectory::time_of_collision()
{
    //ROS_WARN_STREAM(  //warn/close if no collision
    return header.stamp + ros::Duration(times[collision_ind_]);
}

void PipsTrajectory::set_collision_ind(int ind)
{
    collision_ind_ = ind;
}

geometry_msgs::PointStamped PipsTrajectory::get_collision_point()
{
    //ROS_WARN_STREAM(  //warn/close if no collision
    return ni_trajectory::getPointStamped(collision_ind_);
}

size_t PipsTrajectory::num_states()
{
    if(PipsTrajectory::collides())
    {
        return collision_ind_;
    }
    else
    {
        return ni_trajectory::num_states();
    }
}

//
void PipsTrajectory::get_collision_ind(int & ind)
{
    ind = collision_ind_;
}

geometry_msgs::PointStamped PipsTrajectory::get_check_point(const int ind)
{
    //ROS_WARN_STREAM(  //warn/close if no collision
    return ni_trajectory::getPointStamped(ind);
}
