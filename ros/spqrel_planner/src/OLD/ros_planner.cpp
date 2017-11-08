#include "ros_planner.h"
#include "srrg_ros_wrappers/ros_utils.h"
#include <cmath>
#include <fstream>
#if defined(_WIN32)
#include <Windows.h>

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
#include <unistd.h>
#include <sys/resource.h>
#include <sys/times.h>
#include <time.h>

#else
#error "Unable to define getCPUTime( ) for an unknown OS."
#endif


#define DEBUG_MAP_MESSAGE 0


/**
 * Returns the amount of CPU time used by the current process,
 * in seconds, or -1.0 if an error occurred.
 */
double getCPUTime( )
{
#if defined(_WIN32)
    /* Windows -------------------------------------------------- */
    FILETIME createTime;
    FILETIME exitTime;
    FILETIME kernelTime;
    FILETIME userTime;
    if ( GetProcessTimes( GetCurrentProcess( ),
                          &createTime, &exitTime, &kernelTime, &userTime ) != -1 )
    {
        SYSTEMTIME userSystemTime;
        if ( FileTimeToSystemTime( &userTime, &userSystemTime ) != -1 )
            return (double)userSystemTime.wHour * 3600.0 +
                    (double)userSystemTime.wMinute * 60.0 +
                    (double)userSystemTime.wSecond +
                    (double)userSystemTime.wMilliseconds / 1000.0;
    }

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
    /* AIX, BSD, Cygwin, HP-UX, Linux, OSX, and Solaris --------- */

#if defined(_POSIX_TIMERS) && (_POSIX_TIMERS > 0)
    /* Prefer high-res POSIX timers, when available. */
    {
        clockid_t id;
        struct timespec ts;
#if _POSIX_CPUTIME > 0
        /* Clock ids vary by OS.  Query the id, if possible. */
        if ( clock_getcpuclockid( 0, &id ) == -1 )
#endif
#if defined(CLOCK_PROCESS_CPUTIME_ID)
            /* Use known clock id for AIX, Linux, or Solaris. */
            id = CLOCK_PROCESS_CPUTIME_ID;
#elif defined(CLOCK_VIRTUAL)
            /* Use known clock id for BSD or HP-UX. */
            id = CLOCK_VIRTUAL;
#else
            id = (clockid_t)-1;
#endif
        if ( id != (clockid_t)-1 && clock_gettime( id, &ts ) != -1 )
            return (double)ts.tv_sec +
                    (double)ts.tv_nsec / 1000000000.0;
    }
#endif

#if defined(RUSAGE_SELF)
    {
        struct rusage rusage;
        if ( getrusage( RUSAGE_SELF, &rusage ) != -1 )
            return (double)rusage.ru_utime.tv_sec +
                    (double)rusage.ru_utime.tv_usec / 1000000.0;
    }
#endif

#if defined(_SC_CLK_TCK)
    {
        const double ticks = (double)sysconf( _SC_CLK_TCK );
        struct tms tms;
        if ( times( &tms ) != (clock_t)-1 )
            return (double)tms.tms_utime / ticks;
    }
#endif

#if defined(CLOCKS_PER_SEC)
    {
        clock_t cl = clock( );
        if ( cl != (clock_t)-1 )
            return (double)cl / (double)CLOCKS_PER_SEC;
    }
#endif

#endif

    return -1;		/* Failed. */
}




namespace spqrel_navigation {

using namespace std;
using namespace srrg_core;
using namespace srrg_core_ros;


// ROS utils

//! returns the time in milliseconds
double getTime_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return 1e3*tv.tv_sec+1e-3*tv.tv_usec;
}

Eigen::Vector3f convertPose(const tf::StampedTransform& t) {
    double yaw,pitch,roll;
    tf::Matrix3x3 mat =  t.getBasis();
    mat.getRPY(roll, pitch, yaw);
    return Eigen::Vector3f(t.getOrigin().x(), t.getOrigin().y(), yaw);
}

double getYaw(tf::Pose& t) {
    double yaw, pitch, roll;
    t.getBasis().getEulerYPR(yaw,pitch,roll);
    return yaw;
}



ROSPlanner::ROSPlanner(ros::NodeHandle& nh, tf::TransformListener* listener):
    _private_nh("~"),
    _nh(nh),_as(_nh, "move_base", boost::bind(&ROSPlanner::executeCB, this, _1), false),
    _action_name("move_base"){
    _listener = listener;
    if (! _listener)
        _listener = new tf::TransformListener;
    _global_frame_id = "/map";
    _base_frame_id = "base_footprint_frame";
    _command_vel_topic = "cmd_vel";
    _map_topic = "map";
    _temp_topic = "next_pose";
    _path_topic = "path";
    _global_path_topic = "global_path";
    _nearest_object_topic = "nearest_object_distance";

    _forced_max_range = 10;
    _squared_endpoint_distance = 0.1*0.1;
    _show_distance_map = false;
    _force_redisplay=false;
    _set_goal = false;
    _use_gui=false;
    _map_origin.setZero();
    _timers.resize(10);
    _last_timer_slot=0;
    _wait_cicle=100;
    _wait=0;
    prev_tv=0;
    prev_rv=0;
    _have_map = false;
    _new_map_available = false;

    _as.registerPreemptCallback(boost::bind(&ROSPlanner::preemptCB, this) );
    _as.start();
}


void ROSPlanner::setROSParams() {

#if 0
    //---------------------------------------------------------------------------------------------
    cerr << endl << "Gradient Controller Parameters: " << endl;
    //---------------------------------------------------------------------------------------------

    double Ktv;
    _private_nh.param("Ktv",Ktv,1.0);
    cerr << "gradient_controller: [float] _Ktv: " << Ktv << endl;

    double Krv;
    _private_nh.param("Krv",Krv,1.0);
    cerr << "gradient_controller: [float] _Krv: " << Krv << endl;
    _gradient_controller->setAttractionParameters(Ktv,Krv);

    double repulsive_t_scale;
    _private_nh.param("repulsive_t_scale",repulsive_t_scale,0.0);
    cerr << "gradient_controller: [float] _repulsive_t_scale: " << repulsive_t_scale << endl;

    double repulsive_r_scale;
    _private_nh.param("repulsive_r_scale",repulsive_r_scale,0.0);
    cerr << "gradient_controller: [float] _repulsive_r_scale: " << repulsive_r_scale << endl;
    _gradient_controller->setRepulsionParameters(repulsive_t_scale,repulsive_r_scale);

    double max_tv;
    _private_nh.param("max_tv", max_tv, 1.0);
    cerr << "gradient_controller: [float] _max_tv: " << max_tv << endl;

    double max_rv;
    _private_nh.param("max_rv", max_rv, 0.5);
    cerr << "gradient_controller: [float] _max_rv: " << max_rv << endl;
    _gradient_controller->setMaxVelocity(max_tv,max_rv);

    double max_t_acc;
    _private_nh.param("max_t_acc", max_t_acc, 0.1);
    cerr << "gradient_controller: [float] _max_t_acc: " << max_t_acc << endl;

    double max_r_acc;
    _private_nh.param("max_r_acc", max_r_acc, 0.3);
    cerr << "gradient_controller: [float] _max_r_acc: " << max_r_acc << endl;
    _gradient_controller->setMaxAccelerations(max_t_acc,max_r_acc);

    double min_attractor_distance;
    _private_nh.param("min_attractor_distance",min_attractor_distance,0.16);
    cerr << "gradient_controller: [float] _min_attractor_distance: " << min_attractor_distance << endl;

    double max_attractor_distance;
    _private_nh.param("max_attractor_distance",max_attractor_distance,5.3);
    cerr << "gradient_controller: [float] _max_attractor_distance: " << max_attractor_distance << endl;
    _gradient_controller->setAttractorDistances(min_attractor_distance,max_attractor_distance);

    double robot_radius;
    _private_nh.param("robot_radius", robot_radius, 0.3);
    cerr << "gradient_controller: [float] _robot_radius: " << robot_radius << endl;
    _gradient_controller->setRobotRadius(robot_radius);

    double safety_threshold;
    _private_nh.param("safety_threshold",safety_threshold,0.3);
    cerr << "gradient_controller: [float] _safety_threshold: " << safety_threshold << endl;
    _gradient_controller->setSafePathThreshold(safety_threshold);
    //---------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------
    cerr << endl << "Grid Planner Parameters: " << endl;
    //---------------------------------------------------------------------------------------------

    double distance_cost_factor;
    _private_nh.param("distance_cost_factor", distance_cost_factor, .5);
    cerr << "grid_planner: [float] _distance_cost_factor: " << distance_cost_factor << endl;
    setDistanceCostFactor(distance_cost_factor);

    double distance_threshold;
    _private_nh.param("distance_threshold", distance_threshold, 1.0);
    cerr << "grid_planner: [float] _distance_threshold: " << distance_threshold << endl;
    setDistanceThreshold(distance_threshold);

    double local_map_dimension;
    _private_nh.param("local_map_dimension", local_map_dimension, 6.0);
    cerr << "grid_planner: [float] _local_map_dimension: " << local_map_dimension << endl;
    setLocalMapDimension(local_map_dimension);

    int persistency;
    _private_nh.param("persistency",persistency,300);
    cerr << "grid_planner: [int] _persistency: " << persistency << endl;
    setPersistency(persistency);

    double goal_tolerance_t;
    _private_nh.param("goal_tolerance_t",goal_tolerance_t,0.5);
    cerr << "grid_planner: [float] _goal_tolerance_t: " << goal_tolerance_t << endl;
    setGoalToleranceT(goal_tolerance_t);

    double goal_tolerance_r;
    _private_nh.param("goal_tolerance_r",goal_tolerance_r,0.5);
    cerr << "grid_planner: [float] _goal_tolerance_r: " << goal_tolerance_r << endl;
    setGoalToleranceR(goal_tolerance_r);

    cerr << "grid planner: [float] _robot_radius: " << robot_radius << endl;
    setRobotRadius(robot_radius);
    //---------------------------------------------------------------------------------------------
#endif

    //---------------------------------------------------------------------------------------------
    cerr << endl << "ROS Planner Parameters: " << endl;
    //---------------------------------------------------------------------------------------------

    double forced_max_range;
    _private_nh.param("forced_max_range", forced_max_range, 5.0);
    cerr << "spqrel_planner: [float] _forced_max_range: " << forced_max_range << endl;
    setForcedMaxRange(forced_max_range);

    double squared_endpoint_distance;
    _private_nh.param("squared_endpoint_distance",squared_endpoint_distance,0.01);
    cerr << "spqrel_planner: [float] _squared_endpoint_distance: " << squared_endpoint_distance << endl;
    setSquaredEndpointDistance(squared_endpoint_distance);

    int wait_cicle;
    _private_nh.param("wait_cicle",wait_cicle,100);
    cerr << "spqrel_planner: [int] _wait_cycle: " << wait_cicle << endl;
    setWaitCicle(wait_cicle);


    _private_nh.param("use_gui", _use_gui, false);
    cerr << "spqrel_planner: [bool] _use_gui: " << _use_gui << endl;


    //_command_vel_topic
    std::string command_vel_topic;
    _private_nh.param("command_vel_topic", command_vel_topic, std::string("cmd_vel"));
    setCommandVelTopic(command_vel_topic);
    cerr << "spqrel_planner: [string] _command_vel_topic: " << command_vel_topic << endl;

    //_global_frame_id
    std::string global_frame_id;
    _private_nh.param("global_frame_id", global_frame_id, std::string("/map"));
    setGlobalFrameId(global_frame_id);
    cerr << "spqrel_planner: [string] _global_frame_id: " << global_frame_id << endl;

    //_base_frame_id
    std::string base_frame_id;
    _private_nh.param("base_frame_id", base_frame_id, std::string("base_link"));
    setBaseFrameId(base_frame_id);
    cerr << "spqrel_planner: [string] _base_frame_id: " << base_frame_id << endl;

    _private_nh.param("laser_topic", _laser_topic, std::string("base_scan"));
    cerr << "spqrel_planner: [string] _laser_topic: " << _laser_topic << endl;


    bool publish_global_plan;
    _private_nh.param("publish_global_plan", publish_global_plan, false);
    cerr << "spqrel_planner: [float] _publish_global_plan: " << publish_global_plan << endl;
    // setCompute_global_path(publish_global_plan);

}




void ROSPlanner::executeCB(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal){
    geometry_msgs::PoseStamped ps = move_base_goal->target_pose;

    geometry_msgs::PoseStampedConstPtr  ps_ptr (new geometry_msgs::PoseStamped(ps));
    
    setGoalCallback(ps_ptr);     
    move_base_msgs::MoveBaseFeedback feed;
    ros::Rate r(10); // 10 Hz
    r.sleep();
    while(!_as.isPreemptRequested() && _planner.haveGoal() && ros::ok()){
        r.sleep();
        _as.publishFeedback(feed);
    }


    string action_result = _planner._result;

    if(_planner._result=="success") {
        _as.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
    } else if (_as.isPreemptRequested()) {
        action_result = "preempted";
        _as.setAborted(move_base_msgs::MoveBaseResult(), "External preemption.");
    } else {
        _as.setAborted(move_base_msgs::MoveBaseResult(), "Internal event: "+action_result);
    }

    ROS_INFO("Action finished with result: %s",action_result.c_str());

    stopRobot();
}

void ROSPlanner::stopRobot() {
    geometry_msgs::Twist req_twist;
    req_twist.linear.x = 0;
    req_twist.angular.z = 0;
    _cmd_vel_pub.publish(req_twist);
    ros::spinOnce();
}

void ROSPlanner::setCancelCallback(const actionlib_msgs::GoalID& msg) {
    ROS_INFO("Goal cancelled!!!");
    _planner.cancelGoal();
    stopRobot();
}

void ROSPlanner::preemptCB() {
    ROS_INFO("Goal Preempted!!!");
    _planner.cancelGoal();
    stopRobot();
}

void ROSPlanner::init() {

    // subscribe to laser topic (main execution thread for the planner)
    subscribeCallbacks(_laser_topic);

    initEmptyMap();

    if (_use_gui)
        _planner.initGUI();

    _planner.subscribeServices();
}


void ROSPlanner::quit() {
    //unsubscribeCallbacks(_laser_topic);
    _planner.unsubscribeServices();
}


void ROSPlanner::subscribeCallbacks(const std::string& laser_topic){
     std::cerr << "ROSPlanner subscribers to: " << _laser_topic << " " <<
        _map_topic << " " << "move_base_simple/goal" << " " << "move_base/cancel" <<
        std::endl;

    _laser_topic=laser_topic;
    _laser_sub=_nh.subscribe(_laser_topic, 1, &ROSPlanner::laserCallback, this);
    _goal_simple_sub = _nh.subscribe("move_base_simple/goal", 2, &ROSPlanner::setGoalCallback, this);
    _cancel_sub = _nh.subscribe("move_base/cancel", 2, &ROSPlanner::setCancelCallback, this);
    _map_sub = _nh.subscribe(_map_topic, 1, &ROSPlanner::mapMessageCallback, this);

     std::cerr << "ROSPlanner publishers: " << _command_vel_topic << " " <<
        _nearest_object_topic << " " << _temp_topic << " " << _path_topic <<
        std::endl;


    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(_command_vel_topic, 1);
    _nearest_pub = _nh.advertise<std_msgs::Float32>(_nearest_object_topic,-1);
    _temp_pub = _nh.advertise<nav_msgs::OccupancyGrid>(_temp_topic, 1);
    _path_pub= _nh.advertise<nav_msgs::Path>(_path_topic,1);

    //if(_compute_global_path)
    //    _global_path_pub= _nh.advertise<nav_msgs::Path>(_global_path_topic,1);

    //_function = boost::bind(&spqrel_navigation::ROSPlanner::dynamicReconfigureCallback, this , _1, _2);
    //_server.setCallback(_function);
}



void ROSPlanner::rangesToEndpoints(Vector2fVector& endpoints,
                                   const Eigen::Vector3f& laser_pose,
                                   const sensor_msgs::LaserScan::ConstPtr& msg){

    // we have the transforms, we can start assembling the endpoints for the localizer
    // in doing that we do take care that no endpoint is closer than
    // squared_endpoint_distance from its predecessor
    // this avoids unnecessary computation when in crowded settings
    Eigen::Isometry2f laser_transform=v2t(laser_pose);
    endpoints.resize(msg->ranges.size());
    int k = 0;
    double angle=msg->angle_min-msg->angle_increment;
    float max_range = (msg->range_max<_forced_max_range) ? msg->range_max : _forced_max_range;
    Eigen::Vector2f last_endpoint(-1000, -1000);
    for (size_t i=0; i<msg->ranges.size(); i++){
        float r=msg->ranges[i];
        angle+=msg->angle_increment;
        if (r<msg->range_min)
            continue;
        if (r>=max_range)
            continue;
        Eigen::Vector2f dir(cos(angle), sin(angle));
        Eigen::Vector2f ep=laser_transform*(dir*r);
        Eigen::Vector2f delta = last_endpoint-ep;
        if (delta.squaredNorm()>_squared_endpoint_distance) {
            endpoints[k]=ep;
            last_endpoint = ep;
            k++;
        }
    }
    endpoints.resize(k);
}


void ROSPlanner::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

#if DEBUG_MAP_MESSAGE
    cerr << "laserCallback..." << endl;
#endif

    if (!_have_map) return;

    if (!_planner.haveGoal() && _new_map_available) {
        requestMap();
    }


    _last_observation_time = msg->header.stamp;

    std::string error;

    // laser pose on robot
    if (! _listener->waitForTransform (_base_frame_id, 
				       msg->header.frame_id, 
				       _last_observation_time, 
				       ros::Duration(0.5), 
				       ros::Duration(0.5), &error)) {
      cerr << "ROSPlanner: transform error from " << _base_frame_id << " to " << msg->header.frame_id << " : " << error << endl;
      return;
    }
    tf::StampedTransform laser_pose_t;
    _listener->lookupTransform(_base_frame_id, 
			       msg->header.frame_id, 
			       _last_observation_time, 
			       laser_pose_t);

    Eigen::Vector3f laser_pose = convertPose2D(laser_pose_t);
   
    if (! _listener->waitForTransform (_base_frame_id, 
				       msg->header.frame_id, 
				       _last_observation_time, 
				       ros::Duration(0.5), 
				       ros::Duration(0.5), 
				       &error)) {
      cerr << "error: " << error << endl;
      return;
    }


    // global pose
    if (! _listener->waitForTransform (_global_frame_id,
                                       _base_frame_id,
                                       _last_observation_time,
                                       ros::Duration(0.5),
                                       ros::Duration(0.5),
                                       &error)) {
        cerr << "ROSPlanner: transform error from " << _global_frame_id << " to " << _base_frame_id << " : " << error << endl;
        return;
    }

    tf::StampedTransform robot_pose_t;
    _listener->lookupTransform(_global_frame_id,
                               _base_frame_id,
                               _last_observation_time,
                               robot_pose_t);
    Eigen::Isometry2f inverse_map_origin_transform=v2t(_map_origin).inverse();
    //_robot_pose = t2v( inverse_map_origin_transform * v2t(convertPose(robot_pose_t)));

    Eigen::Vector3f robot_pose = t2v( inverse_map_origin_transform * v2t(convertPose2D(robot_pose_t)));

    _planner.setRobotPose(robot_pose);


    Vector2fVector endpoints(msg->ranges.size());
    rangesToEndpoints(endpoints, laser_pose, msg);
    
    //updateTemporaryMap(endpoints);
    //_nearest_pub.publish(_nearest_dynamic_object);

    _planner.setLaserPoints(endpoints);


#if DEBUG_MAP_MESSAGE
    cerr << "laserCallback: before plannerStep" << endl;
    cerr << "+++ mutex lock - laserCallback/plannerStep" << endl;
#endif
    _mtx_goal.lock();

    _planner.plannerStep();

    _mtx_goal.unlock();
#if DEBUG_MAP_MESSAGE
    cerr << "--- mutex unlock - laserCallback/plannerStep" << endl;
#endif

    geometry_msgs::Twist req_twist;
    req_twist.linear.x = _planner._linear_vel;
    req_twist.angular.z = _planner._angular_vel;
    _cmd_vel_pub.publish(req_twist);
    ros::spinOnce();

}



#if 0
void ROSPlanner::dynamicReconfigureCallback(thin_navigation::ThinNavigationConfig &config, uint32_t level){

    ROS_INFO("Reconfigure Request!");

    _gradient_controller->setAttractionParameters(config.Ktv,config.Krv);
    _gradient_controller->setRepulsionParameters(config.repulsive_t_scale,config.repulsive_r_scale);
    _gradient_controller->setAttractorDistances(config.min_attractor_distance,config.max_attractor_distance);

    setPersistency(config.persistency);
    setGoalToleranceT(config.goal_tolerance_t);
    setGoalToleranceR(config.goal_tolerance_r);

    setWaitCicle(config.wait_cicle);

}
#endif



void ROSPlanner::requestMap() {
    // get map via RPC
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO_STREAM("Requesting the map on service " << _map_service_id << " ... ");
    int ntry = 5;
    while(!ros::service::call(_map_service_id, req, resp) && ros::ok() && ntry-->0){
        ROS_WARN("Request for map to service %s failed; trying again...",_map_service_id.c_str());
        ros::Duration d(1.0);
        ros::spinOnce(); d.sleep();
        ros::spinOnce(); d.sleep();
        ros::spinOnce(); d.sleep();
    }
    if (ntry>0) {
        ROS_INFO("Map received from service.");
        mapMessageCallback(resp.map);
        return;
    }

}


void ROSPlanner::mapMessageCallback(const::nav_msgs::OccupancyGrid& msg) {

#if DEBUG_MAP_MESSAGE
    cerr << "map message callback" << endl;
    cerr << "+++ mutex lock - map callback" << endl;
#endif
    _mtx_goal.lock();

    if (_have_map && _planner.haveGoal()) { 
        _new_map_available = true;
        _mtx_goal.unlock();
#if DEBUG_MAP_MESSAGE
        cerr << "--- mutex unlock (1) - map callback --- message dropped ---" << endl;
#endif
        return; 
    }


    //ROS_INFO("Map info: WIDTH: %d, HEIGHT: %d, RESOLUTION: %f",
    //            msg.info.width,msg.info.height,msg.info.resolution);

    UnsignedCharImage map_image(msg.info.width, msg.info.height);  // cols, rows
    int k=0;

    for(int c=0; c<map_image.cols; c++) {

        for(int r=0; r<map_image.rows; r++) {

            int d=msg.data[k];
            if (d<0) { // unknown
                d=127;
            }
            else if(d>=50)
                d=0;
            else
                d=255;
            // int c1 = r, r1 = msg.info.height-c;
            map_image.at<unsigned char>(r,c)=(unsigned char)(d);
            k++;
        }
    }

    cv::transpose(map_image, map_image);
    cv::flip(map_image, map_image, 0);

    tf::Pose pose;
    tf::poseMsgToTF(msg.info.origin, pose);
    Eigen::Vector3f map_origin(pose.getOrigin().x(),
                               pose.getOrigin().y(),
                               getYaw(pose));
    // cerr << "map origin: " << map_origin.transpose() << endl;

    _planner.setMapFromImage(map_image,msg.info.resolution,map_origin, 0.65, 0.05);
    _planner.reset();
    _have_map = true;
    _new_map_available = false;

    _mtx_goal.unlock();
#if DEBUG_MAP_MESSAGE
    cerr << "--- mutex unlock (2) - map callback !!! OK map updated !!!" << endl;
#endif

}



void ROSPlanner::initEmptyMap() {

    double width = 10.0, height = 10.0, resolution = 0.05;

    ROS_INFO("Init empty map info: WIDTH: %f, HEIGHT: %f, RESOLUTION: %f",
                width,height,resolution);

    UnsignedCharImage map_image(width, height);  // cols, rows
    int k=0;

    for(int c=0; c<map_image.cols; c++) {
        for(int r=0; r<map_image.rows; r++) {
            int d=255; // free space
            map_image.at<unsigned char>(r,c)=(unsigned char)(d);
        }
    }

    Eigen::Vector3f map_origin(0.0, 0.0, 0.0);
    cerr << "map origin: " << map_origin.transpose() << endl;

    _planner.setMapFromImage(map_image,resolution,map_origin, 0.65, 0.05);

}


void ROSPlanner::setGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

#if DEBUG_MAP_MESSAGE
    cerr << "set goal callback" << endl;
    cerr << "+++ mutex lock - set goal" << endl;
#endif
    _mtx_goal.lock();

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose, pose);
    Eigen::Vector3f new_pose(pose.getOrigin().x(),
                             pose.getOrigin().y(),
                             getYaw(pose));

    ROS_INFO("Setting goal (%.6f): %.3f %.3f %.3f",
             ros::Time::now().toSec(),
             new_pose.x(),
             new_pose.y(),
             new_pose.z());
    Eigen::Isometry2f inverse_origin=v2t(_map_origin).inverse();
    Eigen::Isometry2f global_pose=v2t(new_pose);
    Eigen::Vector3f map_pose=t2v(inverse_origin*global_pose);

    _planner.setGoal(map_pose);


    _mtx_goal.unlock();
#if DEBUG_MAP_MESSAGE
    cerr << "--- mutex unlock - set goal" << endl;
    cerr << "Setting goal terminated." << endl;
#endif
}



double ROSPlanner::cycleLatency() const {
    double acc=0;
    for (size_t i=0; i<_timers.size(); i++)
        acc+=_timers[i];
    return acc/_timers.size();
}

Eigen::Vector3d t2v(Eigen::Matrix3d A)
{
    Eigen::Vector3d t;
    t.block(0,0,2,1) = A.block(0,2,2,1);
    t(2) = atan2(A(1,0),A(0,0));
    return t;
}

Eigen::Matrix3d v2t(Eigen::Vector3d v)
{
    Eigen::Matrix3d A;
    A << cos(v(2)), -sin(v(2)), v(0),
            sin(v(2)), cos(v(2)), v(1),
            0,0,1;
    return A;

}
Eigen::Vector3d compute_omega(Eigen::Matrix3d T, double gamma, double kappa, double h)
{
    Eigen::Vector3d t = t2v(T);
    Eigen::Vector2d rr = Eigen::Vector2d(t(0),t(1));
    double r = rr.norm();
    float theta = -(M_PI - atan2(t(1),t(0)));
    theta = atan2(sin(theta), cos(theta));
    float delta = theta - t(2);
    delta = atan2(sin(delta), cos(delta));
    double v = gamma*cos(delta)*r;
    double sin_delta_over_delta=1;
    if (fabs(delta)>std::numeric_limits<double>::min()){
        sin_delta_over_delta=sin(delta)/delta;
    }
    double w = kappa*delta+(gamma*cos(delta)*sin_delta_over_delta)*(delta+h*theta);

    return Eigen::Vector3d(w,r,v);
}




// OLD STUFF


void ROSPlanner::executePath(){
#if 0
    geometry_msgs::Twist req_twist;
    if (_plan_found!="GO" && _plan_found!="NEAR" && _plan_found!="ROTATION"){

        req_twist.linear.x = 0;
        req_twist.angular.z = 0;
        _cmd_vel_pub.publish(req_twist);

        return;
    }
    nav_msgs::Path path;
    path.header.frame_id=_base_frame_id;
    path.header.stamp=ros::Time::now();
    Eigen::Isometry2f inverse_origin=v2t(_robot_pose).inverse();
    int i=0;
    while(i<_local_plan.size()){
        Eigen::Vector3f new_pose(_local_plan[i].x(),
                                 _local_plan[i].y(),
                                 _local_plan[i].z());

        Eigen::Isometry2f global_pose=v2t(new_pose);
        Eigen::Vector3f map_pose=t2v(inverse_origin*global_pose);
        geometry_msgs::PoseStamped p;
        p.pose.position.x=map_pose.x();
        p.pose.position.y=map_pose.y();
        p.pose.orientation=tf::createQuaternionMsgFromYaw(map_pose.z());
        path.poses.push_back(p);
        i++;
    }
    _path_pub.publish(path);
    if(_compute_global_path){
        nav_msgs::Path global_path;
        global_path.header.frame_id=_base_frame_id;
        global_path.header.stamp=ros::Time::now();
        int i=0;
        while(i<_global_plan.size()){
            Eigen::Vector3f new_pose(_global_plan[i].x(),
                                     _global_plan[i].y(),
                                     _global_plan[i].z());
            Eigen::Isometry2f global_pose=v2t(new_pose);
            Eigen::Vector3f map_pose=t2v(inverse_origin*global_pose);
            geometry_msgs::PoseStamped p;
            p.pose.position.x=map_pose.x();
            p.pose.position.y=map_pose.y();
            p.pose.orientation=tf::createQuaternionMsgFromYaw(map_pose.z());
            global_path.poses.push_back(p);
            i++;
        }
        _global_path_pub.publish(global_path);
    }
    
    //     // eat the plan until a maximum distance (translational or rotational) is reached;
    //     Eigen::Isometry2f robot_inverse_transform=v2t(_robot_pose).inverse();
    //
    //     Eigen::Vector3f delta;
    //     float _trans_thresh=0.3;//0.3*0.3;
    //     float _rot_thresh=.3*M_PI;
    //     float distance=0;
    //     float min_distance=0.4;
    //     float max_distance=1.0;
    //     float tresh_multip=1;
    //     //ROS_INFO("distance: %f - %f",_plan[0].w(),_plan[0].w()-_robot_radius);
    //     i=0;
    //     while(i<_local_plan.size()){
    //         //ROS_INFO("distance: %f",_plan[i].w());
    //         Eigen::Isometry2f plan_transform=v2t(Eigen::Vector3f(_local_plan[i].x(),_local_plan[i].y(),_local_plan[i].z()));
    //         delta=t2v(robot_inverse_transform*plan_transform);
    //
    //         distance =_local_plan[i].w();
    //         //        if(distance<min_distance){
    //         //            tresh_multip=min_distance;
    //         //        }else if(distance>max_distance){
    //         //            tresh_multip=max_distance;
    //         //        }
    //         //        else{
    //         //            tresh_multip=distance;
    //         //        }
    //         tresh_multip=1;
    //         if (delta.head<2>().squaredNorm()>(_trans_thresh*tresh_multip)){
    //             //cout << delta.head<2>().squaredNorm() << " : "<<(_trans_thresh*distance)<<endl;
    //             break;}
    //        // if (fabs(delta.z())>(_rot_thresh*tresh_multip)){
    //             //cout <<     fabs(delta.z()) << " : " <<    (_rot_thresh*distance)<<endl;
    //          //   break;
    //         //}
    //         //        if (i>=1 && fabs(_local_plan[i].z()-_local_plan[i-1].z())>(_rot_thresh*tresh_multip)){
    //         //            break;
    //         //        }
    //         i++;
    //     }
    //         _next_point=delta;

    //    Eigen::Isometry2f local_pose_transform=v2t(_local_pose);
    //    Eigen::Isometry2f local_goal_transform=v2t(delta);
    //    Eigen::Vector3f local_attractor=t2v(local_pose_transform*local_goal_transform);
    //    int r=local_attractor.x()*_global.inverse_resolution;
    //    int c=local_attractor.y()*_global.inverse_resolution;
    //    _next_point=local_attractor;


    //    float delta_t = _last_observation_time.toSec()-_prev_controller_time.toSec();
    //    float tv=0, rv=0;
    //    float temp_max_tv=_max_tv;
    //    float max_dyn_dist=2;
    //    if(_nearest_dynamic_object>=0 && _nearest_dynamic_object<max_dyn_dist){
    //        temp_max_tv=temp_max_tv*(_nearest_dynamic_object/max_dyn_dist);
    //        //_plan_found="NEAR";
    //    }
    //    PRINT_DEBUG("PLAN FOUND= "<<_plan_found<<endl);
    //    if(_plan_found == "GO"){

    //        /// BARTOLO THINGS
    //        double gamm_a=_translational_gain;
    //        double kappa=_rotational_gain;
    //        double h=.2;
    //        Eigen::Vector3d cose(delta(0),delta(1),delta(2));
    //        Eigen::Vector3d wrv = compute_omega(v2t(cose).inverse(),gamm_a,kappa,h);
    //        tv = wrv(2)*delta_t;
    //        rv = wrv(0)*delta_t;
    //        /// BARTOLO END
    //    } else if(_plan_found=="NEAR"){
    //        float min_curvature_radius=0.01;//0.01
    //        float max_curvature_radius=10;
    //        float squared_translation=delta.head<2>().squaredNorm();
    //        float translation=std::sqrt(squared_translation);
    //        float curvature_radius= 2 * max_curvature_radius;
    //        if (fabs(delta.y())>std::numeric_limits<float>::min()) { //HACK
    //            curvature_radius=.5*squared_translation/delta.y();
    //        }
    //        //cerr << "curvature_radius: " << curvature_radius << endl;
    //        if (fabs(curvature_radius)>max_curvature_radius ){
    //            //cerr << "pure translation" << endl;
    //            tv=translation*_approaching_translational_gain;
    //            // pure translation
    //        } else if (fabs(curvature_radius)<min_curvature_radius || delta.x()<0) {
    //            //cerr << "pure rotation" << endl;
    //            rv=delta.z();//*_approaching_rotational_gain;
    //        } else {
    //            //cerr << "mixed" << endl;
    //            float theta=atan2(delta.x(), curvature_radius-fabs(delta.y()));
    //            if (delta.y()<0)
    //                theta=-theta;
    //            //translation=fabs(curvature_radius*theta);
    //            tv=translation*_approaching_translational_gain;
    //            //cerr << "theoretical tv: " << tv << endl;
    //            //cerr << "theta: " << theta << endl;
    //            rv=_approaching_rotational_gain*theta;
    //        }
    //    }else if(_plan_found=="ROTATION"){
    //        PRINT_DEBUG("DELTA ANGLE: "<<delta.z()<<endl);
    //        tv=0;
    //        rv=delta.z();
    //    }

    //    //    float acc_tv=tv-prev_tv;

    //    //    if (fabs(acc_tv)>_max_acc_tv){
    //    //        tv=(acc_tv*0.02)+prev_tv;
    //    //    }

    //    //    float acc_rv=rv-prev_rv;

    //    //    if (fabs(acc_rv)>_max_acc_rv){
    //    //        rv=(acc_rv*0.02)+prev_rv;
    //    //    }

    //    if (fabs(rv)>_max_rv){
    //        float scale=_max_rv/fabs(rv);
    //        tv*=scale;
    //        rv*=scale;
    //    }

    //    if (fabs(tv)>temp_max_tv){
    //        float scale=temp_max_tv/fabs(tv);
    //        tv*=scale;
    //        rv*=scale;
    //    }


    //    //    tv=current_tv*(1-l)+l*tv;
    //    //    rv=current_rv*(1-l)+l*rv;

    //    _actual_tv=tv;
    //    _actual_rv=rv;
    
    //    req_twist.linear.x = tv;
    //    req_twist.linear.y = 0;
    //    req_twist.linear.z = 0;
    //    req_twist.angular.x = 0;
    //    req_twist.angular.y = 0;
    //    req_twist.angular.z = rv;
    Eigen::Vector2f vel, smoothed_vel, old_vel;
    old_vel(0)=prev_tv; old_vel(1)=prev_rv;
    
    if(_plan_found=="ROTATION"){

        //std::cerr << " " << std::endl;

        Eigen::Vector3f delta;
        Eigen::Isometry2f robot_inverse_transform=v2t(_robot_pose).inverse();
        Eigen::Isometry2f plan_transform=v2t(Eigen::Vector3f(_local_plan[_local_plan.size()-1].x(),
                                             _local_plan[_local_plan.size()-1].y(),
                _local_plan[_local_plan.size()-1].z()));
        delta=t2v(robot_inverse_transform*plan_transform);
        vel(0)=0;
        vel(1)=delta.z();
    }else{

        ///Marco stuff
        _gradient_controller->setImageResolution(_global.resolution);
        _gradient_controller->computeVelocity(_local.distances,_dynamic.distances, _robot_pose, _local_plan, vel);
        _gradient_controller->smoothVelocity(old_vel, vel, smoothed_vel);
        vel=smoothed_vel;

    }
    //     if (fabs(vel(1))>_max_rv){
    //             float scale=_max_rv/fabs(vel(1));
    //             vel(0)*=scale;
    //             vel(1)*=scale;
    //      }
    //
    //         if (fabs(vel(0))>_max_tv){
    //             float scale=_max_tv/fabs(vel(0));
    //             vel(0)*=scale;
    //             vel(1)*=scale;
    //          }
    req_twist.linear.x = vel.x();
    req_twist.linear.y = 0;
    req_twist.linear.z = 0;
    req_twist.angular.x = 0;
    req_twist.angular.y = 0;
    req_twist.angular.z = vel.y();
    
    _cmd_vel_pub.publish(req_twist);
    _prev_controller_time=_last_observation_time;
    //     prev_tv=tv;
    //     prev_rv=rv;
    prev_tv=req_twist.linear.x;
    prev_rv=req_twist.angular.z;
    //    current_tv=tv;
    //    current_rv=rv;
#endif
}
}
