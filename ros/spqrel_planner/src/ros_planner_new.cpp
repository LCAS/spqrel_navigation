#include "ros_planner_new.h"

namespace spqrel_navigation {

  using namespace srrg_planner;
  
  ROSPlanner::ROSPlanner(ros::NodeHandle& nh, tf::TransformListener* listener): _nh(nh){
    
    _listener = listener;
    if (! _listener)
      _listener = new tf::TransformListener;
      
    _laser_topic = "base_scan";
    _goal_topic = "move_base_simple/goal";
    _map_topic = "map";
    _cancel_topic = "move_base/cancel";
    _reset_topic = "reset";
    _cmd_vel_topic = "cmd_vel";
    _path_topic = "path";

    _forced_max_range = 10;
    _squared_endpoint_distance = 0.1*0.1;
      
    _base_frame_id = "base_footprint";
    _global_frame_id = "/map";

  }
  
  void ROSPlanner::stopRobot() {
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.angular.z = 0;
    _cmd_vel_pub.publish(vel);
    ros::spinOnce();
  }

  void ROSPlanner::applyVelocities(){
    std::cerr << "Applying vels: " << velocities().transpose() << std::endl;
    geometry_msgs::Twist vel;
    vel.linear.x = velocities().x();
    vel.angular.z = velocities().y();
    _cmd_vel_pub.publish(vel);
    ros::spinOnce();
  }

  void ROSPlanner::subscribeLaserWithPose(){
    std::cerr << "Subscribing to Laser: " << _laser_topic << std::endl;
    _laserwpose_sub= _nh.subscribe(_laser_topic, 1, &ROSPlanner::laserWithPoseCallback, this);
  }

  void ROSPlanner::subscribeGoal(){
    std::cerr << "Subscribing to Goal: " << _goal_topic << std::endl;
    _goal_sub = _nh.subscribe(_goal_topic, 2, &ROSPlanner::goalCallback, this);
  }

  void ROSPlanner::subscribeMap(){
    std::cerr << "Subscribing to Map: " << _map_topic << std::endl;
    _map_sub = _nh.subscribe(_map_topic, 1, &ROSPlanner::mapCallback, this);
  }

  void ROSPlanner::subscribeCancel(){
    std::cerr << "Subscribing to Cancel: " << _cancel_topic << std::endl;
    _cancel_sub = _nh.subscribe(_cancel_topic, 1, &ROSPlanner::cancelCallback, this);
  }

  void ROSPlanner::subscribeReset(){
    std::cerr << "Subscribing to Reset: " << _reset_topic << std::endl;
    _reset_sub = _nh.subscribe(_reset_topic, 1, &ROSPlanner::resetCallback, this);
  }



  void ROSPlanner::rangesToEndpoints(Vector2fVector& endpoints,
                                   const Eigen::Vector3f& laser_pose,
                                   const sensor_msgs::LaserScan::ConstPtr& msg){

    // we have the transforms, we can start assembling the endpoints for the planner
    // in doing that we do take care that no endpoint is closer than
    // squared_endpoint_distance from its predecessor
    // this avoids unnecessary computation when in crowded settings
    Eigen::Isometry2f laser_transform=v2t(laser_pose);
    endpoints.resize(msg->ranges.size());
    int k = 0;
    double angle=msg->angle_min-msg->angle_increment;
    float max_range = (msg->range_max < _forced_max_range) ? msg->range_max : _forced_max_range;
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
        if (delta.squaredNorm() > _squared_endpoint_distance) {
            endpoints[k]=ep;
            last_endpoint = ep;
            k++;
        }
    }
    endpoints.resize(k);
  }

  void ROSPlanner::laserWithPoseCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    //if (_state == WaitingForMap || _state == WaitingForGoal ) return;
    
    
    // Get laser pose on robot
    tf::StampedTransform laser_pose_tf;
    try {
      _listener->waitForTransform(_base_frame_id, msg->header.frame_id,
				  msg->header.stamp,
				  ros::Duration(0.5));
      _listener->lookupTransform(_base_frame_id, msg->header.frame_id,
				 msg->header.stamp,
				 laser_pose_tf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Laser pose transform: %s",ex.what());
    }
    Eigen::Vector3f laser_pose = convertPose2D(laser_pose_tf);
      
    // Get global pose
    tf::StampedTransform robot_pose_tf;
    try {
      _listener->waitForTransform(_global_frame_id, _base_frame_id,
				  msg->header.stamp,
				  ros::Duration(0.5));
      _listener->lookupTransform(_global_frame_id, _base_frame_id,
				 msg->header.stamp,
				 robot_pose_tf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Robot pose transform: %s",ex.what());
    }   
    Eigen::Vector3f robot_pose = convertPose2D(robot_pose_tf);

    // Pruning some points
    Vector2fVector laser_points;
    rangesToEndpoints(laser_points, laser_pose, msg);

    // Setting data for planner
    setLaserPoints(laser_points);
    setRobotPose(robot_pose);      
  }

  void ROSPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

    
    
    Eigen::Vector3f new_goal(msg->pose.position.x,
			     msg->pose.position.y,
			     tf::getYaw(msg->pose.orientation));

    setGoal(new_goal);

    ROS_INFO("Setting goal (%.6f): %.3f %.3f %.3f",
             ros::Time::now().toSec(),
             new_goal.x(),
             new_goal.y(),
             new_goal.z());
  }
  
  void ROSPlanner::mapCallback(const::nav_msgs::OccupancyGrid& msg) {

    //!TODO: check map changes (e.g. exploration usage)

    std::cerr << "Got map: " << msg.info.height << "x" << msg.info.width << std::endl;
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
            map_image.at<unsigned char>(r,c)=(unsigned char)(d);
            k++;
        }
    }

    cv::transpose(map_image, map_image);
    cv::flip(map_image, map_image, 0);

    Eigen::Vector3f map_origin(msg.info.origin.position.x,
			       msg.info.origin.position.y,
			       tf::getYaw(msg.info.origin.orientation));

    setMapFromImage(map_image, msg.info.resolution, map_origin, 0.65, 0.05);
    reset();
  }


  void ROSPlanner::cancelCallback(const actionlib_msgs::GoalID& msg) {
    ROS_INFO("Goal cancelled!!!");
    cancelGoal();
  }

  void ROSPlanner::resetCallback(const std_msgs::Bool& msg) {
    ROS_INFO("Calling reset!!!");
    reset();
  }

  void ROSPlanner::startCmdVelPublisher(){
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(_cmd_vel_topic, 1);
  }

  void ROSPlanner::startPathPublisher(){
    _path_pub = _nh.advertise<nav_msgs::Path>(_path_topic, 1);
  }

  void ROSPlanner::publishPath(){
    nav_msgs::Path path;

    //Filling header
    path.header.stamp = ros::Time::now();
    path.header.frame_id = _global_frame_id;

    //Filling poses
    path.poses.resize(_path.size());
    for (size_t i = 0; i < _path.size(); i++){
      Eigen::Vector2i point = _path[i];

      Eigen::Vector2f point_image_xy = grid2world(point);
      Eigen::Vector3f point_image(point_image_xy.x(), point_image_xy.y(), 0);
      Eigen::Isometry2f point_world_transform = v2t(_image_map_origin) * v2t(point_image);
      Eigen::Vector3f point_world = t2v(point_world_transform);
      
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = _global_frame_id;
      
      pose.pose.position.x=point_world.x();
      pose.pose.position.y=point_world.y();
      pose.pose.orientation=tf::createQuaternionMsgFromYaw(point_world.z());

      path.poses[i] = pose;
    }


    _path_pub.publish(path);

  }

  void ROSPlanner::requestMap(){
    // get map via RPC
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the map...");
    std::string static_map_service = "static_map";

    while(ros::ok() && !ros::service::call(static_map_service, req, resp)){
      ROS_WARN_STREAM("Request for map " << static_map_service << " failed; trying again...");
      ros::Duration d(0.5);
      d.sleep();
    }
    
    if (!ros::ok()) 
      ros::shutdown();
    else
      mapCallback(resp.map);
  }
}
