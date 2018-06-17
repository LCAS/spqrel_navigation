#include "naoqi_planner.h"

namespace spqrel_navigation {

  using namespace srrg_planner;
  using namespace naoqi_sensor_utils;
  
  NAOqiPlanner::NAOqiPlanner(qi::SessionPtr session){
    _session = session;
    if (! _session)
      throw std::runtime_error("Error: No memory service provided");

    _memory_service = _session->service("ALMemory");
    _motion_service = _session->service("ALMotion");

    _usable_range = 2.0;
    _use_d2l = false;
    
    // Adding blind zones for Pepper's laser
    _dyn_map.clearPoints();
    _dyn_map.setTimeThreshold(30); //seconds
    
    _cycle_time_ms = 200;

    _collision_protection_desired = true;
    _collision_protection_enabled = true;
  }


  void NAOqiPlanner::getParams(boost::program_options::variables_map& vm){

    if (vm.count("use_gui"))
      useGUI(vm["use_gui"].as<bool>());

     bool use_d2l = vm["use_d2l"].as<bool>();
     setUseD2L(use_d2l);

     if (useD2L()){
       qi::AnyValue subscribe_angle_min = _memory_service.call<qi::AnyValue>("getData","NAOqiDepth2Laser/MinAngle");
       qi::AnyValue subscribe_angle_max = _memory_service.call<qi::AnyValue>("getData","NAOqiDepth2Laser/MaxAngle");
       float angle_min = subscribe_angle_min.asFloat();
       float angle_max = subscribe_angle_max.asFloat();
       _dyn_map.addBlindZone(angle_max, 60*M_PI/180);
       _dyn_map.addBlindZone(angle_min, -60*M_PI/180);
       _dyn_map.setNumRanges(2000);
     } else{
       _dyn_map.addBlindZone(30*M_PI/180, 60*M_PI/180);
       _dyn_map.addBlindZone(-30*M_PI/180, -60*M_PI/180);
     }
     
     // --map option
    std::string mapname;
    if (!vm.count("map")){ 
      std::cout << "No map provided. Exiting." << std::endl; 
      exit(0); 
    }else {
      _map_name =  vm["map"].as<std::string>();
    }
    
    if (vm.count("robot_radius"))
      setRobotRadius(vm["robot_radius"].as<float>());
    if (vm.count("max_linear_vel"))
      setMaxLinearVel(vm["max_linear_vel"].as<float>());
    if (vm.count("max_angular_vel"))
      setMaxAngularVel(vm["max_angular_vel"].as<float>());
    if (vm.count("max_linear_acc"))
      setMaxLinearAcc(vm["max_linear_acc"].as<float>());
    if (vm.count("max_angular_acc"))
      setMaxAngularAcc(vm["max_angular_acc"].as<float>());
    if (vm.count("min_angular_vel"))
      setMinAngularVel(vm["min_angular_vel"].as<float>());

    if (vm.count("goal_translation_tolerance"))
      setGoalTranslationTolerance(vm["goal_translation_tolerance"].as<float>());    
    if (vm.count("goal_rotation_tolerance"))
      setGoalRotationTolerance(vm["goal_rotation_tolerance"].as<float>());    

    if (vm.count("recovery_waiting_time"))
      setRecoveryWaitingTime(vm["recovery_waiting_time"].as<int>());    
    if (vm.count("recovery_obstacle_distance"))
      setRecoveryObstacleDistance(vm["recovery_obstacle_distance"].as<float>());    

    if (vm.count("collision_protection_desired"))
      setExternalCollisionProtectionDesired(vm["collision_protection_desired"].as<bool>());
    
    std::cerr << "SPQReL NAOqi Planner launched with params:"      << std::endl;
    std::cerr << "  use_gui: "            << _use_gui             << std::endl;
    std::cerr << "  use_d2l: "            << _use_d2l             << std::endl;
    std::cerr << "  map: "                << _map_name             << std::endl;
    std::cerr << "  robot_radius: "       << robotRadius()       << std::endl;
    std::cerr << "  max_linear_vel: "     << maxLinearVel()      << std::endl;
    std::cerr << "  max_angular_vel: "    << maxAngularVel()     << std::endl;
    std::cerr << "  max_linear_acc: "     << maxLinearAcc()      << std::endl;
    std::cerr << "  max_angular_acc: "    << maxAngularAcc()     << std::endl;
    std::cerr << "  min_angular_vel: "    << minAngularVel()     << std::endl;
    std::cerr << "  goal_translation_tolerance: " << goalTranslationTolerance() << std::endl;
    std::cerr << "  goal_rotation_tolerance: "    << goalRotationTolerance()    << std::endl;
    std::cerr << "  recovery_waiting_time: "      << recoveryWaitingTime()      << std::endl;
    std::cerr << "  recovery_obstacle_distance: " << recoveryObstacleDistance() << std::endl;
    std::cerr << "  collision_protection_desired: " <<  _collision_protection_desired << std::endl;
					    
    std::cerr << std::endl;
  }

  void NAOqiPlanner::stopRobot() {
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> STOPPING ROBOT <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    _motion_service.call<void>("stopMove");
    setExternalCollisionProtectionEnabled(true);
  }

  void NAOqiPlanner::applyVelocities(){
    if (_collision_protection_desired != _collision_protection_enabled)
      setExternalCollisionProtectionEnabled(_collision_protection_desired); 

    std::cerr << "Applying vels: " << velocities().transpose() << std::endl;
    _motion_service.call<void>("move", velocities().x(), 0, velocities().y());
  }


  void NAOqiPlanner::subscribeLaserWithPose(){
    std::string one_laser_key = "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/X/Sensor/Value";
    _subscriber_laserwpose = _memory_service.call<qi::AnyObject>("subscriber", one_laser_key.c_str());
    _signal_laserwpose_id = _subscriber_laserwpose.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::laserWithPoseCallback, this))));

  }
  
  void NAOqiPlanner::subscribeGoal(){
    //subscribe to goal changes
    _subscriber_goal = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/Goal");
    _signal_goal_id = _subscriber_goal.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::goalCallback, this, _1))));
    _subscriber_goalxy = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/GoalXY");
    _signal_goalxy_id = _subscriber_goalxy.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::goalCallbackXY, this, _1))));
  }
  
  void NAOqiPlanner::subscribeMap(){
    readMap(_map_name);
  }
  
  void NAOqiPlanner::subscribeCancel(){
    _subscriber_cancel = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/Cancel");
    _signal_cancel_id = _subscriber_cancel.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::cancelCallback, this))));
  }
  
  void NAOqiPlanner::subscribeReset(){
    _subscriber_reset = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/Reset");
    _signal_reset_id = _subscriber_reset.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::resetCallback, this))));
  }
  
  void NAOqiPlanner::subscribeExternalCollisionProtectionDesired(){
    _subscriber_externalcollisionprotectiondesired = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/ExternalCollisionProtectionDesired");
    _signal_externalcollisionprotectiondesired_id = _subscriber_externalcollisionprotectiondesired.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::externalCollisionProtectionDesiredCallback, this, _1))));
  }

  void NAOqiPlanner::subscribeMoveEnabled(){
    _subscriber_move_enabled = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/MoveEnabled");
    _signal_move_enabled_id = _subscriber_move_enabled.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::moveEnabledCallback, this, _1))));
  }
  
  void NAOqiPlanner::startSubscribers(){
    Planner::startSubscribers();
    std::cerr << "Starting subscribers NAOQI." << std::endl;

    subscribeExternalCollisionProtectionDesired();
    subscribeMoveEnabled();
  }
  
  void NAOqiPlanner::stopSubscribers(){
    _subscriber_laserwpose.disconnect(_signal_laserwpose_id);
    _subscriber_goal.disconnect(_signal_goal_id);
    _subscriber_cancel.disconnect(_signal_cancel_id);
    _subscriber_reset.disconnect(_signal_reset_id);
    _subscriber_externalcollisionprotectiondesired.disconnect(_signal_externalcollisionprotectiondesired_id);
  }

  void NAOqiPlanner::laserWithPoseCallback(){

    //get robot localization
    qi::AnyValue pose_anyvalue = _memory_service.call<qi::AnyValue>("getData", "NAOqiLocalizer/RobotPose");
    
    srrg_core::FloatVector robot_pose_floatvector = pose_anyvalue.toList<float>();
    Eigen::Vector3f robot_pose = srrg_core::fromFloatVector3f(robot_pose_floatvector);
    std::cerr << "Robot pose: " << robot_pose.transpose() << std::endl;

    Vector2fVector laser_points;
    laser_points = getLaser(_memory_service, _usable_range); //Pepper's Laser points
    if (useD2L()) {
      Vector2fVector laser_points_d2l = getLaserFromDepth(_memory_service, _usable_range); //Depth2Laser points
      laser_points.insert(laser_points.end(), laser_points_d2l.begin(), laser_points_d2l.end()); 
    }
    
    // Setting data for planner
    setRobotPose(robot_pose);
    setLaserPoints(laser_points);
  }
  
  void NAOqiPlanner::goalCallback(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> GOAL CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    
    srrg_core::FloatVector goal_vector = value.toList<float>();
    if (goal_vector.size() != 3){
      std::cerr << "not a valid goal" << std::endl;
      return;
    }

    Eigen::Vector3f new_goal(goal_vector[0], goal_vector[1], goal_vector[2]);
    setGoal(new_goal);

    std::cerr << "Setting goal[x,y,theta] "
	      << new_goal.x() << ", "
	      << new_goal.y() << ", "
	      << new_goal.z() << std::endl;
    
  }

  void NAOqiPlanner::goalCallbackXY(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> GOAL XY CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;

    srrg_core::FloatVector goal_vector = value.toList<float>();
    if (goal_vector.size() != 2){
      std::cerr << "not a valid goal" << std::endl;
      return;
    }

    Eigen::Vector2f new_goal(goal_vector[0], goal_vector[1]);
    setGoalXY(new_goal);

    std::cerr << "Setting goal[x,y] "
	      << new_goal.x() << ", "
	      << new_goal.y() << std::endl;
    
  }
  
  void NAOqiPlanner::cancelCallback(){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> Cancel CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    cancelGoal();
  }
  
  void NAOqiPlanner::resetCallback(){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> Reset CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    reset();
  }

  void NAOqiPlanner::externalCollisionProtectionDesiredCallback(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> External Collision Protection Desired CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    _collision_protection_desired = value.as<bool>();
  }

  void NAOqiPlanner::moveEnabledCallback(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> Move Enabled CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    setMoveEnabled(value.as<bool>());
    if (moveEnabled())
      std::cerr << "Move enabled" << std::endl;
    else 
      std::cerr << "Move disabled" << std::endl;
  }

  void NAOqiPlanner::publishPath(){
    FloatVector path_vector;
    if (_path.size()){
      path_vector.resize(_path.size()*3);

      for (size_t i = 0; i < _path.size(); i++){
	Eigen::Vector2i point = _path[i];

	Eigen::Vector2f point_image_xy = grid2world(point);
	Eigen::Vector3f point_image(point_image_xy.x(), point_image_xy.y(), 0);
	Eigen::Isometry2f point_world_transform = v2t(_image_map_origin) * v2t(point_image);
	Eigen::Vector3f point_world = t2v(point_world_transform);
      
	path_vector[3*i] = point_world.x();
	path_vector[3*i+1] = point_world.y();
	path_vector[3*i+2] = point_world.z();
      }
      
      _memory_service.call<void>("insertData", "NAOqiPlanner/Path", path_vector);
    }
  }

  void NAOqiPlanner::publishResult(PlannerResult result){
    switch (result){
    case GoalReached:
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/Result", "GoalReached");
      break;

    case Aborted:
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/Result", "Aborted");
      break;

    }
  }

  void NAOqiPlanner::start(){
    _stop_thread = false;
    _running_thread = std::thread(&NAOqiPlanner::run, this);

    std::cerr << "Planner Thread launched." << std::endl;
  }
  
  void NAOqiPlanner::run() {
    while (!_stop_thread){
      std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
      laserWithPoseCallback();

      std::chrono::steady_clock::time_point runOnce_time_start = std::chrono::steady_clock::now();

      runOnce();
      std::chrono::steady_clock::time_point runOnce_time_end = std::chrono::steady_clock::now();
      int runOnce_cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(runOnce_time_end - runOnce_time_start).count();
      std::cerr << "Cycle runOnce " << runOnce_cycle_ms << " milliseconds" << std::endl << std::endl;
      std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
      int cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
      std::cerr << "Cycle " << cycle_ms << " milliseconds" << std::endl << std::endl;
      if (cycle_ms < _cycle_time_ms)
        usleep((_cycle_time_ms-cycle_ms)*1e3);
    }

    std::cerr << "Planner Thread finished." << std::endl;
  }


  void NAOqiPlanner::stop(){
    _stop_thread=true;
    _running_thread.join();

    stopSubscribers();
    std::cerr << "Planner stopped." << std::endl;
  }

  void NAOqiPlanner::setExternalCollisionProtectionEnabled(bool collision_protection_enabled){
    if (collision_protection_enabled == _collision_protection_enabled)
      return;
    if (collision_protection_enabled)
      std::cerr << "Enabling Pepper self collision protection" << std::endl;
    else
      std::cerr << "Warning: disabling Pepper self collision protection" << std::endl;

    try {
      _motion_service.call<void>("setExternalCollisionProtectionEnabled", "Move", collision_protection_enabled);
      _collision_protection_enabled = collision_protection_enabled;
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/ExternalCollisionProtectionEnabled",_collision_protection_enabled);
    } catch (qi::FutureUserException e) {
      std::cerr << e.what() << std::endl;
    }
  }


}

