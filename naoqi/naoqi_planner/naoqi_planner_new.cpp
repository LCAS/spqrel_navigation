#include "naoqi_planner_new.h"

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
  }


  void NAOqiPlanner::getParams(boost::program_options::variables_map& vm){
   
    bool use_gui = vm["use_gui"].as<bool>();
    useGUI(use_gui);

    // --map option
    std::string mapname;
    if (!vm.count("map")){ 
      std::cout << "No map provided. Exiting." << std::endl; 
      return; 
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
    
    std::cerr << "SPQReL NAOqi Planner launched with params:"      << std::endl;
    std::cerr << "  use_gui: "            << use_gui             << std::endl;
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
    std::cerr << std::endl;
  }

  void NAOqiPlanner::stopRobot() {
    _motion_service.call<void>("stopMove");
  }

  void NAOqiPlanner::applyVelocities(){
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

  void NAOqiPlanner::laserWithPoseCallback(){

    //get robot localization
    qi::AnyValue pose_anyvalue = _memory_service.call<qi::AnyValue>("getData", "NAOqiLocalizer/RobotPose");
    
    srrg_core::FloatVector robot_pose_floatvector = pose_anyvalue.toList<float>();
    Eigen::Vector3f robot_pose = srrg_core::fromFloatVector3f(robot_pose_floatvector);
    std::cerr << "Robot pose: " << robot_pose.transpose() << std::endl;


    Vector2fVector laser_points = getLaser(_memory_service, _usable_range);

    // Setting data for planner
    setRobotPose(robot_pose);
    setLaserPoints(laser_points);
  }
  
  void NAOqiPlanner::goalCallback(qi::AnyValue value){

    srrg_core::FloatVector goal_vector = value.toList<float>();
    if (goal_vector.size() != 3){
      std::cerr << "not a valid goal" << std::endl;
      return;
    }

    Eigen::Vector3f new_goal(goal_vector[0], goal_vector[1], goal_vector[2]);
    setGoal(new_goal);

    printf("Setting goal (%.6f): %.3f %.3f %.3f",
	   std::chrono::steady_clock::now(),
	   new_goal.x(),
	   new_goal.y(),
	   new_goal.z());
    
  }
  
  void NAOqiPlanner::cancelCallback(){
    std::cerr << "Goal cancelled!!!" << std::endl;
    cancelGoal();
  }
  
  void NAOqiPlanner::resetCallback(){
    std::cerr << "Calling reset!!!" << std::endl;
    reset();
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

  

  
}

