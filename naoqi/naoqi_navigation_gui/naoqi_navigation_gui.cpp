#include "naoqi_navigation_gui.h"

namespace naoqi_navigation_gui {
  using namespace naoqi_sensor_utils;


  NAOqiNavigationGUI::NAOqiNavigationGUI(qi::SessionPtr session){
    _session = session;

    if (! _session)
      throw std::runtime_error("Error: No memory service provided");

    _cycle_time_ms = 200;

    _use_d2l = false;
    
    _have_goal = false;
    _goal = Eigen::Vector2i(0,0);
    _robot_pose = Eigen::Vector3f(0.,0.,0.);

    _move_enabled = true;
    _collision_protection_enabled = true; _collision_protection_desired = true;

    _laser_points.clear();
    _laser_points_d2l.clear();
    _set_pose = false;
    _path.clear();
  }

  void NAOqiNavigationGUI::initGUI(){
    cv::namedWindow( "pepper_navigation_gui", 0 );
    cv::setMouseCallback( "pepper_navigation_gui", &NAOqiNavigationGUI::onMouse, this );
    std::cerr << "GUI initialized" << std::endl;
  }

  void NAOqiNavigationGUI::onMouse( int event, int x, int y, int flags, void* v){
    NAOqiNavigationGUI* n=reinterpret_cast<NAOqiNavigationGUI*>(v);
    if (n->_set_pose) {
      if( event == cv::EVENT_LBUTTONDOWN ) {
	std::cerr << "Left Click!" << std::endl;
	n->_robot_pose_image = Eigen::Vector2i(y,x);

	Eigen::Vector2f p=n->grid2world(n->_robot_pose_image);
	Eigen::Vector3f pose(p.x(), p.y(), 0);
	Eigen::Isometry2f pose_origin = v2t(n->_image_map_origin)*v2t(pose);
	Eigen::Vector3f new_pose = t2v(pose_origin);
      
	Eigen::Isometry2f old_robot_pose_transform=v2t(n->_image_map_origin)*v2t(n->_robot_pose);
	Eigen::Vector3f old_robot_pose = t2v(old_robot_pose_transform);

	new_pose.z() = old_robot_pose.z(); //preserve angle, just x,y changes
	std::cerr << "Setting pose: " << new_pose.transpose() << std::endl;
	FloatVector vpose;
	vpose.push_back(new_pose.x());
	vpose.push_back(new_pose.y());
	vpose.push_back(new_pose.z());
	qi::AnyValue value = qi::AnyValue::from(vpose);
	n->_memory_service.call<void>("raiseEvent", "NAOqiLocalizer/SetPose", value);

      }
      if( event == cv::EVENT_RBUTTONDOWN ) {
	std::cerr << "Right Click!" << std::endl;
	Eigen::Vector2f p_angle=n->grid2world(Eigen::Vector2i(y,x));
	Eigen::Vector2f p_pose=n->grid2world(n->_robot_pose_image);
	
	Eigen::Vector2f dp=p_angle-p_pose;
	float angle=atan2(dp.y(), dp.x());
	Eigen::Vector3f pose(p_pose.x(), p_pose.y(), angle);
	Eigen::Isometry2f pose_origin = v2t(n->_image_map_origin)*v2t(pose);
	Eigen::Vector3f new_pose = t2v(pose_origin);
      
	FloatVector vpose;
	vpose.push_back(new_pose.x());
	vpose.push_back(new_pose.y());
	vpose.push_back(new_pose.z());
	qi::AnyValue value = qi::AnyValue::from(vpose);
	n->_memory_service.call<void>("raiseEvent", "NAOqiLocalizer/SetPose", value);
	
      }

      
    } else {
      if( event == cv::EVENT_LBUTTONDOWN && ((flags & cv::EVENT_FLAG_CTRLKEY) != 0)) {
	std::cerr << "Left Click!" << std::endl;
	n->_goal = Eigen::Vector2i(y,x);
	n->_have_goal = true;
	std::cerr << "Setting goal: " << n->_goal.transpose() << std::endl;

	Eigen::Vector2f goal_image_xy = n->grid2world(n->_goal);
	Eigen::Vector3f goal_image(goal_image_xy.x(), goal_image_xy.y(), 0);
	Eigen::Isometry2f goal_transform = v2t(n->_image_map_origin) * v2t(goal_image);

	FloatVector goal;
	goal.push_back(goal_transform.translation().x());
	goal.push_back(goal_transform.translation().y());
	qi::AnyValue value = qi::AnyValue::from(goal);
	n->_memory_service.call<void>("raiseEvent", "NAOqiPlanner/GoalXY", value);
      }
    }
  }

  void NAOqiNavigationGUI::onResult(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> RESULT CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::string result = value.asString();
    if (result == "GoalReached" || result == "Aborted"){
      std::cerr << result << std::endl << std::endl;
      _have_goal = false;
      _path.clear();
    }
  }

  void NAOqiNavigationGUI::onGoal(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> GOAL CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    srrg_core::FloatVector goal = value.toList<float>();
    if (goal.size() != 3){
      std::cerr << "not a valid goal" << std::endl;
      return;
    }

    // we receive the goal in world coordinates [m]

    Eigen::Vector3f vgoal(goal[0], goal[1], goal[3]);
    std::cerr << "Setting goal [m]: " << vgoal.transpose() << std::endl;
    Eigen::Isometry2f goal_transform=_image_map_origin_transform_inverse*v2t(vgoal);

    _goal = world2grid(Eigen::Vector2f(goal_transform.translation().x(), goal_transform.translation().y()));

    _have_goal = true;
  }

  void NAOqiNavigationGUI::onExternalCollisionProtectionEnabled(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> External Collision Protection Enabled CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    _collision_protection_enabled = value.as<bool>();
    if (_collision_protection_enabled){
      std::cerr << "External Collision Protection Desired enabled" << std::endl;
    } else {
      std::cerr << "External Collision Protection Desired disabled" << std::endl;
    }
    
  }

  void NAOqiNavigationGUI::subscribeServices(){
    _memory_service = _session->service("ALMemory");

    //subscribe to result
    _subscriber_result = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/Result");
    _signal_result_id = _subscriber_result.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiNavigationGUI::onResult, this, _1))));

    //subscribe to collision protection changes
    _subscriber_collision_protection_enabled = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/ExternalCollisionProtectionEnabled");
    _signal_collision_protection_enabled_id = _subscriber_collision_protection_enabled.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiNavigationGUI::onExternalCollisionProtectionEnabled, this, _1))));
    
    //subscribe to goal changes
    _subscriber_goal = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/Goal");
    _signal_goal_id = _subscriber_goal.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiNavigationGUI::onGoal, this, _1))));

    _stop_thread=false;
    _servicesMonitorThread = std::thread(&NAOqiNavigationGUI::servicesMonitorThread, this);
    std::cerr << "Planner GUI Services Monitor Thread launched." << std::endl;
  }

  void NAOqiNavigationGUI::unsubscribeServices(){
    _subscriber_result.disconnect(_signal_result_id);
    _subscriber_goal.disconnect(_signal_goal_id);
    _stop_thread=true;
    _servicesMonitorThread.join();
  }

  void NAOqiNavigationGUI::servicesMonitorThread() {
    while (!_stop_thread){
      std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
    
      qi::AnyValue value;

      try {
	//get robot localization
	value = _memory_service.call<qi::AnyValue>("getData", "NAOqiLocalizer/RobotPose");
	srrg_core::FloatVector robot_pose_floatvector = value.toList<float>();
	Eigen::Vector3f robot_pose_vector = srrg_core::fromFloatVector3f(robot_pose_floatvector);
	std::cerr << "Robot pose map: " << robot_pose_vector.transpose() << std::endl;
	
	Eigen::Isometry2f robot_pose_transform=_image_map_origin_transform_inverse*v2t(robot_pose_vector);
	_robot_pose = t2v(robot_pose_transform);
	
	_robot_pose_image = world2grid(Eigen::Vector2f(_robot_pose.x(), _robot_pose.y()));
      } catch (qi::FutureUserException) {
	std::cerr << "NAOqiLocalizer/RobotPose not available" << std::endl;
      }
      //get laser
      _laser_points = getLaser(_memory_service);
      if (useD2L()) {
	_laser_points_d2l = getLaserFromDepth(_memory_service); //Depth2Laser points
      }
      
      try{
	//get path
	value = _memory_service.call<qi::AnyValue>("getData", "NAOqiPlanner/Path");
	srrg_core::FloatVector path_vector = value.toList<float>();
	_path.resize(path_vector.size()/3);
	for (size_t i=0; i<_path.size(); i++){
	  Eigen::Vector3f point_world = Eigen::Vector3f(path_vector[3*i], path_vector[3*i+1], path_vector[3*i+2]);
	  Eigen::Isometry2f point_world_transform = _image_map_origin_transform_inverse*v2t(point_world);
	  Eigen::Vector3f point_image = t2v(point_world_transform);
	  
	  _path[i] = world2grid(Eigen::Vector2f(point_image.x(), point_image.y()));
	}
      } catch (qi::FutureUserException) {
	std::cerr << "NAOqiPlanner/Path not available" << std::endl;
      }
      
      handleGUIDisplay();
      handleGUIInput();

      std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
      int cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
      std::cerr << "Cycle " << cycle_ms << " milliseconds" << std::endl << std::endl;
      if (cycle_ms < _cycle_time_ms)
	usleep((_cycle_time_ms-cycle_ms)*1e3);
    
    }
  
    std::cout << "Navigation GUI Monitor Thread finished." << std::endl;
  }



  void NAOqiNavigationGUI::readMap(const std::string mapname){
    std::cerr << "Reading map" << mapname << std::endl;
  
    // reading map info
    SimpleYAMLParser parser;
    parser.load(mapname);
    std::cerr << "Dirname: " << dirname(strdup(mapname.c_str())) << std::endl;
    
    std::string map_image_name = parser.getValue("image");
    _map_resolution = parser.getValueAsFloat("resolution");
    _map_inverse_resolution = 1./_map_resolution;
    _occ_threshold = parser.getValueAsFloat("occupied_thresh");
    _free_threshold = parser.getValueAsFloat("free_thresh");
    _map_origin = parser.getValueAsVector3f("origin");
    _map_origin_transform_inverse = v2t(_map_origin).inverse();
    
    std::cerr << "MAP NAME: " << map_image_name << std::endl;
    std::cerr << "RESOLUTION: " << _map_resolution << std::endl;
    std::cerr << "ORIGIN: " << _map_origin.transpose() << std::endl;
    std::cerr << "OCC THRESHOLD: " << _occ_threshold << std::endl;
    std::cerr << "FREE THRESHOLD: " << _free_threshold << std::endl;
      
    std::string full_path_map_image = std::string(dirname(strdup(mapname.c_str())))+"/"+map_image_name;
    std::cerr << "Opening image" << full_path_map_image << std::endl;
  
    _map_image = cv::imread(full_path_map_image, CV_LOAD_IMAGE_GRAYSCALE);
    std::cerr << "Image read: (" << _map_image.rows << "x" << _map_image.cols << ")" << std::endl;

    // _map_origin: reference system bottom-left, X right, Y up  (values read from yaml file) (ROS convention)
    // _image_map_origin: image reference system top-left, X down, Y right (opencv convention)

    // transform from _map_origin to _image_map_origin
    Eigen::Vector3f map_to_image(0,_map_image.rows*_map_resolution,-M_PI/2);
    Eigen::Isometry2f tf = v2t(_map_origin) * v2t(map_to_image);
    _image_map_origin = t2v(tf);
    _image_map_origin_transform_inverse = v2t(_image_map_origin).inverse();;

    
    int occ_threshold = (1.0 - _occ_threshold) * 255;
    int free_threshold = (1.0 - _free_threshold) * 255;
    grayMap2indices(_indices_image, _map_image, occ_threshold, free_threshold);
  }
  




  void NAOqiNavigationGUI::handleGUIDisplay() {
    
    RGBImage shown_image;
    cvtColor(_map_image, shown_image, CV_GRAY2BGR);
    
    // Drawing goal
    if (_have_goal)
      cv::circle(shown_image, cv::Point(_goal.y(), _goal.x()), 3, cv::Scalar(255,0,0));

    // Drawing current pose
    cv::rectangle(shown_image,
		  cv::Point(_robot_pose_image.y()+2, _robot_pose_image.x()-2),
		  cv::Point(_robot_pose_image.y()-2, _robot_pose_image.x()+2),
		  cv::Scalar(0,0,255));

    
    //Draw path
    if (_robot_pose_image.x()>=0 && _have_goal && _path.size()){
      for (size_t i = 0; i <_path.size()-1; i++){
	cv::line(shown_image,
		 cv::Point(_path[i].y(), _path[i].x()),
		 cv::Point(_path[i+1].y(), _path[i+1].x()),
		 cv::Scalar(0,0,0));
	
      }
    }

    //Draw laserscans
    for (size_t i=0; i<_laser_points.size(); i++){
      Eigen::Vector2f lp=v2t(_robot_pose)* _laser_points[i];
      int r = lp.x()*_map_inverse_resolution;
      int c = lp.y()*_map_inverse_resolution;
      cv::circle(shown_image, cv::Point(c, r), 3, cv::Scalar(200,200,0));
    }

    for (size_t i=0; i<_laser_points_d2l.size(); i++){
      Eigen::Vector2f lp=v2t(_robot_pose)* _laser_points_d2l[i];
      int r = lp.x()*_map_inverse_resolution;
      int c = lp.y()*_map_inverse_resolution;
      cv::circle(shown_image, cv::Point(c, r), 3, cv::Scalar(200,0,0));
    }

    char buf[1024];
    sprintf(buf, " SetPose: %d", _set_pose);
    cv::putText(shown_image, buf, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(200,0,200), 1);
      
    sprintf(buf, " MoveEnabled: %d", _move_enabled);
    cv::putText(shown_image, buf, cv::Point(20, 50+(int)shown_image.cols*0.03), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(200,0,200), 1);
    sprintf(buf, " CollisionProtectionDesired: %d", _collision_protection_desired);
    cv::putText(shown_image, buf, cv::Point(20, 50+(int)shown_image.cols*0.06), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(200,0,200), 1);
    sprintf(buf, " ExternalCollisionProtectionEnabled: %d", _collision_protection_enabled);
    cv::putText(shown_image, buf, cv::Point(20, 50+(int)shown_image.cols*0.09), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(200,0,200), 1);    
    
    cv::imshow("pepper_navigation_gui", shown_image);
  }  


  void NAOqiNavigationGUI::handleGUIInput(){
    char key=cv::waitKey(10);
    switch(key) {
    case 's':
      _set_pose = !_set_pose;
      break;
    case 'p':
      _move_enabled = !_move_enabled;
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/MoveEnabled", _move_enabled);
      break;
    case 'r':
      std::cerr << "Resetting" << std::endl;
      _have_goal = false;
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/Reset", true);
      break;
    case 'o':
      _collision_protection_desired = ! _collision_protection_desired;
      std::cerr << "External Collision Protection Desired: " << _collision_protection_desired << std::endl;
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/ExternalCollisionProtectionDesired", _collision_protection_desired);
    default:;
    }
  }


}

