#include "naoqi_planner_gui.h"

namespace naoqi_planner_gui {


  NAOqiPlannerGUI::NAOqiPlannerGUI(qi::SessionPtr session){
    _session = session;

    if (! _session)
      throw std::runtime_error("Error: No memory service provided");

    _cycle_time_ms = 200;

    _usable_range = 2.0;
    
    _have_goal = false;
    _goal = Eigen::Vector2i(0,0);
    _robot_pose = Eigen::Vector3f(0.,0.,0.);

    _move_enabled = true;
    _collision_protection_enabled = true; _collision_protection_desired = true;

    _path.clear();
  }

  void NAOqiPlannerGUI::initGUI(){
    cv::namedWindow( "pepper_planner_gui", 0 );
    cv::setMouseCallback( "pepper_planner_gui", &NAOqiPlannerGUI::onMouse, this );
    std::cerr << "GUI initialized" << std::endl;
  }

  void NAOqiPlannerGUI::onMouse( int event, int x, int y, int, void* v){
    NAOqiPlannerGUI* n=reinterpret_cast<NAOqiPlannerGUI*>(v);
  
    if( event == cv::EVENT_LBUTTONDOWN ) {
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

  void NAOqiPlannerGUI::onResult(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> RESULT CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::string result = value.asString();
    if (result == "GoalReached" || result == "Aborted"){
      std::cerr << result << std::endl << std::endl;
      _have_goal = false;
      _path.clear();
    }
  }

  void NAOqiPlannerGUI::onExternalCollisionProtectionEnabled(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> External Collision Protection Enabled CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    _collision_protection_enabled = value.as<bool>();
    if (_collision_protection_enabled){
      std::cerr << "External Collision Protection Desired enabled" << std::endl;
    } else {
      std::cerr << "External Collision Protection Desired disabled" << std::endl;
    }
    
  }

  void NAOqiPlannerGUI::subscribeServices(){
    _memory_service = _session->service("ALMemory");

    //subscribe to result
    _subscriber_result = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/Result");
    _signal_result_id = _subscriber_result.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlannerGUI::onResult, this, _1))));

    //subscribe to collision protection changes
    _subscriber_collision_protection_enabled = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/ExternalCollisionProtectionEnabled");
    _signal_collision_protection_enabled_id = _subscriber_collision_protection_enabled.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlannerGUI::onExternalCollisionProtectionEnabled, this, _1))));
    
    _stop_thread=false;
    _servicesMonitorThread = std::thread(&NAOqiPlannerGUI::servicesMonitorThread, this);
    std::cerr << "Planner GUI Services Monitor Thread launched." << std::endl;
  }

  void NAOqiPlannerGUI::unsubscribeServices(){
    _subscriber_result.disconnect(_signal_result_id);
    _stop_thread=true;
    _servicesMonitorThread.join();
  }

  void NAOqiPlannerGUI::servicesMonitorThread() {
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
      //Vector2fVector laser_points = getLaser(memory_service, _usable_range);

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
  
    std::cout << "Planner GUI Monitor Thread finished." << std::endl;
  }



  void NAOqiPlannerGUI::readMap(const std::string mapname){
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
  




  void NAOqiPlannerGUI::handleGUIDisplay() {
    
    FloatImage shown_image;
    shown_image.create(_indices_image.rows, _indices_image.cols);
    for (int r=0; r<_indices_image.rows; ++r) {
      int* src_ptr=_indices_image.ptr<int>(r);
      float* dest_ptr=shown_image.ptr<float>(r);
      for (int c=0; c<_indices_image.cols; ++c, ++src_ptr, ++dest_ptr){
	if (*src_ptr<-1)
	  *dest_ptr = .5f;
	else if (*src_ptr == -1)
	  *dest_ptr = 1.f;
	else
	  *dest_ptr=0.f;
      }
    }
    
    // Drawing goal
    if (_have_goal)
      cv::circle(shown_image, cv::Point(_goal.y(), _goal.x()), 3, cv::Scalar(0.0f));

    // Drawing current pose
    cv::rectangle(shown_image,
		  cv::Point(_robot_pose_image.y()+2, _robot_pose_image.x()-2),
		  cv::Point(_robot_pose_image.y()-2, _robot_pose_image.x()+2),
		  cv::Scalar(0.0f));

    
    //Draw path
    if (_robot_pose_image.x()>=0 && _have_goal && _path.size()){
      for (size_t i = 0; i <_path.size()-1; i++){
	cv::line(shown_image,
		 cv::Point(_path[i].y(), _path[i].x()),
		 cv::Point(_path[i+1].y(), _path[i+1].x()),
		 cv::Scalar(0.0f));
	
      }
    }

    char buf[1024];
    sprintf(buf, " MoveEnabled: %d", _move_enabled);
    cv::putText(shown_image, buf, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);
    sprintf(buf, " CollisionProtectionDesired: %d", _collision_protection_desired);
    cv::putText(shown_image, buf, cv::Point(20, 30+(int)shown_image.cols*0.03), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);
    sprintf(buf, " ExternalCollisionProtectionEnabled: %d", _collision_protection_enabled);
    cv::putText(shown_image, buf, cv::Point(20, 30+(int)shown_image.cols*0.06), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);    
    
    cv::imshow("pepper_planner_gui", shown_image);
  }  


  void NAOqiPlannerGUI::handleGUIInput(){
    char key=cv::waitKey(10);
    switch(key) {
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

