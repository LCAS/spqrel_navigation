#include "naoqi_planner.h"


namespace naoqi_planner {
  using namespace naoqi_sensor_utils;
  
  NAOqiPlanner::NAOqiPlanner(qi::SessionPtr session){
    _session = session;
    if (! _session)
      throw std::runtime_error("Error: No memory service provided");

    _use_gui = false;
    _what_to_show = Map;

    _cycle_time_ms = 200;
    _restart = true;

    _usable_range = 2.0;
    
    _have_goal = false;
    _goal = Eigen::Vector2i(0,0);
    _robot_pose = Eigen::Vector3f(0.,0.,0.);

    _move_enabled = true;
    _collision_protection_enabled = true; _collision_protection_desired = true;
    
    _state = WaitingForGoal;

    _prev_v = 0.0; _prev_w = 0.0;

    _dyn_map.clearPoints();
    _dyn_map.setTimeThreshold(30); //seconds
    _dyn_map.addBlindZone(30*M_PI/180, 60*M_PI/180);
    _dyn_map.addBlindZone(-30*M_PI/180, -60*M_PI/180);

    _time_last_reloc = std::chrono::steady_clock::now();

    _path.clear();
    _nominal_path.clear();
    _obstacle_path.clear();


  }

  void NAOqiPlanner::initGUI(){
    _use_gui=true;
    cv::namedWindow( "pepper_planner", 0 );
    cv::setMouseCallback( "pepper_planner", &NAOqiPlanner::onMouse, this );
    std::cerr << "GUI initialized" << std::endl;
  }

  void NAOqiPlanner::onMouse( int event, int x, int y, int, void* v)
  {
    NAOqiPlanner* n=reinterpret_cast<NAOqiPlanner*>(v);
    
    if( event == cv::EVENT_LBUTTONDOWN ) {
      std::cerr << "Left Click!" << std::endl;
      n->_goal = Eigen::Vector2i(y,x);
      n->_have_goal = true;
      std::cerr << "Setting goal: " << n->_goal.transpose() << std::endl;
      if (n->_state != GoalReceived){
	n->_state = GoalReceived;
	n->publishState();
      }
    }
    
  }

  void NAOqiPlanner::reset(){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> RESET <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;

    _restart = true;
    cancelGoal();
    //Removing obstacles
    int occ_threshold = (1.0 - _occ_threshold) * 255;
    int free_threshold = (1.0 - _free_threshold) * 255;
    grayMap2indices(_indices_image, _map_image, occ_threshold, free_threshold);
    _dyn_map.clearPoints();
  }
  
  void NAOqiPlanner::handleGUIInput(){
    if (! _use_gui)
      return;

    char key=cv::waitKey(10);
    switch(key) {
    case 'h':
      std::cout << "m: map mode" << std::endl;
      std::cout << "d: distance map" << std::endl;
      std::cout << "c: cost map" << std::endl;
      std::cout << "p: enable/disable motion" << std::endl;
      std::cout << "r: reset distance/cost map and remove the goal" << std::endl <<
              "   (can be used for emergency stop)" << std::endl;
      std::cout << "o: enable/disable external collision protection" << std::endl;
	break;
    case 'm':
      if (_what_to_show == Map)
	break;
      std::cerr << "Switching view to map" << std::endl;
      _what_to_show = Map;
      break;
    case 'd': 
      if (_what_to_show == Distance)
	break;
      std::cerr << "Switching view to distance map" << std::endl;
      _what_to_show = Distance;
      break;
    case 'c': 
      if (_what_to_show == Cost)
	break;
      std::cerr << "Switching view to cost map" << std::endl;
      _what_to_show = Cost;
      break;
    case 'p':    
      _move_enabled = !_move_enabled;
      std::cerr << "Move enabled: " << _move_enabled << std::endl;
      break;
    case 'r':
      std::cerr << "Resetting" << std::endl;
      reset();
      break;
    case 'o':
      _collision_protection_desired = ! _collision_protection_desired;
      std::cerr << "External Collision Protection Desired: " << _collision_protection_desired << std::endl;
    default:;
    }
  }


  void NAOqiPlanner::handleGUIDisplay() {
    if (_use_gui) {
      FloatImage shown_image;
      switch(_what_to_show){
      case Map:
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
	break;
	
      case Distance:
	shown_image=_distance_image*(1./_safety_region);
	break;
      case Cost:
	shown_image=_cost_image*(1.f/_max_cost);
	break;
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
      if (_have_goal && _path.size()>1){
	for (size_t i=0; i<_path.size()-1; i++){
	  Eigen::Vector2i cell_from = _path[i];
	  Eigen::Vector2i cell_to   = _path[i+1];
	  cv::line(shown_image,
		   cv::Point(cell_from.y(), cell_from.x()),
		   cv::Point(cell_to.y(), cell_to.x()),
		   cv::Scalar(0.0f));
	}

	if (_nominal_path.size()>1){
	  for (size_t i=0; i<_nominal_path.size()-1; i++){

	  Eigen::Vector2i cell_from = _nominal_path[i];
	  Eigen::Vector2i cell_to   = _nominal_path[i+1];
	  cv::line(shown_image,
		   cv::Point(cell_from.y(), cell_from.x()),
		   cv::Point(cell_to.y(), cell_to.x()),
		   cv::Scalar(0.5f));
	  
	  
	  }
	}
      }

      //Draw laser
      for (size_t i=0; i<_laser_points.size(); i++){
	Eigen::Vector2f lp=v2t(_robot_pose)* _laser_points[i];
	int r = lp.x()*_map_inverse_resolution;
	int c = lp.y()*_map_inverse_resolution;
	if (! _distance_map.inside(r,c))
	  continue;
	cv::circle(shown_image, cv::Point(c, r), 3, cv::Scalar(1.0f));
      }

      
      char buf[1024];
      sprintf(buf, " MoveEnabled: %d", _move_enabled);
      cv::putText(shown_image, buf, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);
      sprintf(buf, " CollisionProtectionDesired: %d", _collision_protection_desired);
      cv::putText(shown_image, buf, cv::Point(20, 50+(int)shown_image.cols*0.03), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);
      sprintf(buf, " ExternalCollisionProtectionEnabled: %d", _collision_protection_enabled);
      cv::putText(shown_image, buf, cv::Point(20, 50+(int)shown_image.cols*0.06), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);

      
      cv::imshow("pepper_planner", shown_image);
    }  
    
  }
  
  void NAOqiPlanner::readMap(const std::string mapname){
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

  void NAOqiPlanner::onGoal(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> GOAL CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    srrg_core::FloatVector goal = value.toList<float>();
    if (goal.size() != 2){
      std::cerr << "not a valid goal" << std::endl;
      return;
    }

    // we receive the goal in world coordinates [m]

    Eigen::Vector3f vgoal(goal[0], goal[1], 0);
    std::cerr << "Setting goal [m]: " << vgoal.transpose() << std::endl;
    Eigen::Isometry2f goal_transform=_image_map_origin_transform_inverse*v2t(vgoal);

    _goal = world2grid(Eigen::Vector2f(goal_transform.translation().x(), goal_transform.translation().y()));

    _have_goal = true;
    _state = GoalReceived;
    publishState();
  }

  void NAOqiPlanner::onMoveEnabled(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> Move Enabled CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    _move_enabled = value.as<bool>();
    if (_move_enabled){
      std::cerr << "Move enabled" << std::endl;
    } else {
      std::cerr << "Move disabled" << std::endl;
    }
  }

  void NAOqiPlanner::onCollisionProtectionDesired(qi::AnyValue value){
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> Move Collision Protection Desired CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    _collision_protection_desired = value.as<bool>();
    if (_collision_protection_desired){
      std::cerr << "External Collision Protection Desired enabled" << std::endl;
    } else {
      std::cerr << "External Collision Protection Desired disabled" << std::endl;
    }
  }
  
  void NAOqiPlanner::subscribeServices(){
    _memory_service = _session->service("ALMemory");
    _motion_service = _session->service("ALMotion");

    //subscribe to goal changes
    _subscriber_goal = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/Goal");
    _signal_goal_id = _subscriber_goal.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::onGoal, this, _1))));

    //subscribe to move enabled
    _subscriber_move_enabled = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/MoveEnabled");
    _signal_move_enabled_id = _subscriber_move_enabled.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::onMoveEnabled, this, _1))));

    //subscribe to collision protection desired 
    _subscriber_collision_protection_desired = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/CollisionProtectionDesired");
    _signal_collision_protection_desired_id = _subscriber_collision_protection_desired.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::onCollisionProtectionDesired, this, _1))));

    //subscribe to reset
    _subscriber_reset = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiPlanner/Reset");
    _signal_reset_id = _subscriber_reset.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiPlanner::reset, this))));
    
    setExternalCollisionProtectionEnabled(true);
    
    _stop_thread=false;
    _servicesMonitorThread = std::thread(&NAOqiPlanner::servicesMonitorThread, this);
    std::cerr << "Planner Services Monitor Thread launched." << std::endl;
  }

  void NAOqiPlanner::unsubscribeServices(){
    _subscriber_goal.disconnect(_signal_goal_id);
    _subscriber_move_enabled.disconnect(_signal_move_enabled_id);
    _subscriber_collision_protection_desired.disconnect(_signal_collision_protection_desired_id);
    _stop_thread=true;
    _servicesMonitorThread.join();
  }
  

  void NAOqiPlanner::cancelGoal() {
    _have_goal = false;
    // stop the robot
    _motion_service.call<void>("stopMove");
    setExternalCollisionProtectionEnabled(true);
  }

  void NAOqiPlanner::setExternalCollisionProtectionEnabled(bool value) {
    if (value)
      std::cerr << "Enabling Pepper self collision protection" << std::endl;
    else
      std::cerr << "Warning: disabling Pepper self collision protection" << std::endl;

    try {
      _motion_service.call<void>("setExternalCollisionProtectionEnabled", "Move", value);
      _collision_protection_enabled = value;
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/ExternalCollisionProtectionEnabled",_collision_protection_enabled);
    } catch (qi::FutureUserException e) {
      
      std::cerr << e.what() << std::endl;

    }
  }

  void NAOqiPlanner::servicesMonitorThread() {

    while (!_stop_thread){
      publishState();
      std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

      //get robot localization
      qi::AnyValue pose_anyvalue = _memory_service.call<qi::AnyValue>("getData", "NAOqiLocalizer/RobotPose");

      srrg_core::FloatVector robot_pose_floatvector = pose_anyvalue.toList<float>();
      Eigen::Vector3f robot_pose_vector = srrg_core::fromFloatVector3f(robot_pose_floatvector);
      std::cerr << "Robot pose: " << robot_pose_vector.transpose() << std::endl;
      
      Eigen::Isometry2f robot_pose_transform=_image_map_origin_transform_inverse*v2t(robot_pose_vector);
      _robot_pose = t2v(robot_pose_transform); // image coordinates


      std::cerr << "Image pose [m]: " << _robot_pose.transpose() << std::endl;

      _robot_pose_image = world2grid(Eigen::Vector2f(_robot_pose.x(), _robot_pose.y()));

      std::cerr << "Image pose [pixel]: " << _robot_pose_image.transpose() << std::endl;
      
      //get laser
      _laser_points = getLaser(_memory_service, _usable_range);

      //here I'll do something map + loc + laser + goal -> path
      if (_restart){
	_dmap_calculator.setMaxDistance(_safety_region/_map_resolution);
	_dmap_calculator.setIndicesImage(_indices_image);
	_dmap_calculator.setOutputPathMap(_distance_map);
	_dmap_calculator.init();
	_max_distance_map_index = _dmap_calculator.maxIndex();
	_dmap_calculator.compute();
	_distance_map_backup=_distance_map.data();

	//I'm doing also backup of the cost_map without obstacles
	_distance_image = _dmap_calculator.distanceImage()*_map_resolution;
	distances2cost(_cost_image_backup,
		       _distance_image,
		       _robot_radius,
		       _safety_region,
		       _min_cost,
		       _max_cost);

	_restart = false;
	
      } 

      if (_have_goal){
	//Get nominal path without obstacles
	computePath(_cost_image_backup, _path_map_backup, _goal, _nominal_path);

	//Adding obstacles
	std::chrono::steady_clock::time_point time_dmap_start = std::chrono::steady_clock::now();
	_distance_map.data()=_distance_map_backup;

	if (_laser_points.size()>0) {
	  
	  _dyn_map.setMapResolution(_map_resolution);
	  _dyn_map.setRobotPose(_robot_pose);
	  _dyn_map.setCurrentPoints(_laser_points);
	  _dyn_map.compute();
	  Vector2iVector obstacle_points;
	  _dyn_map.getOccupiedCells(obstacle_points);
	  
	  _dmap_calculator.setPoints(obstacle_points, _max_distance_map_index);
	  _dmap_calculator.compute();
	  
	}else{
	  std::cerr << "WARNING: laser data not available." << std::endl;
	}
	
	_distance_image = _dmap_calculator.distanceImage()*_map_resolution;
	distances2cost(_cost_image,
		       _distance_image,
		       _robot_radius,
		       _safety_region,
		       _min_cost,
		       _max_cost);

	std::chrono::steady_clock::time_point time_dmap_end = std::chrono::steady_clock::now();
	std::cerr << "DMapCalculator: "
		  << std::chrono::duration_cast<std::chrono::milliseconds>(time_dmap_end - time_dmap_start).count() << " ms" << std::endl;
	
	computePath(_cost_image, _path_map, _goal, _obstacle_path);

	float path_diff_threshold = 1.5; //meters
	int num_path_diff_cells = path_diff_threshold * _map_inverse_resolution;

	//if (!_path.size() ){
	  //Check the reason of not path found
	  //1) goal is on an obstacle,
	  //2) robot is badly localized so obstacles are projected wrongly into the map
	  /*PathMapCell* cell_goal=&_path_map(_goal.x(), _goal.y());
	  if (cell_goal->cost == getMaxCost()){
	    //Recovery procedure for goal at obstacle
	    std::cerr << ">>>>>>>>>>>>>>>>>>> Goal at obstacle <<<<<<<<<<<<<<<<<<<" << std::endl;

	  }else {
	    //Recovery procedure for robot bad localized
	    int time_ellapsed_last_reloc = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - _time_last_reloc).count();
	    int reloc_rate_seconds = 5;
	    if (time_ellapsed_last_reloc > reloc_rate_seconds){
	      std::cerr << ">>>>>>>>>>>>>>>>>>> Call Relocalization <<<<<<<<<<<<<<<<<<<" << std::endl;
	      _memory_service.call<void>("raiseEvent", "NAOqiLocalizer/SetPose", pose_anyvalue);
	      _time_last_reloc = std::chrono::steady_clock::now();
	    }
	    
	    }*/

	  //recoveryPlan();
	  //}

	//We try to follow the nominal path always, toponav would be in charge of replanning the global path
	
	if (!_obstacle_path.size() || (_obstacle_path.size() - _nominal_path.size()) > num_path_diff_cells){
	  //no path with obstacles, or path very different from nominal once, we try to approach obstacle
	  //Recovery path
	  recoveryPlan();
	}else{
	  //everything is fine, we use the path with obstacles
	  _path = _obstacle_path;
	}

	publishPath();
		
	float linear_vel, angular_vel;
	computeControlToWaypoint(linear_vel, angular_vel);
	if (_move_enabled){

	  if (_have_goal && _collision_protection_desired != _collision_protection_enabled) {
	    setExternalCollisionProtectionEnabled(_collision_protection_desired); 
	  }

	  if (_state != PathNotFound){
	    //apply vels
	    std::cerr << "Applying vels: " << linear_vel  << " " << angular_vel << std::endl;
	    _motion_service.call<void>("move", linear_vel,0,angular_vel);
	  } else {
	    std::cerr << "PATH NOT FOUND: STOPPING ROBOT" << std::endl;
	    if (_prev_v != 0 ||_prev_w != 0){
	      _prev_v = 0;
	      _prev_w = 0;
	      _motion_service.call<void>("stopMove");
	      std::cerr << "SENDING STOPMOVE" << std::endl;
	      

	    }
	  }
	}else{
	  _prev_v = 0;
	  _prev_w = 0;
	  _motion_service.call<void>("stopMove");
	  // we are not controlling the robot, set collision avoidance
	  setExternalCollisionProtectionEnabled(true);
	} // _move_enabled
	
      } // _have_goal //else nothing to compute
      
      

      handleGUIDisplay();
    
      handleGUIInput();
    
      std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
      int cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
      std::cerr << "Cycle " << cycle_ms << " milliseconds" << std::endl << std::endl;
      if (cycle_ms < _cycle_time_ms)
        usleep((_cycle_time_ms-cycle_ms)*1e3);


     // if (!_have_goal) // sleep 1 sec
     //   usleep(1e6);

    } // main while

    setExternalCollisionProtectionEnabled(true);
    _motion_service.call<void>("stopMove");
	  
    std::cout << "Planner Monitor Thread finished." << std::endl;

  }

  void NAOqiPlanner::recoveryRelocalize(){
    int time_ellapsed_last_reloc = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - _time_last_reloc).count();
    int reloc_rate_seconds = 5;
    if (time_ellapsed_last_reloc > reloc_rate_seconds){
      std::cerr << ">>>>>>>>>>>>>>>>>>> Call Relocalization <<<<<<<<<<<<<<<<<<<" << std::endl;
      
      qi::AnyValue pose_anyvalue = _memory_service.call<qi::AnyValue>("getData", "NAOqiLocalizer/RobotPose");
      //setting pose will expand particles around current pose.
      _memory_service.call<void>("raiseEvent", "NAOqiLocalizer/SetPose", pose_anyvalue);
      _time_last_reloc = std::chrono::steady_clock::now();
    }
    
    
  } 

  void NAOqiPlanner::computePath(FloatImage& cost_map, PathMap& path_map, Eigen::Vector2i& goal, Vector2iVector &path){
    std::chrono::steady_clock::time_point time_path_start = std::chrono::steady_clock::now();
    _path_calculator.setMaxCost(_max_cost-1);
    _path_calculator.setCostMap(cost_map);
    _path_calculator.setOutputPathMap(path_map);
    Vector2iVector goals;
    goals.push_back(goal);
    _path_calculator.goals() = goals;
    _path_calculator.compute();
    std::chrono::steady_clock::time_point time_path_end = std::chrono::steady_clock::now();
    path.clear();
    // Filling path
    PathMapCell* current=&path_map(_robot_pose_image.x(), _robot_pose_image.y());
    while (current&& current->parent && current->parent!=current) {
      PathMapCell* parent=current->parent;
      path.push_back(Eigen::Vector2i(current->r, current->c));
      current = current->parent;
    }
  }

  void NAOqiPlanner::recoveryPlan(){
    std::cerr << ">>>>>>>>>>> Recovery Plan: Computing path without obstacles <<<<<<<<<<" << std::endl;
    /*
    std::chrono::steady_clock::time_point time_path_start = std::chrono::steady_clock::now();
    _path_calculator.setMaxCost(_max_cost-1);
    _path_calculator.setCostMap(_cost_image_backup);
    _path_calculator.setOutputPathMap(_path_map_backup);
    Vector2iVector goals;
    goals.push_back(_goal);
    _path_calculator.goals() = goals;
    _path_calculator.compute();
    std::chrono::steady_clock::time_point time_path_end = std::chrono::steady_clock::now();

    std::cerr << "PathCalculator: "
	      << std::chrono::duration_cast<std::chrono::milliseconds>(time_path_end - time_path_start).count() << " ms" << std::endl;
    */

    _path.clear();
    // Filling path
    PathMapCell* current=&_path_map_backup(_robot_pose_image.x(), _robot_pose_image.y());
    std::cerr << " current: " << current <<  " current->parent: " << current->parent << std::endl;
    //if current->parent is null at this point We could change current with the closest cell that has a parent

    while (current && current->parent && current->parent!=current) {
      PathMapCell* parent=current->parent;

      PathMapCell* cell_path=&_path_map(current->r, current->c);
      if (cell_path->cost == getMaxCost()){
	//reached first obstacle in the path to goal

	std::cerr << "Cell cost: " << cell_path->cost << " parent: " << parent << " current: " << current <<  std::endl;
	break;
      }else{
      	_path.push_back(Eigen::Vector2i(current->r, current->c));
	current = current->parent;
      }
    }

    if (!_path.size()){
      std::cerr << ">>>>>>>>>>> Recovery Plan Failed <<<<<<<<<<" << std::endl;

      //We could try to relocalize at this point:
      recoveryRelocalize();
      
    }else {
      std::cerr << ">>>>>>>>>>> Found Path of size: " << _path.size() << std::endl;
      /*
      //removing some cells from the path in front of the obstacle
      float obstacle_distance_threshold = 0.7;
      int num_cells = obstacle_distance_threshold * _map_inverse_resolution;
      if (_path.size()>num_cells){
	int new_size = _path.size()-num_cells;	
	_path.resize(new_size);
      } else
	_path.clear();

      std::cerr << ">>>>>>>>>>> Final Path of size: " << _path.size() << std::endl;
      */

    }
  }

  void NAOqiPlanner::computeControlToWaypoint(float& v, float& w) {
    if (!_path.size()){
      v = 0.0;
      w = 0.0;
      
      _prev_v = v;
      _prev_w = w;
      return;
    }

    // Next waypoint naive computation.
    float nextwp_distance = 1.0; //meters
    int num_cells = nextwp_distance * _map_inverse_resolution;

    Eigen::Vector2i nextwp;
    bool isLastWp = false;
    if (_path.size() > num_cells)
      nextwp = _path[num_cells];
    else{
      nextwp = _path[_path.size()-1];
      isLastWp = true;
    }

    Eigen::Vector2f nextwp_world = grid2world(nextwp);
    Eigen::Vector2f robot_pose_xy(_robot_pose.x(), _robot_pose.y());
    Eigen::Vector2f distance_wp = nextwp_world - robot_pose_xy;
    float angle_wp = normalize(atan2(distance_wp.y(),distance_wp.x()) - _robot_pose.z());
    float goal_distance_threshold = 0.3;
    if (distance_wp.norm() < goal_distance_threshold){
      if (isLastWp){
        std::cerr << ">>>>>>>>>>>>>>>>>>>Arrived to last wp" << std::endl;
        v = 0.0;
        w = 0.0;

        _prev_v = v;
        _prev_w = w;

	Eigen::Vector2f goal_world = grid2world(_goal);
	Eigen::Vector2f distance_goal = goal_world - robot_pose_xy;
	if (distance_goal.norm() < goal_distance_threshold){
	
	  std::cerr << ">>>>>>>>>>>>>>>>>>>Arrived to goal " << std::endl;
	  
	  cancelGoal();
	  setState(GoalReached);
	}
	return;
      }
    }else {
      std::cerr << "Distance to wp: " << distance_wp.norm() << std::endl;
      std::cerr << "Angle to wp: " << angle_wp << std::endl;
    }

    float force = 1.5;
    Eigen::Vector2f F(force*cos(angle_wp), force*sin(angle_wp));

    //Constants definition
    float f = 1.0;
    float b = 1.975;
    float k_i = 0.95;
    float h = 2.05;
    float T = 0.2;
    
    //limiting force
    if (F.norm() > f){
      float angleF = atan2(F.y(), F.x());
      F.x() = cos(angleF)*f;
      F.y() = sin(angleF)*f;
    }

    std::cerr << "Force: " << F.transpose() << std::endl;
    
    v = (F.x() * T + _prev_v) / (1 + 2*b*T);
    std::cerr << "V: " << v << std::endl;
    
    if (v < 0)
      v = 0;
    w = (k_i *h * F.y() * T + _prev_w) / (1 + 2*b*k_i*T);
    std::cerr << "W: " << w << std::endl;
    if (fabs(w) < 1e-4)
      w = 0;

    // Switch to rotation-only behaviour
    float max_angle_goal = 0.3;
    if (fabs(angle_wp) > max_angle_goal)
      v = 0;
    

    _prev_v = v;
    _prev_w = w;

  }


  
  void NAOqiPlanner::publishPath() {
    if (_have_goal){
      IntVector path_vector;
      if (_path.size()){
	path_vector.resize(_path.size()*2);
	for (size_t i=0; i<_path.size(); i++){
	  path_vector[2*i] = _path[i].x();
	  path_vector[2*i+1] = _path[i].y();
	}
	
	_memory_service.call<void>("insertData", "NAOqiPlanner/Path", path_vector);
	_memory_service.call<void>("raiseEvent", "NAOqiPlanner/Status", "PathFound");
	std::cerr << "PAthsize: " << _path.size() * _map_resolution << std::endl;
	FloatVector exec_status;
	exec_status.push_back(_path.size() * _map_resolution);
	Eigen::Vector2f goal_world = grid2world(_goal);
	Eigen::Vector2f robot_pose_xy(_robot_pose.x(), _robot_pose.y());
	Eigen::Vector2f distance_goal = goal_world - robot_pose_xy;
	float angle_goal = normalize(atan2(distance_goal.y(),distance_goal.x()) - _robot_pose.z());



	exec_status.push_back(angle_goal);

	_memory_service.call<void>("raiseEvent", "NAOqiPlanner/ExecutionStatus",  exec_status);
      }else {
	FloatVector exec_status;
	Eigen::Vector2f goal_world = grid2world(_goal);
	Eigen::Vector2f robot_pose_xy(_robot_pose.x(), _robot_pose.y());
	Eigen::Vector2f distance_goal = goal_world - robot_pose_xy;
	float angle_goal = normalize(atan2(distance_goal.y(),distance_goal.x()) - _robot_pose.z());



	exec_status.push_back(distance_goal.norm());
	exec_status.push_back(angle_goal);

	_memory_service.call<void>("insertData", "NAOqiPlanner/ExecutionStatus",  exec_status);




	_memory_service.call<void>("raiseEvent", "NAOqiPlanner/Status", "PathNotFound");
      }
    }
  }

  void NAOqiPlanner::publishState(){
    if (_state==GoalReached){
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/Status", "GoalReached");
      _state = WaitingForGoal;
      return;
    }

    if (!_have_goal && _state!=WaitingForGoal){
      _state = WaitingForGoal;
      _memory_service.call<void>("raiseEvent", "NAOqiPlanner/Status", "WaitingForGoal");
      return;
    }
    
    if (_have_goal) {
      if (_state==GoalReceived){
	_memory_service.call<void>("raiseEvent", "NAOqiPlanner/Status", "GoalReceived");
      }
      
      if (_path.size() && _state!=PathFound){
	_memory_service.call<void>("raiseEvent", "NAOqiPlanner/Status", "PathFound");
	_state = PathFound;
	return;
      }
      
      if (!_path.size() && _state!=PathNotFound){
	_memory_service.call<void>("raiseEvent", "NAOqiPlanner/Status", "PathNotFound");
	_state = PathNotFound;
	return;
      }
      
    }
    
    
  }

}
