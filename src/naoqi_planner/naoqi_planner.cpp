#include "naoqi_planner.h"



namespace naoqi_planner {
  using namespace naoqi_sensor_utils;
  
  NAOqiPlanner::NAOqiPlanner(qi::SessionPtr session){
    _session = session;
    if (! _session)
      throw std::runtime_error("Error: No memory service provided");

    _use_gui = false;
    _what_to_show = Map;

    _restart = true;

    _have_goal = false;
    _goal = Eigen::Vector2i(0,0);
    _robot_pose = Eigen::Vector3f(0.,0.,0.);

    _prev_v = 0.0; _prev_w = 0.0;
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
    }
    
  }

  void NAOqiPlanner::handleGUIInput(){
    if (! _use_gui)
      return;

    char key=cv::waitKey(10);
    switch(key) {
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
    case 'r':
      std::cerr << "Resetting" << std::endl;
      _restart = true;
      _have_goal = false;
      //Removing obstacles
      {
	int occ_threshold = (1.0 - _occ_threshold) * 255;
	int free_threshold = (1.0 - _free_threshold) * 255;
	grayMap2indices(_indices_image, _map_image, occ_threshold, free_threshold);
      }
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
      
      //cv::namedWindow("indices_img", 0);
      //cv::imshow("indices_img", _indices_image);

      // Drawing goal
      if (_have_goal)
	cv::circle(shown_image, cv::Point(_goal.y(), _goal.x()), 3, cv::Scalar(0.0f));

      // Drawing current pose
      cv::rectangle(shown_image,
		    cv::Point(_robot_pose_image.y()+2, _robot_pose_image.x()-2),
		    cv::Point(_robot_pose_image.y()-2, _robot_pose_image.x()+2),
		    cv::Scalar(0.0f));


      //Draw path
      if (_robot_pose_image.x()>=0 && _have_goal){
	PathMapCell* current=&_path_map(_robot_pose_image.x(), _robot_pose_image.y());
	while (current&& current->parent && current->parent!=current) {
	  PathMapCell* parent=current->parent;
	  cv::line(shown_image,
		   cv::Point(current->c, current->r),
		   cv::Point(parent->c, parent->r),
		   cv::Scalar(0.0f));
	  current = current->parent;
	}
      }

      
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

    int occ_threshold = (1.0 - _occ_threshold) * 255;
    int free_threshold = (1.0 - _free_threshold) * 255;
    grayMap2indices(_indices_image, _map_image, occ_threshold, free_threshold);
  }


  void NAOqiPlanner::subscribeServices(){
    qi::AnyObject memory_service = _session->service("ALMemory");
    qi::AnyObject motion_service = _session->service("ALMotion");
    
    _stop_thread=false;
    _servicesMonitorThread = std::thread(&NAOqiPlanner::servicesMonitorThread, this, memory_service, motion_service);
    std::cerr << "Planner Services Monitor Thread launched." << std::endl;
  }

  void NAOqiPlanner::unsubscribeServices(){
    _stop_thread=true;
    _servicesMonitorThread.join();
  }
  
  void NAOqiPlanner::servicesMonitorThread(qi::AnyObject memory_service, qi::AnyObject motion_service) {
    while (!_stop_thread){
      qi::SteadyClock::time_point time_start = qi::SteadyClock::now();

      //get robot localization
      qi::AnyValue value = memory_service.call<qi::AnyValue>("getData", "NAOqiLocalizer/RobotPose");
      srrg_core::FloatVector robot_pose_floatvector = value.toList<float>();
      _robot_pose = srrg_core::fromFloatVector3f(robot_pose_floatvector);
      std::cerr << "Robot pose map: " << _robot_pose.transpose() << std::endl;

      Eigen::Isometry2f robot_pose_transform=_map_origin_transform_inverse*v2t(_robot_pose);
      Eigen::Vector3f robot_pose_vector=t2v(robot_pose_transform);

      _robot_pose_image = world2grid(Eigen::Vector2f(robot_pose_vector.x(), robot_pose_vector.y()));
      
      
      //get laser
      Vector2fVector laserpoints = getLaser(memory_service);
      Vector2iVector obstacle_points;
      for (size_t i=0; i<laserpoints.size(); i++){
	Eigen::Vector2f lp=robot_pose_transform* laserpoints[i];
	int r = lp.x()*_map_inverse_resolution;
	int c = lp.y()*_map_inverse_resolution;
	if (! _distance_map.inside(r,c))
	  continue;
	obstacle_points.push_back(Eigen::Vector2i(r,c));
      }
      
      //here I'll do something map + loc + laser + goal -> path
      
      if (_restart){
	_dmap_calculator.setMaxDistance(_safety_region/_map_resolution);
	_dmap_calculator.setIndicesImage(_indices_image);
	_dmap_calculator.setOutputPathMap(_distance_map);
	_dmap_calculator.init();
	_restart = false;
      }
      _dmap_calculator.setPoints(obstacle_points);

      
      _dmap_calculator.compute();
      _distance_image = _dmap_calculator.distanceImage()*_map_resolution;

      distances2cost(_cost_image, _distance_image, _robot_radius, _safety_region, _min_cost, _max_cost);

      if (_have_goal){
	_path_calculator.setMaxCost(_max_cost-1);
	_path_calculator.setCostMap(_cost_image);
	_path_calculator.setOutputPathMap(_path_map);
	Vector2iVector goals;
	goals.push_back(_goal);
	_path_calculator.goals() = goals;
	_path_calculator.compute();

	_path.clear();


	// Filling path
	std::ofstream ofs ("path.dat", std::ofstream::out);
	PathMapCell* current=&_path_map(_robot_pose_image.x(), _robot_pose_image.y());
	while (current&& current->parent && current->parent!=current) {
	  PathMapCell* parent=current->parent;
	  _path.push_back(Eigen::Vector2i(current->r, current->c));
	  ofs << current->r << current->c << std::endl;
	  
	  current = current->parent;
	}

	float linear_vel, angular_vel;
	computeControlToWaypoint(linear_vel, angular_vel);
	//apply vels
	std::cerr << "Applying vels: " << linear_vel  << " " << angular_vel << std::endl;
	motion_service.call<void>("move", linear_vel,0,angular_vel);

	
      } //else nothing to compute
      
      handleGUIDisplay();
    
      handleGUIInput();
    

      
      qi::SteadyClock::time_point time_end = qi::SteadyClock::now();
      qi::MilliSeconds ms = boost::chrono::duration_cast<qi::MilliSeconds>(time_end - time_start);
      std::cerr << "Cycle " << qi::to_string(ms) << std::endl;
    
      usleep(200000);
    }
    std::cout << "Planner Monitor Thread finished." << std::endl;

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
    Eigen::Vector2f distance_goal = nextwp_world - robot_pose_xy;
    float angle_goal = normalize(atan2(distance_goal.y(),distance_goal.x()) - _robot_pose.z());
    float goal_distance_threshold = 0.3;
    if (distance_goal.norm() < goal_distance_threshold){
      std::cerr << ">>>>>>>>>>Arrived to goal: " << nextwp.transpose() << std::endl;
      if (isLastWp){
	std::cerr << ">>>>>>>>>>Arrived to last goal" << std::endl;
	v = 0.0;
	w = 0.0;
	
	_prev_v = v;
	_prev_w = w;
	_have_goal = false;
	return;
      }
    }else {
      std::cerr << "Distance to goal: " << distance_goal.norm() << std::endl;
      std::cerr << "Angle to goal: " << angle_goal << std::endl;
    }

    float force = 1.5;
    Eigen::Vector2f F(force*cos(angle_goal), force*sin(angle_goal));

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

    v = (F.x() * T + _prev_v) / (1 + 2*b*T);
    if (v < 0)
      v = 0;
    w = (k_i *h * F.y() * T + _prev_w) / (1 + 2*b*k_i*T);
    if (w < 1e-4)
      w = 0;

    _prev_v = v;
    _prev_w = w;

  }


  

  

}
