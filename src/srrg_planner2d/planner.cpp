#include "planner.h"

namespace srrg_planner {
  Planner::Planner() {
    _max_cost = 100.0;
    _min_cost = 20.0;
    _robot_radius = 0.3;
    _safety_region = 1.0;

    _use_gui = false;
    _what_to_show = Map;
    _verbose = false;
    
    _have_goal = false;
    _have_goal_with_angle = false;
    _goal = Eigen::Vector3f(0,0,0);
    _goal_image = Eigen::Vector3f(0,0,0);
    _goal_pixel = Eigen::Vector2i(0,0);

    _robot_pose = Eigen::Vector3f(0,0,0);
    _robot_pose_image = Eigen::Vector3f(0,0,0);
    _robot_pose_pixel = Eigen::Vector2i(0,0);

    _laser_points.clear();
    _dyn_map.clearPoints();

    _move_enabled = true;
    _velocities = Eigen::Vector2f::Zero();
    
    _on_recovery_time = false;
    _recovery_waiting_time = 10;
    _recovery_obstacle_distance = 1.0;

  }

  void Planner::cancelGoal() {
    _have_goal = false;
    _path.clear();
    _velocities = Eigen::Vector2f::Zero();
    _motion_controller.resetVelocities();
    _on_recovery_time = false;
    stopRobot();
  }
  
  void Planner::reset(){
    cancelGoal();

    //Restarting Path Map variables and removing obstacles
    restartPathMap();    
  }

  void Planner::restartPathMap(){
    _dyn_map.clearPoints();
    
    int occ_threshold = (1.0 - _occ_threshold) * 255;
    int free_threshold = (1.0 - _free_threshold) * 255;
    grayMap2indices(_indices_image, _map_image, occ_threshold, free_threshold);


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
		   _distance_image, _robot_radius, _safety_region, _min_cost, _max_cost);
    distances2cost(_cost_image,
		   _distance_image, _robot_radius, _safety_region, _min_cost, _max_cost);

  }
  
  void Planner::readMap(const std::string mapname){
    std::cerr << "Reading map" << mapname << std::endl;
  
    // reading map info
    SimpleYAMLParser parser;
    parser.load(mapname);
    std::cerr << "Dirname: " << dirname(strdup(mapname.c_str())) << std::endl;
    
    std::string map_image_name = parser.getValue("image");
    float map_resolution = parser.getValueAsFloat("resolution");
    float occ_threshold = parser.getValueAsFloat("occupied_thresh");
    float free_threshold = parser.getValueAsFloat("free_thresh");
    Eigen::Vector3f map_origin = parser.getValueAsVector3f("origin");
    
    std::string full_path_map_image = std::string(dirname(strdup(mapname.c_str())))+"/"+map_image_name;
    std::cerr << "Opening image" << full_path_map_image << std::endl;
  
    UnsignedCharImage map_image = cv::imread(full_path_map_image, CV_LOAD_IMAGE_GRAYSCALE);

    setMapFromImage(map_image, map_resolution, map_origin, occ_threshold, free_threshold);
  }



  void Planner::setMapFromImage(const UnsignedCharImage& map_image, const float map_resolution,
				const Eigen::Vector3f& map_origin, const float occ_threshold, const float free_threshold) {

    _mtx_display.lock();

    _map_image = map_image.clone();
    _map_resolution = map_resolution;
    _map_inverse_resolution = 1./_map_resolution;
    _map_origin = map_origin;
    _map_origin_transform_inverse = v2t(_map_origin).inverse();
    _occ_threshold = occ_threshold;
    _free_threshold = free_threshold;

    // _map_origin: reference system bottom-left, X right, Y up  (values read from yaml file) (ROS convention)
    // _image_map_origin: image reference system top-left, X down, Y right (opencv convention)

    // transform from _map_origin to _image_map_origin
    Eigen::Vector3f map_to_image(0, _map_image.rows*_map_resolution, -M_PI/2);
    Eigen::Isometry2f tf = v2t(_map_origin) * v2t(map_to_image);
    _image_map_origin = t2v(tf);
    _image_map_origin_transform_inverse = v2t(_image_map_origin).inverse();;

    restartPathMap();

    std::cerr << "Setting map: \n"
	      << "  Size: " << map_image.rows << "x" << map_image.cols << std::endl
	      << "  Resolution: " << _map_resolution << std::endl
	      << "  Occ threshold: " << _occ_threshold << std::endl
	      << "  Free threshold: " << _free_threshold << std::endl
	      << "  Map origin: " << _map_origin.transpose() << std::endl;
      
    _mtx_display.unlock();

  }

  void Planner::initGUI(){
    _use_gui=true;
    cv::namedWindow( "spqrel_planner", 0 );
    cv::setMouseCallback( "spqrel_planner", &Planner::onMouse, this );
    handleGUIDisplay();
    std::cerr << std::endl;
    std::cerr << "GUI initialized" << std::endl;
    std::cerr << "Use the following keys on the GUI:" << std::endl;
    std::cerr << "M: change to map view" << std::endl;
    std::cerr << "D: change to distance map view" << std::endl;
    std::cerr << "C: change to cost map view" << std::endl;
    std::cerr << "R: reset planner state (clear dynamic obstacles and current goal)" << std::endl;
    std::cerr << std::endl;
  }

  void Planner::onMouse( int event, int x, int y, int flags, void* v){
    Planner* n=reinterpret_cast<Planner*>(v);
  
    if (event == cv::EVENT_LBUTTONDOWN && ((flags & cv::EVENT_FLAG_CTRLKEY) != 0) ) {
      //std::cerr << "Left Click!" << std::endl;
      n->setGoalGUI(Eigen::Vector2i(y,x));
    }
  }

  void Planner::setGoalGUI(Eigen::Vector2i goal){
    _mtx_display.lock();
    //Goal given in pixels
    _goal_pixel = goal;
    //Transform from pixels to image [m]
    Eigen::Vector2f goal_image_xy = grid2world(goal);
    _goal_image = Eigen::Vector3f(goal_image_xy.x(), goal_image_xy.y(), 0);
    //Transform from image to map_origin
    Eigen::Isometry2f goal_transform = v2t(_image_map_origin) * v2t(_goal_image);

    Eigen::Vector2f goal2f;
    goal2f.x() = goal_transform.translation().x();
    goal2f.y() = goal_transform.translation().y();

    _mtx_display.unlock();

    setGoalXY(goal2f);
    std::cerr << "Setting goal: " << "   " << goal2f.transpose() << "   - pixel: " << _goal_pixel.transpose() << std::endl;
  }

  void Planner::setGoalXY(const Eigen::Vector2f& goal){
    //Set goal without angle
    _mtx_display.lock();
    _have_goal = true;
    _have_goal_with_angle = false;
    //Goal given wrt map_origin
    _goal.x() = goal.x();
    _goal.y() = goal.y();
    _goal.z() = 0;
    //From map_origin to image
    Eigen::Isometry2f goal_transform=_image_map_origin_transform_inverse*v2t(_goal);
    _goal_image = t2v(goal_transform); // image coordinates
    //From image to pixels
    _goal_pixel = world2grid(Eigen::Vector2f(_goal_image.x(), _goal_image.y()));  // pixel

    _mtx_display.unlock();
  }

  void Planner::setGoal(const Eigen::Vector3f& goal){
    // Goal in map coordinates

    _mtx_display.lock();
    _have_goal = true;
    _have_goal_with_angle = true;
    //Goal given wrt map_origin
    _goal = goal;
    //From map_origin to image
    Eigen::Isometry2f goal_transform=_image_map_origin_transform_inverse*v2t(_goal);
    _goal_image = t2v(goal_transform); // image coordinates
    //From image to pixels
    _goal_pixel = world2grid(Eigen::Vector2f(_goal_image.x(), _goal_image.y()));  // pixel

    _mtx_display.unlock();
  }

  void Planner::updateGoals(){
    _mtx_display.lock();
     //From map_origin to image
    Eigen::Isometry2f goal_transform=_image_map_origin_transform_inverse*v2t(_goal);
    _goal_image = t2v(goal_transform); // image coordinates
    //From image to pixels
    _goal_pixel = world2grid(Eigen::Vector2f(_goal_image.x(), _goal_image.y()));  // pixel
    _mtx_display.unlock();
  }
  
  void Planner::setRobotPose(const Eigen::Vector3f& robot_pose){
    _mtx_display.lock();
  
    _robot_pose=robot_pose;
  
    Eigen::Isometry2f robot_pose_transform = _image_map_origin_transform_inverse*v2t(robot_pose);
    _robot_pose_image = t2v(robot_pose_transform);
    _robot_pose_pixel = world2grid(Eigen::Vector2f(_robot_pose_image.x(), _robot_pose_image.y()));
  
    _mtx_display.unlock();
  }

  void Planner::setLaserPoints(const Vector2fVector& laser_points){
    _mtx_display.lock();

    _laser_points = laser_points;

    _mtx_display.unlock();
  }

  void Planner::handleGUI(){
    handleGUIInput();
    handleGUIDisplay();
  }
  
  void Planner::handleGUIInput(){
    char key=cv::waitKey(25);
    switch(key) {
    case 'h':
      std::cout << "m: map mode" << std::endl;
      std::cout << "d: distance map" << std::endl;
      std::cout << "c: cost map" << std::endl;
      std::cout << "p: enable/disable motion" << std::endl;
      std::cout << "r: reset distance/cost map and remove the goal" << std::endl
		<< "   (can be used for emergency stop)" << std::endl;
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
      setMoveEnabled(!_move_enabled);
      std::cerr << "Move enabled: " << _move_enabled << std::endl;
      break;
    case 'r':
      std::cerr << "Resetting" << std::endl;
      reset();
      break;
      /*case 'o':
      _collision_protection_desired = ! _collision_protection_desired;
      std::cerr << "External Collision Protection Desired: " << _collision_protection_desired << std::endl;*/
    default:;
    }
  }

  void Planner::handleGUIDisplay() {
    _mtx_display.lock();

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
      
    int robot_radius_pixel = _robot_radius*_map_inverse_resolution;
    // Drawing goal
    if (_have_goal){
      cv::circle(shown_image, cv::Point(_goal_pixel.y(), _goal_pixel.x()), robot_radius_pixel, cv::Scalar(0.0f));
      if (_have_goal_with_angle){
	Eigen::Vector2i goal_angle_pixel(cos(_goal_image.z())*robot_radius_pixel, sin(_goal_image.z())*robot_radius_pixel);
	cv::line(shown_image,
		 cv::Point(_goal_pixel.y(), _goal_pixel.x()),
		 cv::Point(_goal_pixel.y()+goal_angle_pixel.y(), _goal_pixel.x()+goal_angle_pixel.x()),
		 cv::Scalar(0.0f));
      }
    }

    // Drawing current pose
    cv::rectangle(shown_image,
		  cv::Point(_robot_pose_pixel.y()+robot_radius_pixel, _robot_pose_pixel.x()-robot_radius_pixel),
		  cv::Point(_robot_pose_pixel.y()-robot_radius_pixel, _robot_pose_pixel.x()+robot_radius_pixel),
		  cv::Scalar(0.0f));
    Eigen::Vector2i robot_pose_angle_pixel(cos(_robot_pose_image.z())*robot_radius_pixel, sin(_robot_pose_image.z())*robot_radius_pixel);
    cv::line(shown_image,
	     cv::Point(_robot_pose_pixel.y(), _robot_pose_pixel.x()),
	     cv::Point(_robot_pose_pixel.y()+robot_pose_angle_pixel.y(), _robot_pose_pixel.x()+robot_pose_angle_pixel.x()),
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
      /*
	if (_nominal_path.size()>1){
	for (size_t i=0; i<_nominal_path.size()-1; i++){

	Eigen::Vector2i cell_from = _nominal_path[i];
	Eigen::Vector2i cell_to   = _nominal_path[i+1];
	cv::line(shown_image,
	cv::Point(cell_from.y(), cell_from.x()),
	cv::Point(cell_to.y(), cell_to.x()),
	cv::Scalar(0.5f));
	  
	  
	}
	}*/
    }

    //Draw laser
    for (size_t i=0; i<_laser_points.size(); i++){
      Eigen::Vector2f lp=v2t(_robot_pose_image)* _laser_points[i];
      int r = lp.x()*_map_inverse_resolution;
      int c = lp.y()*_map_inverse_resolution;
      if (! _distance_map.inside(r,c))
	continue;
      cv::circle(shown_image, cv::Point(c, r), 3, cv::Scalar(1.0f));
    }

    
    char buf[1024];
    sprintf(buf, " MoveEnabled: %d", _move_enabled);
    cv::putText(shown_image, buf, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);
    /*
      sprintf(buf, " CollisionProtectionDesired: %d", _collision_protection_desired);
      cv::putText(shown_image, buf, cv::Point(20, 50+(int)shown_image.cols*0.03), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);
      sprintf(buf, " ExternalCollisionProtectionEnabled: %d", _collision_protection_enabled);
      cv::putText(shown_image, buf, cv::Point(20, 50+(int)shown_image.cols*0.06), cv::FONT_HERSHEY_SIMPLEX, shown_image.rows*1e-3, cv::Scalar(1.0f), 1);
    */
      
    cv::imshow("spqrel_planner", shown_image);

    _mtx_display.unlock();
    
    
  }
  
  void Planner::plannerStep(){
    if (!_have_goal) {
      status = "no_goal";
      return;
    }

    status = "running";

    std::chrono::steady_clock::time_point time_start, time_end, time_dmap_start, time_dmap_end, 
        time_path_start, time_path_end, time_other1_start, time_other1_end, time_other2_start, time_other2_end,
        time_control_start, time_control_end, time_vel_start, time_vel_end;


    time_start = std::chrono::steady_clock::now();              // TIME_START

    
    //Adding obstacles
    time_dmap_start = std::chrono::steady_clock::now();         // TIME_DMAP_START
    _distance_map.data()=_distance_map_backup;

    if (_laser_points.size()>0) {
	  
      _dyn_map.setMapResolution(_map_resolution);
      _dyn_map.setRobotPose(_robot_pose_image);
      _dyn_map.setCurrentPoints(_laser_points);
      _dyn_map.compute();
      Vector2iVector obstacle_points;
      _dyn_map.getOccupiedCells(obstacle_points);
    
      _dmap_calculator.setPoints(obstacle_points, _max_distance_map_index);
      _dmap_calculator.compute();
    
    }else{
      std::cerr << "WARNING: laser data not available." << std::endl;
      status = "no laser";
    }
  
    _distance_image = _dmap_calculator.distanceImage()*_map_resolution;
    distances2cost(_cost_image, _distance_image, _robot_radius, _safety_region, _min_cost, _max_cost);

    time_dmap_end = time_path_start = std::chrono::steady_clock::now();           // TIME_DMAP_END = TIME_PATH_START 


    computePath(_cost_image, _path_map, _goal_pixel, _obstacle_path);

    time_path_end = std::chrono::steady_clock::now();           // TIME_PATH_END = TIME_OTHER_START 

    _path = _obstacle_path;

    if (!_path.size()){

      time_other1_start = std::chrono::steady_clock::now();     // TIME_OTHER1_START

      if (_robot_pose_pixel == _goal_pixel){
	    // Path is zero because robot is on the goal
	    publishResult(GoalReached);
        status = "goal_reached";
	    cancelGoal();
      }else{
	    bool recovery_success = manageRecovery();
	    if (!recovery_success){
	      publishResult(Aborted);
          status = "aborted";
	      cancelGoal();	  
	    }
      }


      time_other1_end = std::chrono::steady_clock::now();       // TIME_OTHER1_END

    } else {


      time_control_start = std::chrono::steady_clock::now();     // TIME_CONTROL_START
     

      if (_on_recovery_time)
        _on_recovery_time = false; //Path was found after recoveryTime
      
      bool goal_reached = computeControlToWaypoint(_have_goal_with_angle);
      
      time_control_end = time_other2_start = std::chrono::steady_clock::now();     // TIME_CONTROL_END = TIME_OTHER2_START

      if (goal_reached){
        publishResult(GoalReached);
        status = "goal_reached";
        cancelGoal();
      } else {
        if (moveEnabled()) {

          time_vel_start = std::chrono::steady_clock::now();
          applyVelocities();
          time_vel_end = std::chrono::steady_clock::now();

        }
        else 
          _motion_controller.resetVelocities();
      }

      time_other2_end = std::chrono::steady_clock::now();       // TIME_OTHER2_END

    }

    time_end = std::chrono::steady_clock::now();                // TIME_END

    if (verbose()) {
        
        int cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count(); 

        if (cycle_ms>10) {
            std::cerr << "  -- DMapCalculator: "
	          << std::chrono::duration_cast<std::chrono::milliseconds>(time_dmap_end - time_dmap_start).count() << " ms" << std::endl;    

            std::cerr << "  -- ComputePath: "
	          << std::chrono::duration_cast<std::chrono::milliseconds>(time_path_end - time_path_start).count() << " ms" << std::endl; 

            std::cerr << "  -- Control: "
	          << std::chrono::duration_cast<std::chrono::milliseconds>(time_control_end - time_control_start).count() << " ms" << std::endl; 

            std::cerr << "  -- Velocity: "
	          << std::chrono::duration_cast<std::chrono::milliseconds>(time_vel_end - time_vel_start).count() << " ms" << std::endl; 

            std::cerr << "  -- Other stuff: "
	          << std::chrono::duration_cast<std::chrono::milliseconds>(time_other1_end - time_other1_start).count()  << " ms " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(time_other2_end - time_other2_start).count()  << " ms " 
              << std::endl; 

            std::cerr << "Total Cycle " << cycle_ms << " ms" << std::endl << std::endl; 
        } 
    }

  }

  void Planner::computePath(FloatImage& cost_map, PathMap& path_map, Eigen::Vector2i& goal, Vector2iVector &path){

    _path_calculator.setMaxCost(_max_cost-1);
    _path_calculator.setCostMap(cost_map);
    _path_calculator.setOutputPathMap(path_map);
    Vector2iVector goals;
    goals.push_back(goal);
    _path_calculator.goals() = goals;
    _path_calculator.compute();

    path.clear();
    // Filling path
    PathMapCell* current=&path_map(_robot_pose_pixel.x(), _robot_pose_pixel.y());
    while (current && current->parent && current->parent!=current) {
      PathMapCell* parent=current->parent;
      path.push_back(Eigen::Vector2i(current->r, current->c));
      current = current->parent;
    }
  }


  bool Planner::computeControlToWaypoint(bool goal_with_angle){
    
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

    bool goal_reached = false;
    if (isLastWp){
      if (goal_with_angle){
	// Giving (x, y, theta) to controller to arrive with the given robot orientation
	goal_reached = _motion_controller.computeVelocities(_robot_pose_image, _goal_image, _velocities);
      }
      else{
	// Giving (x, y) to controller. We do not care about the final robot orientation
	Eigen::Vector2f goal_image_xy(_goal_image.x(), _goal_image.y());
	goal_reached = _motion_controller.computeVelocities(_robot_pose_image, goal_image_xy, _velocities);
      }
    }
    else {
      // Giving (x, y) to controller since we still have a long way till reaching the goal
      Eigen::Vector2f nextwp_image_xy = grid2world(nextwp);
      goal_reached = _motion_controller.computeVelocities(_robot_pose_image, nextwp_image_xy, _velocities);
    }
  
    return goal_reached;
    
  }

  void Planner::setMoveEnabled(bool move_enabled){
    _move_enabled = move_enabled;
    if (!_move_enabled)
      stopRobot(); //Stop robot as soon as movement is disabled
  }

  void Planner::startSubscribers(){
    std::cerr << "Starting subscribers." << std::endl;
    
    subscribeLaserWithPose();
    subscribeGoal();
    subscribeMap();
    subscribeCancel();
    subscribeReset();
  }

  void Planner::startPublishers(){
    std::cerr << "Starting publishers." << std::endl;

    startCmdVelPublisher();
    startPathPublisher();
    startResultPublisher();
    startStatusPublisher();
  }

  void Planner::init(){
    std::cerr << "Starting Planner." << std::endl;
    
    startSubscribers();
    startPublishers();

    if (_use_gui)
      initGUI();    
    
  }

  void Planner::runOnce(){
    if (_have_goal)
      plannerStep();

    if (_use_gui)
      handleGUI();

    if (_path.size())
      publishPath();

    if (status!="") {
       publishState();
       // std::cerr << status << std::endl;
       if (status=="goal_reached" || status=="aborted")
          status = "";
    }
  }

  bool Planner::manageRecovery(){
    
    // if we can compute a path to approach the goal we follow it
    // else stop the robot and wait some time before aborting
    
    if (recoveryPath()){
      computeControlToWaypoint(false);
      applyVelocities();
      return true;
    }
    else {
      _motion_controller.resetVelocities();
      bool r = recoveryTime();
      stopRobot();
      return r;
    }
  }

  bool Planner::recoveryTime(){
    //Returns false if recovery failed (waiting time exceeded)
    if (!_on_recovery_time){
      //First call to recovery
      _on_recovery_time = true;
      //Start count time
      _recovery_time = std::chrono::steady_clock::now();
      return true;
    }
    else {
      std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();

      int time_waited = std::chrono::duration_cast<std::chrono::seconds>(current_time - _recovery_time).count();

      return time_waited <= _recovery_waiting_time;
    }
  }

  
  bool Planner::recoveryPath(){
    
    //Get nominal path without obstacles
    computePath(_cost_image_backup, _path_map_backup, _goal_pixel, _nominal_path);

    if (_nominal_path.size()){
      //Extract part of the nominal path up to the blocking obstacle
      _path.clear();
      Eigen::Vector2i cell;
      for (size_t i=0; i<_nominal_path.size(); i++){
	    cell = _nominal_path[i];
	    //std::cerr << _cost_image(cell.x(), cell.y()) << std::endl;
	    if (_cost_image(cell.x(), cell.y()) != maxCost())
	      _path.push_back(cell);
	    else
	      break;
      }

      Eigen::Vector2f obstacle_image = grid2world(cell);
      //Check if we can approach the obstacle
      float distance_goal = sqrt((_robot_pose_image.x()-obstacle_image.x())*(_robot_pose_image.x()-obstacle_image.x())+
				 (_robot_pose_image.y()-obstacle_image.y())*(_robot_pose_image.y()-obstacle_image.y()));

      if ( distance_goal > _recovery_obstacle_distance){
        return true;	
      }else {
        std::cerr << "Recovery plan failed: " << _nominal_path.size() << " - robot too close to obstacle" << std::endl;
        status = "recovery_too_close_to_obstacle";
        return false;
      }

    }
    else {
      std::cerr << "Recovery plan failed: " << _nominal_path.size() << " - path not found on empty space." << std::endl;
      status = "recovery_path_not_found";
      return false;
    }
    
  }
 
}
