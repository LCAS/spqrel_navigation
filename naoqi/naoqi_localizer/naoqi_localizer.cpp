#include "naoqi_localizer.h"

namespace naoqi_localizer {
  using namespace std;
  using namespace srrg_core;
  using namespace naoqi_sensor_utils;
  
  NAOqiLocalizer::NAOqiLocalizer(qi::SessionPtr session){
    _session = session;
    if (! _session)
      throw std::runtime_error("Error: No memory service provided");
  
    _restarted = true;
    _old_odom_pose.setZero();
  
    _forced_max_range = 10;
    _squared_endpoint_distance = 0.1*0.1;
    _show_distance_map = false;
    _force_redisplay=false;
    _set_pose = false;
    _use_gui=false;
    _use_d2l=false;
    _map_origin.setZero();
    _image_map_origin.setZero();

    _timers.resize(10);
    _last_timer_slot=0;
    _laser_pose.setZero();

    _cycle_time_ms = 200;

    Eigen::Matrix3f coeffs = Eigen::Matrix3f::Identity();
    coeffs = 0.01*coeffs;
    coeffs(1,1) = 0.0001;
    
    setNoiseCoeffs(coeffs);
    
  }

  void NAOqiLocalizer::initGUI(){
    _use_gui=true;
    cv::namedWindow( "pepper_localizer", 0 );
    cv::setMouseCallback( "pepper_localizer", &NAOqiLocalizer::onMouse, this );
    cerr << "GUI initialized" << endl;
  }

  void NAOqiLocalizer::subscribeServices(){
    _memory_service = _session->service("ALMemory");
    _motion_service = _session->service("ALMotion");

    _stop_thread=false;
    _servicesMonitorThread = std::thread(&NAOqiLocalizer::servicesMonitorThread, this);

    //subscribe to manual changes on pose
    _subscriber_pose = _memory_service.call<qi::AnyObject>("subscriber", "NAOqiLocalizer/SetPose");
    _signal_pose_id = _subscriber_pose.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&NAOqiLocalizer::onPoseChanged, this, _1))));


    std::cerr << "Localizer Services Monitor Thread launched." << std::endl;
  }

  void NAOqiLocalizer::unsubscribeServices(){
    _stop_thread=true;
    _servicesMonitorThread.join();
  }

  void NAOqiLocalizer::servicesMonitorThread() {
    while (!_stop_thread){
      std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
      qi::AnyValue result =  _motion_service.call<qi::AnyValue>("getRobotPosition", false);
      std::vector<float> robotpose = result.toList<float>();
      Eigen::Vector3f odom_pose(robotpose[0], robotpose[1], robotpose[2]);
    
      if (!_restarted) {
	Eigen::Vector3f control=t2v(v2t(_old_odom_pose).inverse()*v2t(odom_pose));
	predict(control);
      }
    
      _old_odom_pose=odom_pose;

      Vector2fVector endpoints;
      endpoints = getLaser(_memory_service); //Pepper's Laser points

      if (useD2L()) {
	Vector2fVector endpoints_d2l = getLaserFromDepth(_memory_service); //Depth2Laser points
	endpoints.insert(endpoints.end(), endpoints_d2l.begin(), endpoints_d2l.end()); 
      }
      
      bool updated = update(endpoints);
      computeStats();

      _force_redisplay|=updated;
    
      handleGUIDisplay();
    
      _restarted=false;
    
      handleGUIInput();

      Eigen::Isometry2f origin=v2t(_image_map_origin);
      Eigen::Isometry2f robot_transform=origin*v2t(_mean);
      Eigen::Vector3f robot_pose=t2v(robot_transform);

      std::cerr << "Robot pose: " << robot_pose.transpose() << std::endl;
      FloatVector robot_pose_vector = toFloatVector3f(robot_pose);

      _memory_service.call<void>("insertData", "NAOqiLocalizer/RobotPose", robot_pose_vector);
      _memory_service.call<void>("raiseEvent", "NAOqiLocalizer/RobotPose", robot_pose_vector);
      
      std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
      int cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
      std::cerr << "Cycle " << cycle_ms << " milliseconds" << std::endl;
      if (cycle_ms < _cycle_time_ms)
	usleep((_cycle_time_ms-cycle_ms)*1e3);
    }
    std::cout << "Localizer Monitor Thread finished." << std::endl;

  }

  void NAOqiLocalizer::onMouse( int event, int x, int y, int, void* v)
  {
    NAOqiLocalizer* n=reinterpret_cast<NAOqiLocalizer*>(v);
    if (!n->_set_pose)
      return;
    if( event == cv::EVENT_LBUTTONDOWN ) {
      std::cerr << "Left Click!" << std::endl;
      Eigen::Vector2f p=n->grid2world(Eigen::Vector2i(y,x));
      Eigen::Vector3f mean=n->mean();
      mean.x()=p.x();
      mean.y()=p.y();
      n->setPose(mean);
      n->_force_redisplay=true;
    }
    if( event == cv::EVENT_RBUTTONDOWN ) {
      std::cerr << "Right Click!" << std::endl;
      Eigen::Vector2f p=n->grid2world(Eigen::Vector2i(y,x));
      Eigen::Vector3f mean=n->mean();
      Eigen::Vector2f dp=p-mean.head<2>();
      float angle=atan2(dp.y(), dp.x());
      mean.z()=angle;
      n->setPose(mean);
      n->_force_redisplay=true;
    }
    
  }

  void NAOqiLocalizer::handleGUIInput(){
    if (! _use_gui)
      return;

    char key=cv::waitKey(10);
    switch(key) {
    case 'g': 
      cerr << "starting global localization" << endl;
      _restarted = true;
      startGlobal();
      break;
    case 'd': 
      _show_distance_map = ! _show_distance_map;
      cerr << "toggle distance map: " << _show_distance_map << endl;
      _force_redisplay=true;
      break;
    case 'r': 
      setParticleResetting(! particleResetting());
      cerr << "particle resetting = " << particleResetting();
      break;
    case 's': 
      _set_pose=!_set_pose;
      cerr << "set pose is " << _set_pose << endl;
      _force_redisplay=true;
      break;
    default:;
    }
  }

  void NAOqiLocalizer::handleGUIDisplay() {
    if (_use_gui && (_restarted || _force_redisplay) ) {
      RGBImage img;
      paintState(img, _show_distance_map);
      char buf[1024];
      sprintf(buf, " SetPose: %d", _set_pose);
      cv::putText(img, buf, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, img.rows*1e-3, cv::Scalar(200,0,200), 1);
      //sprintf(buf, " DynamicRestart: %d", particleResetting());
      //cv::putText(img, buf, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200,0,200), 1);
      //sprintf(buf, " Latency/cycle: %f [ms]", cycleLatency());
      //cv::putText(img, buf, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200,0,200), 1);
  
      cv::imshow("pepper_localizer", img);
    }  
    _force_redisplay=false;
   
  }

  double NAOqiLocalizer::cycleLatency() const {
    double acc=0;
    for (size_t i=0; i<_timers.size(); i++)
      acc+=_timers[i];
    return acc/_timers.size();
  }

  void NAOqiLocalizer::onPoseChanged(qi::AnyValue value) {
    std::cerr << ">>>>>>>>>>>>>>>>>>>>>>>>> POSE CHANGE CALLBACK <<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;

    srrg_core::FloatVector vpose = value.toList<float>();
    if (vpose.size() != 3){
      std::cerr << "not a valid pose" << std::endl;
      return;
    }

    Eigen::Vector3f new_pose(vpose[0],vpose[1],vpose[2]);
    printf("Setting pose (%s): %.3f %.3f %.3f",
	   qi::to_string(qi::SteadyClock::now()).c_str(),
	   new_pose.x(),
	   new_pose.y(),
	   new_pose.z());
  
    Eigen::Isometry2f inverse_origin=v2t(_image_map_origin).inverse();
    Eigen::Isometry2f global_pose=v2t(new_pose);
    Eigen::Vector3f map_pose=t2v(inverse_origin*global_pose);
    Eigen::Vector3f standard_deviations=Eigen::Vector3f(0.1, 0.1, 0.1);
    setPose(map_pose, standard_deviations);
    _restarted=true;
  }


  void NAOqiLocalizer::setInitialPose(float x, float y,float theta) {
    Eigen::Vector3f new_pose(x,y,theta);
    printf("Setting pose (%s): %.3f %.3f %.3f",
	   qi::to_string(qi::SteadyClock::now()).c_str(),
	   new_pose.x(),
	   new_pose.y(),
	   new_pose.z());
  
    Eigen::Isometry2f inverse_origin=v2t(_image_map_origin).inverse();
    Eigen::Isometry2f global_pose=v2t(new_pose);
    Eigen::Vector3f map_pose=t2v(inverse_origin*global_pose);
    setPose(map_pose);
    _restarted=true;
  }

  void NAOqiLocalizer::readMap(const std::string mapname){
    std::cerr << "Reading map" << mapname << std::endl;
  
    // reading map info
    SimpleYAMLParser parser;
    parser.load(mapname);
    std::cerr << "Dirname: " << dirname(strdup(mapname.c_str())) << std::endl;
  
    std::string map_image_name = parser.getValue("image");
    float res = parser.getValueAsFloat("resolution");
    float occupied_thresh = parser.getValueAsFloat("occupied_thresh");
    float free_thresh = parser.getValueAsFloat("free_thresh");
    int negate = parser.getValueAsInt("negate");
    Eigen::Vector3f new_origin = parser.getValueAsVector3f("origin");
    //Parser testing
    std::cerr << "MAP NAME: " << map_image_name << std::endl;
    std::cerr << "RESOLUTION: " << res << std::endl;
    std::cerr << "ORIGIN: " << new_origin.transpose() << std::endl;
    std::cerr << "NEGATE: " << negate << std::endl;
    std::cerr << "OCC THRESHOLD: " << occupied_thresh << std::endl;
    std::cerr << "FREE THRESHOLD: " << free_thresh << std::endl;
    std::cerr << "NON EXISTING KEY: " << parser.getValue("non_exists") << std::endl;

    std::string full_path_map_image = std::string(dirname(strdup(mapname.c_str())))+"/"+map_image_name;
    std::cerr << "Opening image" << full_path_map_image << std::endl;

    UnsignedCharImage map_image = cv::imread(full_path_map_image, CV_LOAD_IMAGE_GRAYSCALE);
    std::cerr << "Image read: (" << map_image.rows << "x" << map_image.cols << ")" << std::endl;

    int occupied_threshold = (1.0 - occupied_thresh) * 255;
    int free_threshold = (1.0 - free_thresh) * 255;
    //setMap(map_image, res, 10, 230);
    setMap(map_image, res, occupied_threshold, free_threshold);
    _map_origin=new_origin;

    // _map_origin: reference system bottom-left, X right, Y up  (values read from yaml file) (ROS convention)
    // _image_map_origin: image reference system top-left, X down, Y right (opencv convention)

    // transform from _map_origin to _image_map_origin
    Eigen::Vector3f map_to_image(0,map_image.rows*res,-M_PI/2);
    Eigen::Isometry2f tf = v2t(_map_origin) * v2t(map_to_image);
    _image_map_origin = t2v(tf);

    _restarted=true;
  }
}
