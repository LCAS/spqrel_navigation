#include "ros_localizer.h"
#include "srrg_ros_wrappers/ros_utils.h"
#include "srrg_system_utils/system_utils.h"

#include <iostream>
#include <chrono>
#include <ctime>
namespace srrg_localizer2d_ros{
  using namespace std;
  using namespace spqrel_navigation;
  using namespace srrg_core;
  using namespace srrg_core_ros;

  ROSLocalizer::ROSLocalizer(ros::NodeHandle& nh, tf::TransformListener* listener):
    _nh(nh){
    _listener = listener;
    if (! _listener)
      _listener = new tf::TransformListener;
    _restarted = true;
    _old_odom_pose.setZero();
    _odom_frame_id = "/odom";
    _base_frame_id = "/base_link";
    _forced_max_range = 25;
    _squared_endpoint_distance = 0.1*0.1;
    _show_distance_map = false;
    _force_redisplay=false;
    _set_pose = false;
    _use_gui=false;
    _map_origin.setZero();
    _broadcaster = 0;
    _timers.resize(10);
    _last_timer_slot=0;
    _laser_pose.setZero();
    _tf_timecheck = true;
    _cnt_not_updated = 0;
  }

  void ROSLocalizer::setTFTimeCheck(bool tf_timecheck) {
    _tf_timecheck = tf_timecheck;
  }

  void ROSLocalizer::initGUI(){
    _use_gui=true;
    cv::namedWindow( "srrg_localizer_2d", 0 );
    cv::setMouseCallback( "srrg_localizer_2d", &ROSLocalizer::onMouse, this );
    cerr << "GUI initialized" << endl;
  }

  void ROSLocalizer::rangesToEndpoints(srrg_core::Vector2fVector& endpoints,
					const Eigen::Vector3f& laser_pose,
					const sensor_msgs::LaserScan::ConstPtr& msg){

    // we have the transforms, we can start assembling the endpoints for the localizer
    // in doing that we do take care that no endpoint is closer than
    // squared_endpoint_distance from its predecessor
    // this avoids unnecessary computation when in crowded settings
    Eigen::Isometry2f laser_transform=srrg_core::v2t(laser_pose);
    endpoints.resize(msg->ranges.size());
    int k = 0;
    double angle=msg->angle_min-msg->angle_increment;
    float max_range = (msg->range_max<_forced_max_range) ? msg->range_max : _forced_max_range;
    Eigen::Vector2f last_endpoint(-1000, -1000);

    std::vector<float> ranges = msg->ranges;
    if (_inverted_laser){
      std::reverse(ranges.begin(), ranges.end());
    }

    for (size_t i=0; i<ranges.size(); i++){
      float r=ranges[i];
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

  void ROSLocalizer::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    _last_observation_time = msg->header.stamp;


    std::string error;

#if 0
    // laser pose on robot
    if (! _listener->waitForTransform (_base_frame_id,
				       msg->header.frame_id,
				       _last_observation_time,
				       ros::Duration(0.5),
				       ros::Duration(0.5), &error)) {
      //cerr << "ROSLocalizer: transform error from " << _base_frame_id << " to " << msg->header.frame_id << " : " << error << endl;
      //return;
    }
    tf::StampedTransform laser_pose_t;
    _listener->lookupTransform(_base_frame_id,
			       msg->header.frame_id,
			       ros::Time(0), // _last_observation_time,
			       laser_pose_t);
#endif

    // Get laser pose on robot
    tf::StampedTransform laser_pose_t;
    try {
      _listener->waitForTransform(_base_frame_id, msg->header.frame_id,
				 _last_observation_time,
				 ros::Duration(0.5), ros::Duration(0.5), &error);

      if (_tf_timecheck)
          _listener->lookupTransform(_base_frame_id, msg->header.frame_id,
				 _last_observation_time,
				 laser_pose_t);
      else
          _listener->lookupTransform(_base_frame_id, msg->header.frame_id,
				 ros::Time(0),
				 laser_pose_t);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Laser pose transform: %s",ex.what());
    }
    Eigen::Vector3f laser_pose = convertPose2D(laser_pose_t);


#if 0
    // odometry
    if (! _listener->waitForTransform (_odom_frame_id,
				       _base_frame_id,
				       _last_observation_time,
				       ros::Duration(0.5),
				       ros::Duration(0.5),
				       &error)) {
      //cerr << "ROSLocalizer: transform error from " << _odom_frame_id << " to " << _base_frame_id << " : " << error << endl;
      //return;
    }
    tf::StampedTransform odom_pose_t;
    _listener->lookupTransform(_odom_frame_id,
			       _base_frame_id,
			       ros::Time(0), // _last_observation_time,
			       odom_pose_t);
#endif

    // Get odometry pose
    tf::StampedTransform odom_pose_t;
    try {
      _listener->waitForTransform(_odom_frame_id, _base_frame_id,
				 _last_observation_time,
				 ros::Duration(0.5), ros::Duration(0.5), &error);
      if (_tf_timecheck)
        _listener->lookupTransform(_odom_frame_id, _base_frame_id,
				 _last_observation_time,
				 odom_pose_t);
      else
        _listener->lookupTransform(_odom_frame_id, _base_frame_id,
				 ros::Time(0),
				 odom_pose_t);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Robot odom pose transform: %s",ex.what());
    }
    Eigen::Vector3f odom_pose = convertPose2D(odom_pose_t);


#if 0
// LI what's for???
    if (! _listener->waitForTransform (_base_frame_id,
				       msg->header.frame_id,
				       _last_observation_time,
				       ros::Duration(0.5),
				       ros::Duration(0.5),
				       &error)) {
      cerr << "error: " << error << endl;
      return;
    }
#endif

    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    double t0=getTime();
    if (!_restarted) {
      Eigen::Vector3f control=t2v(v2t(_old_odom_pose).inverse()*v2t(odom_pose));
      predict(control);
    } else {
      _last_timer_slot=0;
    }

    _old_odom_pose=odom_pose;

    Vector2fVector endpoints(msg->ranges.size());
    rangesToEndpoints(endpoints, laser_pose, msg);
    bool updated = update(endpoints);

    // std::cerr << "  -- localizer updated: " << updated << std::endl;

    if (!updated) {
        if (_cnt_not_updated<20) { // force localization update for 20 cycles
            forceUpdate();
            _cnt_not_updated++;
        }
    }
    else
        _cnt_not_updated = 0;

    computeStats();
    publishPose();
    publishRanges();
    // printf("seq: %d, ts:%.9lf, [%f %f %f] [%f %f %f]\n",
    // 	   msg->header.seq,
    // 	   _last_observation_time.toSec(),
    // 	   odom_pose.x(),
    // 	   odom_pose.y(),
    // 	   odom_pose.z(),
    // 	   _mean.x(),
    // 	   _mean.y(),
    // 	   _mean.z());

    _force_redisplay|=updated;
    if (_restarted || _force_redisplay)
      publishParticles();
    double t1=getTime();
    _timers[_last_timer_slot]=t1-t0;
    _last_timer_slot++;
    if(_last_timer_slot>=_timers.size())
      _last_timer_slot=0;

    handleGUIDisplay();

    _restarted=false;

    handleGUIInput();
    std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
    int cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
    // std::cerr << "Cycle " << cycle_ms << " ms" << std::endl << std::endl;
  }

  void ROSLocalizer::handleGUIInput(){
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

  void ROSLocalizer::handleGUIDisplay() {
    if (_use_gui && (_restarted || _force_redisplay) ) {
      RGBImage img;
      paintState(img, _show_distance_map);
      char buf[1024];
      sprintf(buf, " SetPose: %d", _set_pose);
      cv::putText(img, buf, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200,0,200), 1);
      sprintf(buf, " DynamicRestart: %d", particleResetting());
      cv::putText(img, buf, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200,0,200), 1);
      sprintf(buf, " Latency/cycle: %f [ms]", cycleLatency());
      cv::putText(img, buf, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200,0,200), 1);


 // dynamic_restart: %d\n particles: %d\n inliers: %d",
 // 	      _set_pose, particleResetting(), (int) _particles.size(), 1000);
 //      std::string status_str(status);
 //      cv::putText(img, status_str, cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200,0,200), 1);
      cv::imshow("srrg_localizer_2d", img);
   }
   _force_redisplay=false;

  }

  void ROSLocalizer::onMouse( int event, int x, int y, int, void* v)
  {
    ROSLocalizer* n=reinterpret_cast<ROSLocalizer*>(v);
    if (!n->_set_pose)
      return;
    if( event == cv::EVENT_LBUTTONDOWN ) {
      Eigen::Vector2f p=n->grid2world(Eigen::Vector2i(y,x));
      Eigen::Vector3f mean=n->mean();
      mean.x()=p.x();
      mean.y()=p.y();
      n->setPose(mean);
      n->_force_redisplay=true;
    }
    if( event == cv::EVENT_RBUTTONDOWN ) {
      Eigen::Vector2f p=n->grid2world(Eigen::Vector2i(y,x));
      Eigen::Vector3f mean=n->mean();
      Eigen::Vector2f dp=p-mean.head<2>();
      float angle=atan2(dp.y(), dp.x());
      mean.z()=angle;
      n->setPose(mean);
      n->_force_redisplay=true;
    }

  }

  void ROSLocalizer::subscribeCallbacks(const std::string& laser_topic){
    _has_laser_pose=false;
    _laser_topic=laser_topic;
    _laser_sub=_nh.subscribe(_laser_topic, 10, &ROSLocalizer::laserCallback, this);
    _initial_pose_sub = _nh.subscribe("initialpose", 2, &ROSLocalizer::setPoseCallback, this);
    _global_loc_srv = _nh.advertiseService("global_localization",
					   &ROSLocalizer::globalLocalizationCallback,
					   this);

    _pose_pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
    _particlecloud_pub = _nh.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
    _ranges_pub = _nh.advertise<LocalizerRanges>("localizer_ranges", 2);
    _broadcaster = new tf::TransformBroadcaster;
  }

  void ROSLocalizer::requestMap() {
    // get map via RPC
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the map...");

    while(ros::ok() && !ros::service::call(_static_map_service, req, resp)){

      ROS_WARN_STREAM("Request for map " << _static_map_service << " failed; trying again...");
      ros::Duration d(0.5);
      d.sleep();
    }

    if (!ros::ok())
      ros::shutdown();
    else
      mapMessageCallback(resp.map);

  }

  void ROSLocalizer::mapMessageCallback(const::nav_msgs::OccupancyGrid& msg) {
    UnsignedCharImage map_image(msg.info.width, msg.info.height);
    int k=0;
    for(int c=0; c<map_image.cols; c++) {
      for(int r=0; r<map_image.rows; r++) {
	    int d=msg.data[k];
	    if (d<0) {
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
    setMap(map_image, msg.info.resolution, 10, 230);
    tf::Pose pose;
    tf::poseMsgToTF(msg.info.origin, pose);
    Eigen::Vector3f new_origin(pose.getOrigin().x(),
			       pose.getOrigin().y(),
			       getYaw(pose));
    cerr << "map origin: " << new_origin.transpose() << endl;
    _map_origin=new_origin;

    // _map_frame_id=msg.header.frame_id;

    cerr << "global/map frame id: [" << _global_frame_id << "]" << endl;
    _restarted=true;
  }

  bool ROSLocalizer::globalLocalizationCallback(std_srvs::Empty::Request& req,
					         std_srvs::Empty::Response& res) {
    startGlobal();
    _restarted=true;
    return true;
  }


  void ROSLocalizer::setPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    Eigen::Vector3f new_pose(pose.getOrigin().x(),
			     pose.getOrigin().y(),
			     getYaw(pose));

    ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
	     ros::Time::now().toSec(),
	     new_pose.x(),
	     new_pose.y(),
	     new_pose.z());
    Eigen::Isometry2f inverse_origin=v2t(_map_origin).inverse();
    Eigen::Isometry2f global_pose=v2t(new_pose);
    Eigen::Vector3f map_pose=t2v(inverse_origin*global_pose);
    setPose(map_pose);
    _restarted=true;
  }

  void ROSLocalizer::setInitialPose(float x, float y,float theta) {
    Eigen::Vector3f new_pose(x,y,theta);
    ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
         ros::Time::now().toSec(),
         new_pose.x(),
         new_pose.y(),
         new_pose.z());
    Eigen::Isometry2f inverse_origin=v2t(_map_origin).inverse();
    Eigen::Isometry2f global_pose=v2t(new_pose);
    Eigen::Vector3f map_pose=t2v(inverse_origin*global_pose);
    setPose(map_pose);
    _restarted=true;
  }

  void ROSLocalizer::publishParticles(){
    if (! _particlecloud_pub.getNumSubscribers())
      return;
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = _global_frame_id;
    cloud_msg.poses.resize(_particles.size());
    Eigen::Isometry2f origin=v2t(_map_origin);
    for(size_t i=0;i<particles().size();i++){
        Eigen::Vector3f p=t2v(origin*v2t(_particles[i]._pose));
	    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(p.z()),
				        tf::Vector3(p.x(),p.y(),0)), cloud_msg.poses[i]);
      }
    _particlecloud_pub.publish(cloud_msg);
  }

  void ROSLocalizer::publishPose(){
    geometry_msgs::PoseWithCovarianceStamped p;
    // Fill in the header
    p.header.frame_id = _global_frame_id;
    p.header.stamp=_last_observation_time;

    // clear the covariance
    for(size_t i=0; i<36; i++){
      p.pose.covariance[i]=0;
    }


    Eigen::Isometry2f origin=v2t(_map_origin);
    Eigen::Isometry2f robot_transform=origin*v2t(_mean);
    Eigen::Vector3f robot_pose=t2v(robot_transform);

    // Copy in the pose
    p.pose.pose.position.x = robot_pose.x();
    p.pose.pose.position.y = robot_pose.y();
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(robot_pose.z()),p.pose.pose.orientation);
    // Copy in the covariance, converting from 3-D to 6-D
    for(int i=0; i<2; i++) {
      for(int j=0; j<2; j++) {
	    p.pose.covariance[6*i+j] = _covariance(i,j);
      }
    }
    p.pose.covariance[6*5+5] = _covariance(2,2);

    _pose_pub.publish(p);

    Eigen::Vector3f delta_map_odom=t2v(robot_transform*v2t(_old_odom_pose).inverse());
    if (isNan(delta_map_odom) || isNan(_covariance)){
      cerr << "robot_transform.matrix() " << endl;
      cerr << robot_transform.matrix() << endl;

      cerr << "old_odom_pose.matrix() " << endl;
      cerr << _old_odom_pose.matrix() << endl;

      cerr << "covariance" << endl;
      cerr << _covariance << endl;

      throw std::runtime_error("Dude, you are trying to send a nan transform");
    }
    tf::Transform tmp_tf(tf::createQuaternionFromYaw(delta_map_odom.z()),
			 tf::Vector3(delta_map_odom.x(),
				     delta_map_odom.y(),
				     0.0));

    tf::StampedTransform map_to_odom(tmp_tf,
				     _last_observation_time,
				     _global_frame_id, _odom_frame_id);

    _broadcaster->sendTransform(map_to_odom);


  }

  double ROSLocalizer::cycleLatency() const {
    double acc=0;
    for (size_t i=0; i<_timers.size(); i++)
      acc+=_timers[i];
    return acc/_timers.size();
  }

  void ROSLocalizer::publishRanges() {
    LocalizerRanges localizer_ranges;
    localizer_ranges.header.stamp=_last_observation_time;
    localizer_ranges.header.frame_id=_base_frame_id;
    localizer_ranges.x.resize(_last_endpoints.size());
    localizer_ranges.y.resize(_last_endpoints.size());
    localizer_ranges.dist.resize(_last_endpoints.size());
    for (size_t i=0; i<_last_endpoints.size(); i++){
      localizer_ranges.x[i]=_last_endpoints[i].x();
      localizer_ranges.y[i]=_last_endpoints[i].y();
      localizer_ranges.dist[i]=_endpoint_distances[i];
    }
    _ranges_pub.publish(localizer_ranges);
  }
}
