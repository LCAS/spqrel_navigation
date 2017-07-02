#include "multiecholaser_message_listener.h"
#include <srrg_messages/laser_message.h>
#include <srrg_types/defs.h>

namespace srrg_core_ros {

  using namespace std;
  using namespace srrg_core;

  MultiEchoLaserMessageListener::MultiEchoLaserMessageListener(ros::NodeHandle* nh, 
					     SensorMessageSorter* sorter_,
					     tf::TransformListener* listener_,
					     const std::string& odom_frame_id,
					     const std::string& base_link_frame_id) {
    _node_handle  = nh;
    _deque_length =  10;
    _imu_interpolator = 0;
    _listener = listener_;
    _sorter = sorter_;
    _base_link_frame_id = base_link_frame_id;
    _odom_frame_id = odom_frame_id;
    _laser_offset.setIdentity();
    _verbose = 0;
  }


  void MultiEchoLaserMessageListener::subscribe(const std::string& laser_topic) {
    _laser_topic = laser_topic;
    _laser_subscriber = _node_handle->subscribe<sensor_msgs::MultiEchoLaserScan>(_laser_topic, 1, &MultiEchoLaserMessageListener::laserCallback, this);
    cerr << "subscribing to topic: " << _laser_topic << endl; 
  }

  size_t getIndexBestIntensityEcho(const sensor_msgs::LaserEcho& intensity_echoes){
    size_t index = 0;
    float intensityBestEcho = 0.0;
    for (size_t i = 0; i< intensity_echoes.echoes.size(); i++){
      if (intensity_echoes.echoes[i] > intensityBestEcho){
	intensityBestEcho = intensity_echoes.echoes[i];
	index = i;
      }
    }

    return index;
  }


  void MultiEchoLaserMessageListener::laserCallback(const sensor_msgs::MultiEchoLaserScanConstPtr& in_msg) {
    _laser_deque.push_back(*in_msg);
    if (_laser_deque.size()<_deque_length){
      return;
    }
    sensor_msgs::MultiEchoLaserScan msg = _laser_deque.front();
    _laser_deque.pop_front();
    
    if (_listener) {
      tf::StampedTransform transform;
      try{
	_listener->waitForTransform(_odom_frame_id, 
				    _base_link_frame_id, 
				    msg.header.stamp, 
				    ros::Duration(0.1) );
	_listener->lookupTransform (_odom_frame_id, 
				    _base_link_frame_id, 
				    msg.header.stamp, 
				    transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
      }
      Eigen::Quaternionf q;
      q.x() = transform.getRotation().x();
      q.y() = transform.getRotation().y();
      q.z() = transform.getRotation().z();
      q.w() = transform.getRotation().w();
      _odom_pose.linear()=q.toRotationMatrix();
      _odom_pose.translation()=Eigen::Vector3f(transform.getOrigin().x(),
					       transform.getOrigin().y(),
					       transform.getOrigin().z());
    }      

    LaserMessage* scan_msg=new LaserMessage(_laser_topic, msg.header.frame_id,  msg.header.seq, msg.header.stamp.toSec());

    // getting offset
    if (_listener) {
      char buffer[1024];
      buffer[0] = 0;

      tf::StampedTransform transform;
      try{
	_listener->waitForTransform(_base_link_frame_id, msg.header.frame_id, 
				    msg.header.stamp, 
				    ros::Duration(0.1) );
	_listener->lookupTransform (_base_link_frame_id, msg.header.frame_id, 
				    msg.header.stamp, 
				    transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	return;
      }

      Eigen::Quaternionf q;
      q.x() = transform.getRotation().x();
      q.y() = transform.getRotation().y();
      q.z() = transform.getRotation().z();
      q.w() = transform.getRotation().w();
      _laser_offset.linear()=q.toRotationMatrix();
      _laser_offset.translation()=Eigen::Vector3f(transform.getOrigin().x(),
						  transform.getOrigin().y(),
						  transform.getOrigin().z());
    }
    scan_msg->setOffset(_laser_offset);


    
    // fill in stuff
    scan_msg->setMinAngle(msg.angle_min);
    scan_msg->setMaxAngle(msg.angle_max);
    scan_msg->setAngleIncrement(msg.angle_increment);
    scan_msg->setMinRange(msg.range_min);
    scan_msg->setMaxRange(msg.range_max);
    scan_msg->setTimeIncrement(msg.time_increment);
    scan_msg->setScanTime(msg.scan_time);

    std::vector<float> ranges;
    std::vector<float> intensities;
    ranges.resize(msg.ranges.size());
    intensities.resize(msg.intensities.size());
    if (ranges.size() == intensities.size()) {
      //laser provides both ranges and intensities
      //we take the echo with higher intensity

      for (size_t i = 0; i < msg.ranges.size(); i++){
	sensor_msgs::LaserEcho beam_ranges = msg.ranges[i];
	sensor_msgs::LaserEcho beam_intensities = msg.intensities[i];
	if (!beam_ranges.echoes.size()){
	  //if no range echo set ranges and intensities to 0
	  ranges[i]=0;
	  intensities[i]=0;
	  continue;
	}else {
	  //assume beam_ranges and beam_intensities have same number of echoes
	  size_t index_best_echo = getIndexBestIntensityEcho(beam_intensities);
	  assert(index_best_echo < beam_ranges.echoes.size() && "Error: range and intensity with different number of echoes");
	  float range_echo = beam_ranges.echoes[index_best_echo];
	  if ((range_echo < scan_msg->minRange()) || std::isnan(range_echo))
	    ranges[i]=0;
	  else if (range_echo > scan_msg->maxRange())
	    ranges[i]=scan_msg->maxRange();
	  else
	    ranges[i]=range_echo;
	  intensities[i] = beam_intensities.echoes[index_best_echo];
	}
      }
    }else {
      //laser only provides ranges or the sizes of the ranges and intensities differ
      //we simply take the first echo
      for (size_t i = 0; i < msg.ranges.size(); i++){
	sensor_msgs::LaserEcho beam_ranges = msg.ranges[i];
	if (!beam_ranges.echoes.size()){
	  ranges[i]=0;
	  continue;
	}
	float echo = beam_ranges.echoes[0];
	if ((echo < scan_msg->minRange()) || std::isnan(echo))
	  ranges[i]=0;
	else if (echo > scan_msg->maxRange())
	  ranges[i]=scan_msg->maxRange();
	else
	  ranges[i]=echo;
      }

      for (size_t i = 0; i < msg.intensities.size(); i++){
	sensor_msgs::LaserEcho beam_intensities = msg.intensities[i];
	if (!beam_intensities.echoes.size()){
	  intensities[i]=0;
	  continue;
	}else{
	  //we simply take the first echo
	  float echo = beam_intensities.echoes[0]; 
	  intensities[i]=echo;
	}
      }
    }
    
    scan_msg->setRanges(ranges);
    scan_msg->setIntensities(intensities);
  
    if (_imu_interpolator && _listener) {
      Eigen::Isometry3f imu = Eigen::Isometry3f::Identity();
      Eigen::Quaternionf q_imu;
      if (!_imu_interpolator->getOrientation(q_imu, msg.header.stamp.toSec()))
	return;
      imu.linear() = q_imu.toRotationMatrix();
      if (_verbose)
	cerr << "i";
      scan_msg->setImu(imu);
    } else if (_listener) {
      scan_msg->setOdometry(_odom_pose);
      if (_verbose)
	cerr << "L";
    } else {
      if (_verbose)
	cerr << "x";
    }
    _sorter->insertMessage(scan_msg);
  }



}
