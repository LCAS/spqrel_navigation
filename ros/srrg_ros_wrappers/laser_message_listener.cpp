#include "laser_message_listener.h"
#include <srrg_messages/laser_message.h>
#include <srrg_types/defs.h>

namespace srrg_core_ros {

  using namespace std;
  using namespace srrg_core;


  LaserMessageListener::LaserMessageListener(ros::NodeHandle* nh, 
					     SensorMessageSorter* sorter_,
					     tf::TransformListener* listener_,
					     const std::string& odom_frame_id,
					     const std::string& base_link_frame_id) {
    _node_handle  = nh;
    _deque_length =  5;
    _imu_interpolator = 0;
    _listener = listener_;
    _sorter = sorter_;
    _base_link_frame_id = base_link_frame_id;
    _odom_frame_id = odom_frame_id;
    _laser_offset.setIdentity();
    _verbose = 0;
  }


  void LaserMessageListener::subscribe(const std::string& laser_topic) {
    _laser_topic = laser_topic;
    _laser_subscriber = _node_handle->subscribe<sensor_msgs::LaserScan>(_laser_topic, 1, &LaserMessageListener::laserCallback, this);
    cerr << "subscribing to topic: " << _laser_topic << endl; 
  }


  void LaserMessageListener::laserCallback(const sensor_msgs::LaserScanConstPtr& in_msg) {
    _laser_deque.push_back(*in_msg);
    if (_laser_deque.size()<_deque_length){
      return;
    }
    sensor_msgs::LaserScan msg = _laser_deque.front();
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
    ranges.resize(msg.ranges.size());
    for (size_t i = 0; i < msg.ranges.size(); i++){
      if ((msg.ranges[i] < msg.range_min) || std::isnan(msg.ranges[i]))
	ranges[i]=0;
      else if (msg.ranges[i] > msg.range_max)
	ranges[i]=msg.range_max;
      else
	ranges[i]=msg.ranges[i];
    }
    scan_msg->setRanges(ranges);

    std::vector<float> intensities;
    intensities.resize(msg.intensities.size());
    for (size_t i = 0; i < msg.intensities.size(); i++)
      intensities[i]=msg.intensities[i];
    scan_msg->setIntensities(intensities);
  
    if (_imu_interpolator && _listener) {
      Eigen::Isometry3f imu = Eigen::Isometry3f::Identity();
      Eigen::Quaternionf q_imu;
      if (!_imu_interpolator->getOrientation(q_imu, msg.header.stamp.toSec()))
	return;
      imu.linear() = q_imu.toRotationMatrix();
      if (_verbose)
	cerr << "[li]";
      scan_msg->setImu(imu);
    } else if (_listener) {
      scan_msg->setOdometry(_odom_pose);
      if (_verbose)
	cerr << "[lo]";
    } else {
      if (_verbose)
	cerr << "[lx]";
    }
    _sorter->insertMessage(scan_msg);
  }



}
