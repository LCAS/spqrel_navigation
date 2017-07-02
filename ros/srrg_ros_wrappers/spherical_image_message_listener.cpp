#include "spherical_image_message_listener.h"
#include <srrg_messages/spherical_image_message.h>
#include <srrg_types/defs.h>

namespace srrg_core_ros {

  using namespace std;
  using namespace srrg_core;

  SphericalImageMessageListener::SphericalImageMessageListener(ros::NodeHandle* nh, 
					     SensorMessageSorter* sorter_,
					     tf::TransformListener* listener_,
					     const std::string& odom_frame_id,
					     const std::string& base_link_frame_id) {
    _node_handle  = nh;
    _depth_scale = 1e-3;
    _imu_interpolator = 0;
    _listener = listener_;
    _sorter = sorter_;
    _base_link_frame_id = base_link_frame_id;
    _odom_frame_id = odom_frame_id;
    _camera_offset.setIdentity();
    _verbose = 0;
  }


  void SphericalImageMessageListener::subscribe(const std::string& topic_, int num_messages) {
    _topic = topic_;

    _subscriber = _node_handle->subscribe<SphericalDepthImage>(_topic, 100, &SphericalImageMessageListener::imageCallback, this);
    cerr << "subscribing to topic: " << _topic  << endl; 
  }


  void SphericalImageMessageListener::imageCallback(const SphericalDepthImageConstPtr& msg) {
   
    
    if (_listener) {
      // get the odometry
      tf::StampedTransform transform;
      try{
	_listener->waitForTransform(_odom_frame_id, 
				    _base_link_frame_id, 
				    msg->header.stamp, 
				    ros::Duration(0.5) );
	_listener->lookupTransform (_odom_frame_id, 
				    _base_link_frame_id, 
				    msg->header.stamp, 
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

      // get the sensor offset
      try{
	_listener->waitForTransform(_base_link_frame_id, 
				    msg->header.frame_id, 
				    msg->header.stamp, 
				    ros::Duration(0.5) );
	_listener->lookupTransform (_base_link_frame_id, 
				    msg->header.frame_id, 
				    msg->header.stamp, 
				    transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
      }
      q.x() = transform.getRotation().x();
      q.y() = transform.getRotation().y();
      q.z() = transform.getRotation().z();
      q.w() = transform.getRotation().w();
      _camera_offset.linear()=q.toRotationMatrix();
      _camera_offset.translation()=Eigen::Vector3f(transform.getOrigin().x(),
						   transform.getOrigin().y(),
						   transform.getOrigin().z());

    }      


    cv_bridge::CvImageConstPtr bridge;
    bridge = cv_bridge::toCvCopy(msg->image,msg->image.encoding);

    _cv_image = bridge->image.clone();
 
    Eigen::Vector4f K;
    K[0]=msg->horizontal_fov;
    K[1]=msg->vertical_fov;
    K[2]=msg->image.width/msg->horizontal_fov;
    K[3]=msg->image.height/msg->vertical_fov;

    SphericalImageMessage* img_msg = new SphericalImageMessage(_topic, msg->header.frame_id,  msg->header.seq, msg->header.stamp.toSec());
    img_msg->setImage(_cv_image);
    img_msg->setOffset(_camera_offset);
    img_msg->setCameraMatrix(K);
    img_msg->setDepthScale(_depth_scale);
  
    if (_imu_interpolator && _listener) {
      Eigen::Isometry3f imu = Eigen::Isometry3f::Identity();
      Eigen::Quaternionf q_imu;
      if (!_imu_interpolator->getOrientation(q_imu, msg->header.stamp.toSec()))
	return;
      imu.linear() = q_imu.toRotationMatrix();
      if (_verbose)
	cerr << "i";
      img_msg->setImu(imu);
    } else if (_listener) {
      img_msg->setOdometry(_odom_pose);
      if (_verbose)
	cerr << "o";
    } else {
      if (_verbose)
	cerr << "x";
    }
    _sorter->insertMessage(img_msg);
  }


}
