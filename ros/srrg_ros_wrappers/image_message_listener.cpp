#include "image_message_listener.h"
#include <srrg_messages/pinhole_image_message.h>
#include <srrg_types/defs.h>

#include <sensor_msgs/image_encodings.h>

namespace srrg_core_ros {

  using namespace std;
  using namespace srrg_core;

  ImageMessageListener::ImageMessageListener(ros::NodeHandle* nh, 
					     image_transport::ImageTransport* image_transport, 
					     SensorMessageSorter* sorter_,
					     tf::TransformListener* listener_,
					     const std::string& odom_frame_id,
					     const std::string& base_link_frame_id) {
    _node_handle  = nh;
    _image_transport = image_transport;
    _depth_scale = 1e-3;
    _deque_length =  10;
    _imu_interpolator = 0;
    _listener = listener_;
    _sorter = sorter_;
    _base_link_frame_id = base_link_frame_id;
    _odom_frame_id = odom_frame_id;
    _camera_offset.setIdentity();
    _verbose = 0;
  }


  void ImageMessageListener::subscribe(const std::string& image_topic, int num_messages) {
    _has_camera_matrix = false;
    _image_topic = image_topic;

    int lastindex = _image_topic.find_last_of("/"); 
    string rawname = _image_topic.substr(0, lastindex); 
    _camera_info_topic = rawname + "/camera_info";
    _image_subscriber = _image_transport->subscribe(_image_topic, num_messages, &ImageMessageListener::imageCallback, this);
    _camera_info_subscriber = _node_handle->subscribe<sensor_msgs::CameraInfo>(_camera_info_topic,
                                                                               100,
                                                                               &ImageMessageListener::cameraInfoCallback,
                                                                               this);
    cerr << "subscribing to topics: " << _image_topic << ", " << _camera_info_topic << endl; 
  }


  void ImageMessageListener::imageCallback(const sensor_msgs::ImageConstPtr& in_msg) {
    _image_deque.push_back(*in_msg);
    if (_image_deque.size()<_deque_length){
      return;
    }
    sensor_msgs::Image msg = _image_deque.front();
    _image_deque.pop_front();
    
    
    if (_listener) {
      tf::StampedTransform transform;
      try{
	_listener->waitForTransform(_odom_frame_id, 
				    _base_link_frame_id, 
				    msg.header.stamp, 
				    ros::Duration(0.5) );
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


    cv_bridge::CvImageConstPtr bridge;
    bridge = cv_bridge::toCvCopy(msg,msg.encoding);
  
    if (! _has_camera_matrix){
      cerr << "received: " << _image_topic << " waiting for camera matrix" << endl;
      return;
    }

    if (bridge->image.type()==CV_8UC4) {
      cv::Mat temp;
      cv::cvtColor(bridge->image, temp, CV_BGRA2RGB);
      _cv_image=temp.clone();
    } else {
      _cv_image = bridge->image.clone();
    }
  
    PinholeImageMessage* img_msg = new PinholeImageMessage(_image_topic, msg.header.frame_id,  msg.header.seq, msg.header.stamp.toSec());
    img_msg->setImage(_cv_image);
    img_msg->setOffset(_camera_offset);
    img_msg->setCameraMatrix(_K);
    img_msg->setDepthScale(_depth_scale);
  
    if (_imu_interpolator && _listener) {
      Eigen::Isometry3f imu = Eigen::Isometry3f::Identity();
      Eigen::Quaternionf q_imu;
      if (!_imu_interpolator->getOrientation(q_imu, msg.header.stamp.toSec()))
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


  void ImageMessageListener::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    cerr << "got camera matrix for topic: " << _camera_info_topic << endl;
    if (_listener) {
      char buffer[1024];
      buffer[0] = 0;

      tf::StampedTransform transform;
      try{
	_listener->waitForTransform(_base_link_frame_id, msg->header.frame_id, 
				    msg->header.stamp, 
				    ros::Duration(0.5) );
	_listener->lookupTransform (_base_link_frame_id, msg->header.frame_id, 
				    msg->header.stamp, 
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
      _camera_offset.linear()=q.toRotationMatrix();
      _camera_offset.translation()=Eigen::Vector3f(transform.getOrigin().x(),
						   transform.getOrigin().y(),
						   transform.getOrigin().z());
    }      
    int i;
    for (int r=0; r<3; r++)
      for (int c=0; c<3; c++, i++)
	_K(r,c) = msg->K[i];
    _K.row(2) << 0,0,1;
    _has_camera_matrix = true;
    _camera_info_subscriber.shutdown();
  }

}
