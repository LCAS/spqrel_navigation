#pragma once
#include <srrg_types/defs.h>
#include <srrg_messages/sensor_message_sorter.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "imu_interpolator.h"

#include <limits>
#include <deque>
#include <queue>

namespace srrg_core_ros {

  class ImageMessageListener{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ImageMessageListener(ros::NodeHandle* nh, 
			 image_transport::ImageTransport* image_transport, 
			 srrg_core::SensorMessageSorter* sorter_,
			 tf::TransformListener* listener_ = 0,
			 const std::string& odom_frame_id="",
			 const std::string& base_link_frame_id="");

    inline image_transport::ImageTransport* imageTransport() { return _image_transport; }

    inline ros::NodeHandle* nodeHandle() { return _node_handle; }

    inline void setImuInterpolator(ImuInterpolator* interpolator) {_imu_interpolator = interpolator; }

    void subscribe(const std::string& image_topic, int num_messages=100);

    void imageCallback(const sensor_msgs::ImageConstPtr& in_msg);

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

    inline  const std::string& imageTopic() const { return _image_topic; }

    inline bool verbose() const { return _verbose; }
    inline void setVerbose(bool v) {_verbose = v;}

    inline int dequeLength() const {return _deque_length;}
    inline void setDequeLength(int dl) {_deque_length=dl;}

  protected:
    srrg_core::SensorMessageSorter* _sorter;
    Eigen::Matrix3f _K;
    Eigen::Matrix3f _out_K;
    bool _has_camera_matrix;
    ros::NodeHandle* _node_handle;
    image_transport::Subscriber _image_subscriber;
    ros::Subscriber _camera_info_subscriber;
    image_transport::ImageTransport* _image_transport;
    std::string _image_topic;
    std::string _camera_info_topic;
    int _deque_length;
    std::deque<sensor_msgs::Image> _image_deque;
    cv::Mat _cv_image;
    float _depth_scale;
    Eigen::Isometry3f _camera_offset;
    ImuInterpolator* _imu_interpolator;
    tf::TransformListener* _listener;
    std::string _base_link_frame_id;
    std::string _odom_frame_id;
    Eigen::Isometry3f _odom_pose;
    bool _verbose;
  };
}
