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
#include <srrg_core_ros/SphericalDepthImage.h>

namespace srrg_core_ros {

  class SphericalImageMessageListener{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SphericalImageMessageListener(ros::NodeHandle* nh,
				  srrg_core::SensorMessageSorter* sorter_,
				  tf::TransformListener* listener_ = 0,
				  const std::string& odom_frame_id="",
				  const std::string& base_link_frame_id="");

    inline ros::NodeHandle* nodeHandle() { return _node_handle; }

    inline void setImuInterpolator(ImuInterpolator* interpolator) {_imu_interpolator = interpolator; }

    void subscribe(const std::string& _topic, int num_messages=100);

    void imageCallback(const srrg_core_ros::SphericalDepthImageConstPtr& in_msg);

    inline  const std::string& topic() const { return _topic; }

    inline bool verbose() const { return _verbose; }
    inline void setVerbose(bool v) {_verbose = v;}

  protected:
    srrg_core::SensorMessageSorter* _sorter;
    Eigen::Vector4f _K;
    ros::NodeHandle* _node_handle;
    ros::Subscriber _subscriber;
    std::string _topic;
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
