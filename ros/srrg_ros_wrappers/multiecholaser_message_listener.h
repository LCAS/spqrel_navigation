#pragma once
#include <srrg_types/defs.h>
#include <srrg_messages/sensor_message_sorter.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <tf/transform_listener.h>
#include <iostream>
#include "imu_interpolator.h"

#include <limits>
#include <deque>
#include <queue>

namespace srrg_core_ros {

  class MultiEchoLaserMessageListener{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    MultiEchoLaserMessageListener(ros::NodeHandle* nh, 
			 srrg_core::SensorMessageSorter* sorter_,
			 tf::TransformListener* listener_ = 0,
			 const std::string& odom_frame_id="",
			 const std::string& base_link_frame_id="");

    
    inline ros::NodeHandle* nodeHandle() { return _node_handle; }

    inline void setImuInterpolator(ImuInterpolator* interpolator) {_imu_interpolator = interpolator; }

    void subscribe(const std::string& laser_topic);

    void laserCallback(const sensor_msgs::MultiEchoLaserScanConstPtr& in_msg);
    
    inline  const std::string& laserTopic() const { return _laser_topic; }

    inline bool verbose() const { return _verbose; }
    inline void setVerbose(bool v) {_verbose = v;}

  protected:
    srrg_core::SensorMessageSorter* _sorter;
    ros::NodeHandle* _node_handle;
    ros::Subscriber _laser_subscriber;
    std::string _laser_topic;
    int _deque_length;
    std::deque<sensor_msgs::MultiEchoLaserScan> _laser_deque;
    Eigen::Isometry3f _laser_offset;
    ImuInterpolator* _imu_interpolator;
    tf::TransformListener* _listener;
    std::string _base_link_frame_id;
    std::string _odom_frame_id;
    Eigen::Isometry3f _odom_pose;
    bool _verbose;
  };
}
