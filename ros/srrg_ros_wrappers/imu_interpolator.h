#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>

namespace srrg_core_ros {

  struct ImuInterpolator {
    ImuInterpolator(ros::NodeHandle* nh_);

    typedef std::map<double, sensor_msgs::Imu> ImuQueue;

    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    bool getOrientation(Eigen::Quaternionf& q, double ts);

    void subscribe(const std::string& imu_topic);

    ImuQueue _imu_queue;
    float _time_window;
    ros::Subscriber _imu_subscriber;
    ros::NodeHandle* _nh;
  };

}
