#include "imu_interpolator.h"


namespace srrg_core_ros {
  using namespace std;
  using namespace Eigen;


  ImuInterpolator::ImuInterpolator(ros::NodeHandle* nh_) {
    _time_window = 30;
    _nh = nh_;
  }

  void ImuInterpolator::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    _imu_queue.insert(make_pair( msg->header.stamp.toSec(), *msg));
    ImuQueue::iterator lb = _imu_queue.lower_bound(msg->header.stamp.toSec()-_time_window);
    if (lb!=_imu_queue.end()){
      _imu_queue.erase(_imu_queue.begin(), lb);
    }
  }

  bool ImuInterpolator::getOrientation(Eigen::Quaternionf& q, double ts){
    ImuQueue::iterator lb = _imu_queue.lower_bound(ts);
    ImuQueue::iterator ub = _imu_queue.upper_bound(ts);
    if (_imu_queue.size()) {
      double t_min = _imu_queue.begin()->first;
      double t_max = _imu_queue.rbegin()->first;
    }
    if (lb==_imu_queue.end() || ub==_imu_queue.end())
      return false;
    double lt = lb->second.header.stamp.toSec();
    Eigen::Quaternionf lq;
    lq.x() = lb->second.orientation.x;
    lq.y() = lb->second.orientation.y;
    lq.z() = lb->second.orientation.z;
    lq.w() = lb->second.orientation.w;

    double ut = ub->second.header.stamp.toSec();
    Eigen::Quaternionf uq;
    uq.x() = lb->second.orientation.x;
    uq.y() = lb->second.orientation.y;
    uq.z() = lb->second.orientation.z;
    uq.w() = lb->second.orientation.w;

    // do a slerp to recover the orientation
    Eigen::Quaternionf dq=lq.inverse()*uq;
    Eigen::AngleAxisf aa(dq);
    double fraction = (ts-lt)/(ut-lt);
    aa.angle()=aa.angle()*fraction;
    q=lq*aa;
    return true;
  }

  void ImuInterpolator::subscribe(const std::string& imu_topic) {
    _imu_subscriber = _nh->subscribe<sensor_msgs::Imu>(imu_topic, 1, &ImuInterpolator::imuCallback, this);
  }

}
