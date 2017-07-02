#include "ros_utils.h"

namespace srrg_core_ros {
  
  Eigen::Isometry3f pose2eigen(const geometry_msgs::Pose& p){
    Eigen::Isometry3f iso;
    iso.translation().x()=p.position.x;
    iso.translation().y()=p.position.y;
    iso.translation().z()=p.position.z;
    Eigen::Quaternionf q;
    q.x()= p.orientation.x;
    q.y()= p.orientation.y;
    q.z()= p.orientation.z;
    q.w()= p.orientation.w;
    iso.linear()=q.toRotationMatrix();
    return iso;
  }

  geometry_msgs::Pose eigen2pose(const Eigen::Isometry3f& t){
    Eigen::Quaternionf q(t.linear());
    geometry_msgs::Pose p;
    p.position.x = t.translation().x();
    p.position.y = t.translation().y();
    p.position.z = t.translation().z();

    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
  }

  Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p){
    Eigen::Isometry3f iso;
    iso.translation().x()=p.getOrigin().x();
    iso.translation().y()=p.getOrigin().y();
    iso.translation().z()=p.getOrigin().z();
    Eigen::Quaternionf q;
    tf::Quaternion tq = p.getRotation();
    q.x()= tq.x();
    q.y()= tq.y();
    q.z()= tq.z();
    q.w()= tq.w();
    iso.linear()=q.toRotationMatrix();
    return iso;
  }

  tf::Transform eigen2tfTransform(const Eigen::Isometry3f& T){
    Eigen::Quaternionf q(T.linear());
    Eigen::Vector3f t=T.translation();
    tf::Transform tft;
    tft.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
    tft.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    return tft;
  }


  Eigen::Vector3f convertPose2D(const tf::StampedTransform& t) {
    double yaw,pitch,roll;
    tf::Matrix3x3 mat =  t.getBasis();
    mat.getRPY(roll, pitch, yaw);
    return Eigen::Vector3f(t.getOrigin().x(), t.getOrigin().y(), yaw);
  }

  double getYaw(tf::Pose& t) {
    double yaw, pitch, roll;
    t.getBasis().getEulerYPR(yaw,pitch,roll);
    return yaw;
  }

}
