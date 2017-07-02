#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/tf.h>

namespace srrg_core_ros {

  Eigen::Isometry3f pose2eigen(const geometry_msgs::Pose& p);
  geometry_msgs::Pose eigen2pose(const Eigen::Isometry3f& t);

  Eigen::Isometry3f transform2eigen(const geometry_msgs::Transform& p);
  geometry_msgs::Transform eigen2transform(const Eigen::Isometry3f& t);

  Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p);
  tf::Transform eigen2tfTransform(const Eigen::Isometry3f& t);

  Eigen::Vector3f convertPose2D(const tf::StampedTransform& t) ;
  double getYaw(tf::Pose& t) ;

}
