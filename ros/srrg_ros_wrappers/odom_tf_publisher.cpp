#include "odom_tf_publisher.h"

namespace srrg_core_ros {

  using namespace std;
  using namespace srrg_core;

  OdomTfPublisher::OdomTfPublisher(ros::NodeHandle* nh_, 
				   tf::TransformBroadcaster* broadcaster_,
				   StaticTransformTree* tree) {
    _nh = nh_;
    _broadcaster = broadcaster_;
    _base_link_frame_id = "/base_link";
    _odom_frame_id = "/odom";
    _tree = tree;
  }

  void OdomTfPublisher::publishTransform (const Eigen::Isometry3f& my_t, 
					  const std::string& from,
					  const std::string& to,
					  const ros::Time& time) {

    Eigen::Quaternionf q(my_t.linear());
    Eigen::Vector3f t=my_t.translation();
    tf::Transform published_transform;
    published_transform.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
    published_transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    _broadcaster->sendTransform(tf::StampedTransform(published_transform, 
						     time, 
						     from, to));
  }

  void OdomTfPublisher::callback(const nav_msgs::OdometryConstPtr& msg) {
    Eigen::Quaternionf orientation;
    orientation.x() = msg->pose.pose.orientation.x;
    orientation.y() = msg->pose.pose.orientation.y;
    orientation.z() = msg->pose.pose.orientation.z;
    orientation.w() = msg->pose.pose.orientation.w;
    Eigen::Isometry3f odom_transform;
    odom_transform.linear() = orientation.toRotationMatrix();
    odom_transform.translation().x() = msg->pose.pose.position.x;
    odom_transform.translation().y() = msg->pose.pose.position.y;
    odom_transform.translation().z() = msg->pose.pose.position.z;
    
    
    publishTransform(odom_transform,_odom_frame_id, _base_link_frame_id, msg->header.stamp);

    for (StaticTransformTree::StringTransformMap::const_iterator it = _tree->tree().begin();
	 it !=_tree->tree().end(); it++){
      
      const StaticTransformMessage& t=*it->second;
      publishTransform(t.transform(), t.fromFrameId(), t.toFrameId(), msg->header.stamp);
    }
  }

  void OdomTfPublisher::subscribe(const std::string& odom_topic) {
    _odom_subscriber = _nh->subscribe<nav_msgs::Odometry>(odom_topic, 1, &OdomTfPublisher::callback, this);
  }
  

}
