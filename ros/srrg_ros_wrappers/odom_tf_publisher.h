#include <srrg_messages/static_transform_tree.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>

namespace srrg_core_ros {

  class OdomTfPublisher {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    OdomTfPublisher(ros::NodeHandle* nh_, 
		    tf::TransformBroadcaster* broadcaster_,
		    srrg_core::StaticTransformTree* tree);

    inline const std::string& odomFrameId() const {return _odom_frame_id;}
    inline const std::string& baseLinkFrameId() const {return _base_link_frame_id;}
    inline void setOdomFrameId(std::string& t)  { _odom_frame_id=t;}
    inline void setBaseLinkFrameId(std::string& t)  {  _base_link_frame_id=t;}

    void publishTransform (const Eigen::Isometry3f& my_t, 
			   const std::string& from,
			   const std::string& to,
			   const ros::Time& time);

    void callback(const nav_msgs::OdometryConstPtr& msg);
    void subscribe(const std::string& odom_topic);

  protected:
    srrg_core::StaticTransformTree* _tree;
    ros::Subscriber _odom_subscriber;
    ros::NodeHandle* _nh;
    tf::TransformBroadcaster* _broadcaster;
    std::string _base_link_frame_id;
    std::string _odom_frame_id;
  };

}
