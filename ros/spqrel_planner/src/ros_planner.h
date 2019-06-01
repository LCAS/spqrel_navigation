#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


#include "srrg_planner2d/planner.h"
#include "srrg_ros_wrappers/ros_utils.h"

namespace spqrel_navigation {

  using namespace srrg_core_ros;
  using namespace srrg_planner;

  class ROSPlanner: public Planner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ROSPlanner(ros::NodeHandle& nh, tf::TransformListener* listener=0);

    void getParams();
    void requestMap();

    void stopRobot();
    void applyVelocities();

  protected:

    ros::NodeHandle& _nh;
    tf::TransformListener* _listener;
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> _as;
    
    //Topics
    std::string _laser_topic;
    std::string _goal_topic;
    std::string _map_topic;
    std::string _cancel_topic;
    std::string _reset_topic;
    std::string _cmd_vel_topic;
    std::string _path_topic;
    std::string _status_topic;
    std::string _static_map_service;

    //Subscribers
    ros::Subscriber _laserwpose_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _map_sub;
    ros::Subscriber _cancel_sub;
    ros::Subscriber _reset_sub;
    void subscribeLaserWithPose();
    void subscribeGoal();
    void subscribeMap();
    void subscribeCancel();
    void subscribeReset();
    virtual void stopSubscribers(){};    
    
    //Callbacks
    void laserWithPoseCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void moveBaseGoalCallback();
    void mapCallback(const::nav_msgs::OccupancyGrid& msg);
    void cancelCallback(const actionlib_msgs::GoalID& msg);
    void resetCallback(const std_msgs::Bool& msg);

    //Publishers
    ros::Publisher _cmd_vel_pub;
    ros::Publisher _path_pub, _status_pub;
    void startCmdVelPublisher();
    void startPathPublisher();
    void startResultPublisher(){};
    void startStatusPublisher();
    void publishPath();
    virtual void publishState();
    virtual void publishResult(PlannerResult result);
    virtual void publishExecutionStatus(){};
    virtual void stopPublishers(){};

    //Frames
    std::string _base_frame_id;
    std::string _global_frame_id;

    //Laser management
    float _forced_max_range;
    float _squared_endpoint_distance;
    void rangesToEndpoints(Vector2fVector& endpoints,
			   const Eigen::Vector3f& laser_pose,
			   const sensor_msgs::LaserScan::ConstPtr& msg);

    // Flags
    bool _map_received;
    bool _tf_timecheck;

  };
}
