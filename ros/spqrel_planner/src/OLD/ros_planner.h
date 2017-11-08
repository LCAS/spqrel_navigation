#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sys/time.h>
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_srvs/Empty.h"

#include <actionlib/server/simple_action_server.h>
#include "move_base_msgs/MoveBaseAction.h"

#include "dynamic_reconfigure/server.h"

#include "planner.h"


namespace spqrel_navigation {

using namespace std;

/**
     class that implements a ROS planning node.
   */
class ROSPlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! initializes the localizer on the node handle provided
    //! if the listener is not given, it will create a transform
    //! listener on its own
    ROSPlanner(ros::NodeHandle& nh, tf::TransformListener* listener=0);

    //! set ROS parameters
    void setROSParams();

    //! init stuff
    void init();
    void initEmptyMap();

    //! quit stuff
    void quit();

    //!setters fot common parameters
    inline const std::string& baseFrameId() const { return _base_frame_id;}
    inline void setBaseFrameId(const std::string fid) {_base_frame_id=fid;}
    inline const std::string& globalFrameId() const { return _global_frame_id;}
    inline void setGlobalFrameId(const std::string gid) {_global_frame_id=gid;}
    inline const std::string& laserTopic() const {return _laser_topic;}
    inline void setCommandVelTopic(const std::string top) {_command_vel_topic=top;}
    inline const std::string& commandVelTopic() const { return _command_vel_topic;}
    inline void setMapServiceId(const std::string mid) {_map_service_id=mid;}

    //! sets/gets the maximum range of a laser, ranges faerther than this are ignored
    inline float forcedMaxRange() const {return _forced_max_range;}
    inline void setForcedMaxRange( float mr) {_forced_max_range=mr;}
    //! endpoints that are closer between each other than this threshold are suppressed
    inline float squaredEndpointDistance() const {return _squared_endpoint_distance;}
    inline void setSquaredEndpointDistance(float sd) { _squared_endpoint_distance=sd;}

    inline int waitCicle(){return _wait_cicle;}
    inline void setWaitCicle(int wait_cicle_){_wait_cicle = wait_cicle_;}
    //! call this when all is in place, AND the map is loaded (either via direct setMap function or
    //! by calling the requestMap service)
    void subscribeCallbacks(const std::string& laser_topic="/base_scan");
    void executeCB(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);
    void preemptCB();

    //! requests a map via rpc
    void requestMap();

protected:
    
    tf::TransformListener* _listener; //< listener object uset do compute the position of the robot in the map

    std::string _laser_topic;   //< laser topic to be used for localization
    std::string _base_frame_id; //< frame of the robot
    std::string _global_frame_id;  //< frame of the map
    std::string _map_service_id;  //< name of map service
    std::string _map_topic;  //< name of map service
    std::string _command_vel_topic;  //< topic for the command vel
    std::string _temp_topic;  //< topic for ???
    std::string _path_topic;
    std::string _global_path_topic;
    std::string _nearest_object_topic;


    std::vector<double> _timers; //< holds the update time of the last n cycles
    size_t _last_timer_slot; // the current update cycle

    float _forced_max_range; //< max range to use in localization
    float _squared_endpoint_distance; //< max distance betweem endpoints in the laser. The ones that are closer are suppressed
    float _translational_gain;
    float _rotational_gain;
    float _approaching_translational_gain;
    float _approaching_rotational_gain;
    float _max_tv;
    float _max_rv;
    float _max_acc_tv;
    float _max_acc_rv;
    float prev_tv;
    float prev_rv;
    int _wait_cicle;
    ros::Subscriber _laser_sub; //< subscriber for laser messages
    ros::Subscriber _goal_simple_sub; //< subscriber for the set_goal
    ros::Subscriber _cancel_sub; //< subscriber for the cancel_goal
    ros::Subscriber _map_sub; //< subscriber for the map
    ros::Time _last_observation_time;   //< stores the last time an observation has been processed,
    //<to keep the timestamps of the transforms consistent
    ros::Time _curr_controller_time;
    ros::Time _prev_controller_time;

    ros::NodeHandle& _nh;             //< global node handle
    ros::NodeHandle _private_nh;     //< private node handle
    ros::Publisher _cmd_vel_pub; // < to control the robot;
    ros::Publisher _nearest_pub;
    ros::Publisher _temp_pub;
    ros::Publisher _path_pub;
    ros::Publisher _global_path_pub;
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> _as;
    std::string _action_name;
    // create messages that are used to published feedback/result
    move_base_msgs::MoveBaseActionFeedback _feedback;
    move_base_msgs::MoveBaseActionResult _result;

    //Dynamic Reconfigure server and function to enable runtime parameters tuning
//    dynamic_reconfigure::Server<thin_navigation::ThinNavigationConfig> _server;
//    dynamic_reconfigure::Server<thin_navigation::ThinNavigationConfig>::CallbackType _function;

    
    Eigen::Vector3f _map_origin;    //< world coordinates of the upper left pixel

    //! converts a range scan into a vector of regularized endpoints
    //! @param endpoints: a vector2fvector containing the cartesian
    //!                   written by the method coordinares of the regularized endpoints
    //! @paeam laser_pose: pose of the laser on the robot
    //! @param msg: the laser message to be processed
    void rangesToEndpoints(Vector2fVector& endpoints,
                           const Eigen::Vector3f& laser_pose,
                           const sensor_msgs::LaserScan::ConstPtr& msg);


    //! handles a laser scan and updates the filter
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    //! handles the set-goal-estimate message from outside
    void setGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    //! handles the cancel-goal message from outside
    void setCancelCallback(const actionlib_msgs::GoalID& msg);

    //! handles dynamic_reconfigure messages
//    void dynamicReconfigureCallback(thin_navigation::ThinNavigationConfig &config, uint32_t level);


    void stopRobot();

    // called when the map is updated. causes some overhead
    void mapMessageCallback(const nav_msgs::OccupancyGrid& msg);

    //! computes the average computation time of the last n updates
    double cycleLatency() const;

    //! shows the gui stuff
    bool _use_gui;           //! handled internally
    bool _show_distance_map; //! if 1 in the gui shows the distance map. (to be toggled with "d")
    bool _force_redisplay;   //! if toggled to one forces the display and sends out all messages
    bool _set_goal;          //! set_goal mode (to be toggled with "s")
    //bool _have_goal;         //! the robot has a target goal: use _planner.haveGoal()
    boost::mutex _mtx_goal;
    //bool _have_wait;
    bool _have_map;           //! a map has been retrieved
    bool _new_map_available;  //! a new map is available
    int _wait;
    void executePath();

    Planner _planner;
};

}

