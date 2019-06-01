#pragma once

#include <boost/thread/mutex.hpp>

#include <libgen.h> 

#include "srrg_path_map/path_map_utils.h"
#include "srrg_path_map/distance_map_path_search.h"
#include "srrg_path_map/dijkstra_path_search.h"

#include "yaml_parser/simple_yaml_parser.h"

#include "motion_controller.h"
#include "dynamic_map.h"

namespace srrg_planner {
  using namespace srrg_core;

  enum WhatToShow {Map, Distance, Cost};
  //enum State {WaitingForMap, WaitingForGoal, GoalAccepted, PathFound, PathNotFound, GoalReached};
  //enum Event {MapRecieved, GoalReceived};
  enum PlannerResult {GoalReached, Aborted};
  
  class Planner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //! Constructor
    Planner();

    // Path map parameters
    inline void setMaxCost(float max_cost) {_max_cost = max_cost;}
    inline float maxCost() const {return _max_cost;}
    inline void setMinCost(float min_cost) {_min_cost = min_cost;}
    inline float minCost() const {return _min_cost;}
    inline void setRobotRadius(float robot_radius) {_robot_radius = robot_radius;}
    inline float robotRadius() const {return _robot_radius;}
    inline void setSafetyRegion(float safety_region) {_safety_region = safety_region;}
    inline float safetyRegion() const {return _safety_region;}

    // Motion controller parameters
    inline void setMaxLinearVel(float max_linear_vel) {_motion_controller.setMaxLinearVel(max_linear_vel);}
    inline float maxLinearVel() const {return _motion_controller.maxLinearVel();}
    inline void setMaxAngularVel(float max_angular_vel) {_motion_controller.setMaxAngularVel(max_angular_vel);}
    inline float maxAngularVel() const {return _motion_controller.maxAngularVel();}
    inline void setMaxLinearAcc(float max_linear_acc) {_motion_controller.setMaxLinearAcc(max_linear_acc);}
    inline float maxLinearAcc() const {return _motion_controller.maxLinearAcc();}
    inline void setMaxAngularAcc(float max_angular_acc) {_motion_controller.setMaxAngularAcc(max_angular_acc);}
    inline float maxAngularAcc() const {return _motion_controller.maxAngularAcc();}
    inline void setMinAngularVel(float min_angular_vel) {_motion_controller.setMinAngularVel(min_angular_vel);}
    inline float minAngularVel() const {return _motion_controller.minAngularVel();}

    inline void setGoalTranslationTolerance(float goal_translation_tolerance) {_motion_controller.setGoalTranslationTolerance(goal_translation_tolerance);}
    inline float goalTranslationTolerance() const {return _motion_controller.goalTranslationTolerance();}
    inline void setGoalRotationTolerance(float goal_rotation_tolerance) {_motion_controller.setGoalRotationTolerance(goal_rotation_tolerance);}
    inline float goalRotationTolerance() const {return _motion_controller.goalRotationTolerance();}

    inline void setRecoveryWaitingTime(int recovery_waiting_time) {_recovery_waiting_time = recovery_waiting_time;}
    inline int recoveryWaitingTime() const {return _recovery_waiting_time;}
    inline void setRecoveryObstacleDistance(float recovery_obstacle_distance) {_recovery_obstacle_distance = recovery_obstacle_distance;}
    inline float recoveryObstacleDistance() const {return _recovery_obstacle_distance;}
    
    //! reads a map in yaml format
    void readMap(const std::string mapname);
    
    //! sets a map from an image
    void setMapFromImage(const UnsignedCharImage& map_image, const float map_resolution,
			 const Eigen::Vector3f& map_origin, const float occ_threshold, const float free_threshold);
    
    //! GUI
    inline void useGUI(bool use_gui){_use_gui = use_gui;}
    void initGUI();
    void handleGUI();
    inline void setVerbose(bool verbose){_verbose = verbose;}
    inline bool verbose() {return _verbose;}
      
    void setGoal(const Eigen::Vector3f& goal);
    void setGoalXY(const Eigen::Vector2f& goal);
    void setRobotPose(const Eigen::Vector3f& robot_pose);
    void setLaserPoints(const Vector2fVector& laser_points);

    //! Computes a step of the planner: given current pose, goal and scan computes a path
    //! and velocities to be applied to reach next waypoint
    void plannerStep();
    
    //! Computes a path given a cost_map, path_map and goal
    void computePath(FloatImage& cost_map, PathMap& path_map, Eigen::Vector2i& goal, Vector2iVector &path);

    //! Velocities
    //! TODO: seting max vels
    inline Eigen::Vector2f velocities() const {return _velocities;}
    
    void cancelGoal();
    void reset();
    void restartPathMap();

    //! Virtual functions to be implemented for the specific robot/environment (e.g., ROS, NAOqi...)
    //! Sends a command to stop the robot.
    virtual void stopRobot() = 0;
    //! Applies a velocity command to the robot.
    virtual void applyVelocities() = 0;
    //! Subscribes to services that should fill the variables (_robot_pose, _laser_points, _goal, _map_image)
    virtual void startSubscribers();
    //! Unsubscribe to services
    virtual void stopSubscribers() = 0;    
    
    virtual void startPublishers();
    virtual void stopPublishers() = 0;

    virtual void init();
    virtual void runOnce();
    
    std::string status;

  protected:

    inline Eigen::Vector2i world2grid(const Eigen::Vector2f p) {
      return Eigen::Vector2i(p.x()*_map_inverse_resolution, p.y()*_map_inverse_resolution);
    }
  
    inline Eigen::Vector2f grid2world(const Eigen::Vector2i p) {
      return Eigen::Vector2f(p.x()*_map_resolution, p.y()*_map_resolution);
    }

    //! Images
    UnsignedCharImage _map_image;
    IntImage _indices_image;
    FloatImage _distance_image;
    FloatImage _cost_image, _cost_image_backup;
    PathMap _distance_map;
    std::vector<PathMap::CellType, PathMap::AllocatorType> _distance_map_backup;
    PathMap _path_map, _path_map_backup;
    DistanceMapPathSearch _dmap_calculator;
    DijkstraPathSearch _path_calculator;
    int _max_distance_map_index;

    Vector2iVector _path, _obstacle_path, _nominal_path;

    //! Path computation variables
    float _max_cost;
    float _min_cost;
    float _robot_radius;
    float _safety_region;
  
    //! Map variables
    float _map_resolution;
    float _map_inverse_resolution;
    float _occ_threshold;
    float _free_threshold;
    Eigen::Vector3f _map_origin;    //< world coordinates of the bottom left pixel 
    Eigen::Vector3f _image_map_origin;    //< world coordinates of the upper left pixel 
    Eigen::Isometry2f _map_origin_transform_inverse;
    Eigen::Isometry2f _image_map_origin_transform_inverse;

    //! GUI stuff
    bool _use_gui;
    WhatToShow _what_to_show;
    boost::mutex _mtx_display;
    static void onMouse( int event, int x, int y, int flags, void* v);
    void handleGUIInput();
    void handleGUIDisplay();
    bool _verbose;

    //! Goal managing
    void setGoalGUI(Eigen::Vector2i goal);
    void updateGoals();
    bool _have_goal;
    bool _have_goal_with_angle;
    Eigen::Vector3f _goal; //map coordinates wrt _map_origin
    Eigen::Vector3f _goal_image; //image coordinates [m]
    Eigen::Vector2i _goal_pixel; //pixel coordinates

    //! Robot pose
    Eigen::Vector3f _robot_pose;  //map coordinates wrt _map_origin
    Eigen::Vector3f _robot_pose_image; //image coordinates [m]
    Eigen::Vector2i _robot_pose_pixel; //pixel coordinates

    //! Dynamic obstacle avoidance
    Vector2fVector _laser_points;
    DynamicMap _dyn_map;

    //! Motion generator
    bool _move_enabled;
    Eigen::Vector2f _velocities;
    MotionController _motion_controller;
    bool computeControlToWaypoint(bool goal_with_angle);
    void setMoveEnabled(bool move_enabled);
    inline bool moveEnabled() const {return _move_enabled;}

    //! Virtual functions to be implemented for the specific environment (e.g., ROS, NAOqi...) 
    //! Subscribers
    virtual void subscribeLaserWithPose() = 0;
    virtual void subscribeGoal() = 0;
    virtual void subscribeMap() = 0;
    virtual void subscribeCancel() = 0;
    virtual void subscribeReset() = 0;
    //! Publishers
    virtual void startCmdVelPublisher() = 0;
    virtual void startPathPublisher() = 0;
    virtual void startResultPublisher() = 0;
    virtual void startStatusPublisher() = 0;
    virtual void publishPath() = 0;
    virtual void publishState() = 0;
    virtual void publishResult(PlannerResult result) = 0;
    virtual void publishExecutionStatus() = 0;

    //! Recovery procedures
    bool _on_recovery_time;
    std::chrono::steady_clock::time_point _recovery_time;
    int _recovery_waiting_time;
    float _recovery_obstacle_distance;
    bool manageRecovery();
    bool recoveryTime();
    bool recoveryPath();
  };


}
