#pragma once

#include "yaml_parser/simple_yaml_parser.h"

#include "dynamic_map.h"

#include <libgen.h> 

#include <thread>
#include <atomic>

#include "srrg_path_map/path_map_utils.h"
#include "srrg_path_map/distance_map_path_search.h"
#include "srrg_path_map/dijkstra_path_search.h"


namespace spqrel_navigation {

  using namespace srrg_core;

  enum WhatToShow {Map, Distance, Cost};
 
  class Planner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      //! Constructor
      Planner();

    inline void setMaxCost(float max_cost) {_max_cost = max_cost;}
    inline float getMaxCost() const {return _max_cost;}
    inline void setMinCost(float min_cost) {_min_cost = min_cost;}
    inline float getMinCost() const {return _min_cost;}
    inline void setRobotRadius(float robot_radius) {_robot_radius = robot_radius;}
    inline float getRobotRadius() const {return _robot_radius;}
    inline void setSafetyRegion(float safety_region) {_safety_region = safety_region;}
    inline float getSafetyRegion() const {return _safety_region;}
//    inline void setExternalCollisionProtectionDesired(bool collision_protection_desired) {
//      _collision_protection_desired = collision_protection_desired;}
//    inline float getExternalCollisionProtectionDesired() const {return _collision_protection_desired;}
      
    void subscribeServices();
    void unsubscribeServices();

    //! reads a map in yaml format
    void readMap(const std::string mapname);

    //! sets a map from an image
    void setMapFromImage(UnsignedCharImage& map_image, float map_resolution,
        Eigen::Vector3f map_origin, int occ_threshold, int free_threshold);

    void initGUI();

    inline void setRobotPose(Eigen::Vector3f robot_pose) {
        _robot_pose=robot_pose;
    }

    inline void setLaserPoints(Vector2fVector laser_points) {
        _laser_points = laser_points;
    }
 
    void setGoal(Eigen::Vector3f vgoal);
    void cancelGoal();

  protected:

    inline Eigen::Vector2i world2grid(const Eigen::Vector2f p) {
      return Eigen::Vector2i(p.x()*_map_inverse_resolution, p.y()*_map_inverse_resolution);
    }
    
    inline Eigen::Vector2f grid2world(const Eigen::Vector2i p) {
      return Eigen::Vector2f(p.x()*_map_resolution, p.y()*_map_resolution);
    }

    inline float normalize(const float angle) {
      return angle + (2*M_PI)*floor((M_PI-angle)/(2*M_PI));
    }
    
    UnsignedCharImage _map_image;

    IntImage _indices_image; 
    FloatImage _distance_image;
    FloatImage _cost_image;
    PathMap _distance_map;
    std::vector<PathMap::CellType, PathMap::AllocatorType> _distance_map_backup;
    PathMap _path_map;
    DistanceMapPathSearch _dmap_calculator;
    DijkstraPathSearch _path_calculator;
    int _max_distance_map_index;
    
    Vector2iVector _path;
    bool _move_enabled;
    bool _collision_protection_enabled, _collision_protection_desired;
    void computeControlToWaypoint(float& v, float& w);
    float _prev_v, _prev_w;

    Vector2fVector _laser_points;
    DynamicMap _dyn_map;

    float _usable_range;
    bool _restart;
    void reset();
    
    bool _have_goal;
    Eigen::Vector2i _goal;
    Eigen::Vector3f _robot_pose;
    Eigen::Vector2i _robot_pose_image;
    
    Eigen::Vector3f _map_origin;    //< world coordinates of the bottom left pixel 
    Eigen::Vector3f _image_map_origin;    //< world coordinates of the upper left pixel 
    Eigen::Isometry2f _map_origin_transform_inverse;
    Eigen::Isometry2f _image_map_origin_transform_inverse;
    float _map_resolution;
    float _map_inverse_resolution;
    float _occ_threshold;
    float _free_threshold;

    //planner variables
    float _max_cost;
    float _min_cost;
    float _robot_radius;
    float _safety_region;

    //! Execution monitoring
    std::thread _servicesMonitorThread;
    void servicesMonitorThread();
    std::atomic<bool> _stop_thread;
    float _cycle_time_ms; 

/*
    // subscribers and publishers
    qi::AnyObject _subscriber_goal;
    qi::SignalLink _signal_goal_id;
    void onGoal(qi::AnyValue value);
    qi::AnyObject _subscriber_move_enabled;
    qi::SignalLink _signal_move_enabled_id;
    void onMoveEnabled(qi::AnyValue value);
    qi::AnyObject _subscriber_collision_protection_desired;
    qi::SignalLink _signal_collision_protection_desired_id;
    void onCollisionProtectionDesired(qi::AnyValue value);
    qi::AnyObject _subscriber_reset;
    qi::SignalLink _signal_reset_id;
*/
    void publishPath();
    void publishGoalReached();

    //! GUI stuff
    bool _use_gui;
    WhatToShow _what_to_show;
    static void onMouse( int event, int x, int y, int, void* v);
    void handleGUIInput();
    void handleGUIDisplay();
    void handleGUIDisplay_OLD();

  };


}

