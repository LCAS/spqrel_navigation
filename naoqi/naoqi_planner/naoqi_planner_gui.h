#pragma once

#include "yaml_parser/simple_yaml_parser.h"
#include "naoqi_sensor_utils/naoqi_sensor_utils.h"

#include <libgen.h> 

#include <qi/session.hpp>

#include <thread>

#include "srrg_path_map/path_map_utils.h"

namespace naoqi_planner_gui {

  using namespace srrg_core;

  class NAOqiPlannerGUI {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      NAOqiPlannerGUI(qi::SessionPtr session);
    
    void initGUI();
    void readMap(const std::string mapname);

    void subscribeServices();
    void unsubscribeServices();

  protected:
    inline Eigen::Vector2i world2grid(const Eigen::Vector2f p) {
      return Eigen::Vector2i(p.x()*_map_inverse_resolution, p.y()*_map_inverse_resolution);
    }
	
    inline Eigen::Vector2f grid2world(const Eigen::Vector2i p) {
      return Eigen::Vector2f(p.x()*_map_resolution, p.y()*_map_resolution);
    }
    
    qi::SessionPtr _session;
    qi::AnyObject _memory_service;
    
    UnsignedCharImage _map_image;
    IntImage _indices_image;
    
    float _usable_range;
    bool _move_enabled;
    bool _collision_protection_enabled, _collision_protection_desired;
    
    //! State info
    bool _have_goal;
    Eigen::Vector2i _goal;
    Eigen::Vector3f _robot_pose;
    Eigen::Vector2i _robot_pose_image;
    Vector2iVector _path;

    //! Map info
    Eigen::Vector3f _map_origin;    //< world coordinates of the bottom left pixel 
    Eigen::Vector3f _image_map_origin;    //< world coordinates of the upper left pixel 
    Eigen::Isometry2f _map_origin_transform_inverse;
    Eigen::Isometry2f _image_map_origin_transform_inverse;
    float _map_resolution;
    float _map_inverse_resolution;
    float _occ_threshold;
    float _free_threshold;

    //! Execution monitoring
    std::thread _servicesMonitorThread;
    void servicesMonitorThread();
    std::atomic<bool> _stop_thread;
    float _cycle_time_ms;

    // subscribers and publishers
    qi::AnyObject _subscriber_result;
    qi::SignalLink _signal_result_id;
    void onResult(qi::AnyValue value);
    qi::AnyObject _subscriber_collision_protection_enabled;
    qi::SignalLink _signal_collision_protection_enabled_id;
    void onExternalCollisionProtectionEnabled(qi::AnyValue value);
    
    //! GUI stuff
    static void onMouse( int event, int x, int y, int, void* v);
    void handleGUIInput();
    void handleGUIDisplay();
  };

}
