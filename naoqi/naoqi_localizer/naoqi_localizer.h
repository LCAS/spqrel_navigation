#pragma once

#include "srrg_localizer2d/localization_filter.h"
#include "yaml_parser/simple_yaml_parser.h"
#include "naoqi_sensor_utils/naoqi_sensor_utils.h"

#include <libgen.h> 

#include <qi/session.hpp>

#include <thread>
#include <chrono>

namespace naoqi_localizer {

  using namespace std;

  /*!
    class that implements a NAOqi localization node.
    It uses a particle filter with likelihood function based on a distance map
  */
  class NAOqiLocalizer : public srrg_localizer2d::LocalizationFilter{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      //! initializes the localizer 
      NAOqiLocalizer(qi::SessionPtr session);
  
    //! call this after construction if you want to have a gui
    void initGUI();

    inline bool useD2L() {return _use_d2l;}
    inline void setUseD2L(bool use_d2l) {_use_d2l = use_d2l;}
    
    //! sets/gets the maximum range of a laser, ranges further than this are ignored
    inline float forcedMaxRange() const {return _forced_max_range;}
    inline void setForcedMaxRange( float mr) {_forced_max_range=mr;}
  
    //! endpoints that are closer between each other than this threshold are suppressed
    inline float squaredEndpointDistance() const {return _squared_endpoint_distance;}
    inline void setSquaredEndpointDistance(float sd) { _squared_endpoint_distance=sd;}
  
    //! call this when all is in place, AND the map is loaded (either via direct setMap function or
    //! by calling the requestMap service)
    void subscribeServices();
    void unsubscribeServices();

    //! reads a map in yaml format
    void readMap(const std::string mapname);
  
    void setInitialPose(float x, float y,float theta);
  protected:

    qi::SessionPtr _session;
    qi::AnyObject _memory_service;
    qi::AnyObject _motion_service;

    Eigen::Vector3f _old_odom_pose; //< stores the previous odom pose, used to compute the control 
    Eigen::Vector3f _laser_pose; //< stores the laser pose from the system
    /* bool _has_laser_pose;*/
    bool _restarted;
    
    std::vector<double> _timers; //< holds the update time of the last n cycles
    size_t _last_timer_slot; // the current update cycle

    float _forced_max_range; //< max range to use in localization
    float _squared_endpoint_distance; //< max distance betweem endpoints in the laser. The ones that are closer are suppressed

    Eigen::Vector3f _map_origin;    //< world coordinates of the bottom left pixel 
    Eigen::Vector3f _image_map_origin;    //< world coordinates of the upper left pixel 
  
    //! converts a range scan into a vector of regularized endpoints
    //! @param endpoints: a vector2fvector containing the cartesian 
    //!                   written by the method coordinares of the regularized endpoints
    //! @paeam laser_pose: pose of the laser on the robot
    //! @param msg: the laser message to be processed
    srrg_core::Vector2fVector rawPointsToRobotFrame(srrg_core::Vector2fVector& rawPoints);

    //! use depth2laser data
    bool _use_d2l;
    
    /*
    //! publishes the particles
    void publishParticles();

    //! publishes the statistics (amcl_pose) and the transform (tf)
    void publishPose();

    //! publishes the distance between each laser endpoint and the closest point in the map
    void publishRanges();
    */
    //! handles a laser scan and updates the filter
    std::thread _servicesMonitorThread;
    void servicesMonitorThread();
    std::atomic<bool> _stop_thread;
    int _cycle_time_ms;
    
    //! handles the set-pose-estimate message from outside
    //! puts all partcles in the neighborhood of the pose in msg
    void onPoseChanged(qi::AnyValue value);
    qi::AnyObject _subscriber_pose;
    qi::SignalLink _signal_pose_id;
    
    /*
    //! triggers global localization
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res);
  
    // called when the map is updated. causes some overhead
    void mapMessageCallback(const nav_msgs::OccupancyGrid& msg);
    */
    // GUI stuff
    //! callback fot the mouse, used to set the robot pose
    //! when in set_pose mode, left click sets the pose
    //! right click sets the orientation
    static void onMouse( int event, int x, int y, int, void* );
    
    //! handles the keyboard
    //!  "s": toggles setPose mode
    //!  "d": toggles map mode (distance/occupancy)
    //!  "r": toggles particle resetting (spread the particles when enterning in unknown)
    //!  "g": triggers global localization
    void handleGUIInput();

    //! computes the average computation time of the last n updates
    double cycleLatency() const;

    //! shows the gui stuff
    void handleGUIDisplay();
    bool _use_gui;           //! handled internally
    bool _show_distance_map; //! if 1 in the gui shows the distance map. (to be toggled with "d")
    bool _force_redisplay;   //! if toggled to one forces the display and sends out all messages 
    bool _set_pose;          //! set_pose mode (to be toggled with "s")
  };

}
