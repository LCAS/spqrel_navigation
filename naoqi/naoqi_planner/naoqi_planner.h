#pragma once

#include <thread>

#include <qi/session.hpp>
#include <boost/program_options.hpp>

#include "naoqi_sensor_utils/naoqi_sensor_utils.h"

#include "srrg_planner2d/planner.h"

namespace spqrel_navigation {

  using namespace srrg_planner;

  class NAOqiPlanner: public Planner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    NAOqiPlanner(qi::SessionPtr session);

    void getParams(boost::program_options::variables_map& vm);

    void stopRobot();
    void applyVelocities();

    void start();
    void stop();

    inline bool useD2L() {return _use_d2l;}
    inline void setUseD2L(bool use_d2l) {_use_d2l = use_d2l;}
    
  protected:

    qi::SessionPtr _session;
    qi::AnyObject _memory_service;
    qi::AnyObject _motion_service;
    
    std::string _map_name;

    float _usable_range;
    bool _use_d2l; //Use Depth2Laser data

    //! Subscribers
    qi::AnyObject _subscriber_laserwpose;
    qi::SignalLink _signal_laserwpose_id;
    qi::AnyObject _subscriber_goal, _subscriber_goalxy;
    qi::SignalLink _signal_goal_id, _signal_goalxy_id;
    qi::AnyObject _subscriber_cancel;
    qi::SignalLink _signal_cancel_id;
    qi::AnyObject _subscriber_reset;
    qi::SignalLink _signal_reset_id;
    qi::AnyObject _subscriber_externalcollisionprotectiondesired;
    qi::SignalLink _signal_externalcollisionprotectiondesired_id;
    qi::AnyObject _subscriber_move_enabled;
    qi::SignalLink _signal_move_enabled_id;
    void subscribeLaserWithPose();
    void subscribeGoal();
    void subscribeMap();
    void subscribeCancel();
    void subscribeReset();
    void subscribeExternalCollisionProtectionDesired();
    void subscribeMoveEnabled();
    void startSubscribers();
    void stopSubscribers();

    //! Callbacks
    void laserWithPoseCallback();
    void goalCallback(qi::AnyValue value);
    void goalCallbackXY(qi::AnyValue value);
    void cancelCallback();
    void resetCallback();
    void externalCollisionProtectionDesiredCallback(qi::AnyValue value);
    void moveEnabledCallback(qi::AnyValue value);

    //! Publishers
    void startCmdVelPublisher(){};
    void startPathPublisher(){};
    void startResultPublisher(){};
    void publishPath();
    virtual void publishState(){};
    void publishResult(PlannerResult result);
    virtual void publishExecutionStatus(){};
    virtual void stopPublishers(){};

    //! Execution
    int _cycle_time_ms; 
    std::atomic<bool> _stop_thread;
    std::thread _running_thread;
    void run();

    //! Pepper self protection management
    bool _collision_protection_desired;
    bool _collision_protection_enabled;
    inline void setExternalCollisionProtectionDesired(bool collision_protection_desired) {_collision_protection_desired = collision_protection_desired;};
    void setExternalCollisionProtectionEnabled(bool collision_protection_enabled);
    
  };




}
