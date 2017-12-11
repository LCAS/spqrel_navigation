#pragma once

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
    
  protected:

    qi::SessionPtr _session;
    qi::AnyObject _memory_service;
    qi::AnyObject _motion_service;
    
    std::string _map_name;

    float _usable_range;

    //! Subscribers
    qi::AnyObject _subscriber_laserwpose;
    qi::SignalLink _signal_laserwpose_id;
    qi::AnyObject _subscriber_goal;
    qi::SignalLink _signal_goal_id;
    qi::AnyObject _subscriber_cancel;
    qi::SignalLink _signal_cancel_id;
    qi::AnyObject _subscriber_reset;
    qi::SignalLink _signal_reset_id;
    void subscribeLaserWithPose();
    void subscribeGoal();
    void subscribeMap();
    void subscribeCancel();
    void subscribeReset();

    //! Callbacks
    void laserWithPoseCallback();
    void goalCallback(qi::AnyValue value);
    void cancelCallback();
    void resetCallback();

    //! Publishers
    void publishPath();
    //void publishState() = 0;
    //void publishResult(PlannerResult result) = 0;
    //void publishExecutionStatus() = 0;

    
  };




}
