#pragma once

#include <srrg_types/defs.h>
#include <srrg_messages/sensor_message_sorter.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace srrg_ros_wrapper {

  class CIMUMessageListener {

    //ds ctor/dtor (MUST NOT THROW)
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CIMUMessageListener( ros::NodeHandle* p_pNode, srrg_core::SensorMessageSorter* p_pSorter );

    //ds accessors
  public:

    //ds subscription
    inline ros::NodeHandle* nodeHandle( ){ return m_pNode; }
    void subscribe( const std::string& p_strTopic );

    inline const std::string& getTopic( ) const { return m_strTopic; }

    //ds members (inheritable)
  protected:

    srrg_core::SensorMessageSorter* m_pSorter;
    ros::NodeHandle* m_pNode;
    ros::Subscriber m_pSubscriberIMU;
    std::string m_strTopic;

    //ds helpers
  private:

    void _callbackIMU( const sensor_msgs::ImuPtr p_pIMUMessage );

  };

}
