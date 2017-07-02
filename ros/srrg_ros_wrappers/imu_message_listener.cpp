#include "imu_message_listener.h"
#include <srrg_messages/imu_message.h>
#include <srrg_types/defs.h>

namespace srrg_ros_wrapper {

  using namespace srrg_core;
  
  CIMUMessageListener::CIMUMessageListener(ros::NodeHandle* p_pNode, SensorMessageSorter* p_pSorter ): m_pNode( p_pNode ), m_pSorter( p_pSorter ) {
    //ds nothing to do
  }


  void CIMUMessageListener::subscribe( const std::string& p_strTopic ) {
    //ds register the topic
    m_strTopic = p_strTopic;

    //ds subscribe to the topic
    m_pSubscriberIMU = m_pNode->subscribe( p_strTopic, -1, &CIMUMessageListener::_callbackIMU, this );

    //ds log
    std::cerr << "subscribing to topic: " << p_strTopic << std::endl;
  }

  void CIMUMessageListener::_callbackIMU( const sensor_msgs::ImuPtr p_pIMUMessage ) {
    //ds get a new message
    CIMUMessage* msgIMU = new CIMUMessage( m_strTopic, p_pIMUMessage->header.frame_id,  p_pIMUMessage->header.seq, p_pIMUMessage->header.stamp.toSec( ) );

    //ds fill the message
    msgIMU->setOrientation( Eigen::Quaterniond( p_pIMUMessage->orientation.x, p_pIMUMessage->orientation.y, p_pIMUMessage->orientation.z, p_pIMUMessage->orientation.w ) );
    msgIMU->setAngularVelocity( Eigen::Vector3d( p_pIMUMessage->angular_velocity.x, p_pIMUMessage->angular_velocity.y, p_pIMUMessage->angular_velocity.z ) );
    msgIMU->setLinearAcceleration( Eigen::Vector3d( p_pIMUMessage->linear_acceleration.x, p_pIMUMessage->linear_acceleration.y, p_pIMUMessage->linear_acceleration.z ) );

    //ds add message to sorter
    m_pSorter->insertMessage( msgIMU );
  }

}
