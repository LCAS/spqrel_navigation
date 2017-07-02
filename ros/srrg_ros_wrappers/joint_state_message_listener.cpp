#include "joint_state_message_listener.h"
#include <srrg_messages/joint_state_message.h>
#include <srrg_types/defs.h>

namespace srrg_core_ros {

  using namespace srrg_core;

JointStateMessageListener::JointStateMessageListener(ros::NodeHandle* p_pNode, SensorMessageSorter* p_pSorter ): m_pNode( p_pNode ), m_pSorter( p_pSorter ) {
    //ds nothing to do
}


void JointStateMessageListener::subscribe(const std::string& p_strTopic ) {
    //ds register the topic
    m_strTopic = p_strTopic;

    //ds subscribe to the topic
    m_pSubscriberJointState = m_pNode->subscribe( p_strTopic, -1, &JointStateMessageListener::_callbackJointState, this );

    //ds log
    std::cerr << "subscribing to topic: " << p_strTopic << std::endl;
}

void JointStateMessageListener::_callbackJointState( const sensor_msgs::JointStatePtr p_pJointStateMessage ) {
    //ds get a new message
    JointStateMessage* msgJointState = new JointStateMessage( m_strTopic, p_pJointStateMessage->header.frame_id,  p_pJointStateMessage->header.seq, p_pJointStateMessage->header.stamp.toSec( ) );
    msgJointState->joints().resize(p_pJointStateMessage->name.size());
    for (size_t i=0; i<msgJointState->joints().size(); i++){
      JointStateMessage::JointStatus& joint =msgJointState->joints()[i];
      joint.name=p_pJointStateMessage->name[i];
      joint.position=p_pJointStateMessage->position[i];
      joint.velocity=p_pJointStateMessage->velocity[i];
      joint.effort=p_pJointStateMessage->effort[i];
    }


    //ds add message to sorter
    m_pSorter->insertMessage( msgJointState );
  }

}
