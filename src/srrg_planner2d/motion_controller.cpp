#include "motion_controller.h"

MotionController::MotionController(){

  _max_linear_acc = 1.0;
  _max_linear_vel = 0.25;
  _max_angular_vel = 0.5;
  _max_angular_acc = 2.0;

  _min_linear_vel  = 0.05;
  _min_angular_vel = 0.05;
  
  _updateParameters();

  _prev_linear_vel  = 0;
  _prev_angular_vel = 0;

  _T = 0.2;

  _goal_translation_tolerance = 0.2;
  _goal_rotation_tolerance = 0.3;

  _force = 1.5;
  _orienting = false;

}

void MotionController::_updateParameters(){

  _f = _max_linear_acc;

  _b = _f / (2 * _max_linear_vel);

  _h = (2 * _b * _max_angular_vel) / _f;

  _k_i = _max_angular_acc / (_f * _h);

}

void MotionController::_movementGenerator(Eigen::Vector2f& F, float& v, float& w) {
  //Given a force to the goal, the controller parameters, and previous vels, compute new vels.

  //Limiting force
  if (F.norm() > _f){
    float angleF = atan2(F.y(), F.x());
    F.x() = cos(angleF) * _f;
    F.y() = sin(angleF) * _f;
  }

  v = (F.x() * _T + _prev_linear_vel) / (1 + 2 * _b * _T);
  w = (_k_i * _h * F.y() * _T + _prev_angular_vel) / (1 + 2 * _b * _k_i * _T);

  if (v < 0)
    v = 0;

  if (fabs(w) < 1e-4)
    w = 0;
}

// adapting v and w according to angle to goal
void MotionController::adjustVel(float &v, float &w, float angle_goal) {

    // printf(" ++ adjust vel %.3f %.3f  (angle_goal: %.3f)", v,w,angle_goal);

    float delta_slow_down = 0.25;
    // Check if angle to the goal greater than a threshold
    if (fabs(angle_goal) > (1+delta_slow_down) * _goal_rotation_tolerance){
      // Switch to rotation-only behaviour to orientate towards the goal
      if (fabs(w) > 0 && fabs(w) < _min_angular_vel)
        w = (w < 0 ? -_min_angular_vel : _min_angular_vel);
      v = 0; // dont move forward
      _orienting = true;
    }
    // slow down if close to target
    else if (fabs(angle_goal) > _goal_rotation_tolerance){
      int sign = (w>=0)?+1:-1;
      float w1 = w;
      w =  fabs(w) * (fabs(angle_goal) - _goal_rotation_tolerance)/
                     (delta_slow_down * _goal_rotation_tolerance) ;

      if (w<_min_angular_vel) w = _min_angular_vel; // keep some min velocity

      w = w * sign;
      //printf("[slow down %.3f -> %.3f]\n", w1,w);
      if (fabs(w) > 0 && fabs(w) < _min_angular_vel)
        w = (w < 0 ? -_min_angular_vel : _min_angular_vel);
      v = 0; // dont move forward
      _orienting = true;
    }
    else if (_orienting) {
        // oriented! stop rotation for 1 cycle
        w=0;
        _orienting = false;
    }

    // printf(" -> %.3f %.3f\n", v, w);
}


bool MotionController::computeVelocities(const Eigen::Vector3f& robot_pose, const Eigen::Vector2f& goal_xy, Eigen::Vector2f& velocities){

  bool reached = false;
  velocities = Eigen::Vector2f::Zero();

  Eigen::Vector2f robot_pose_xy(robot_pose.x(), robot_pose.y());
  Eigen::Vector2f distance_goal = goal_xy - robot_pose_xy;

  //std::cerr << " -- goal: " << goal_xy.transpose()  << " -- robot: " << robot_pose_xy.transpose() 
  //      << " - distance_goal: " << distance_goal.transpose() << std::endl;

  if (distance_goal.norm() > _goal_translation_tolerance){
    // Far from target goal
    float angle_goal = normalize(atan2(distance_goal.y(), distance_goal.x()) - robot_pose.z());
    
    //std::cerr << " -- angle_goal: " << angle_goal << std::endl;

    Eigen::Vector2f F(_force*cos(angle_goal), _force*sin(angle_goal));

    float v, w;
    _movementGenerator(F, v, w);

#if 1 // LI using new function
    adjustVel(v, w, angle_goal);
    velocities.x() = v;
    velocities.y() = w;
#else    
    // Check if angle to the goal greater than a threshold
    if (fabs(angle_goal) > _goal_rotation_tolerance){
      // Switch to rotation-only behaviour to orientate towards the goal
      if (fabs(w) > 0 && fabs(w) < _min_angular_vel)
	    w = (w < 0 ? -_min_angular_vel : _min_angular_vel);
      velocities.y() = w;
    }
    else {
      // Otherwise the velocities computed are returned
      velocities.x() = v;
      velocities.y() = w;
    }
#endif

  } else {
    reached = true;
  }
  
  _prev_linear_vel = velocities.x();
  _prev_angular_vel = velocities.y();

  return reached;
}


bool MotionController::computeVelocities(const Eigen::Vector3f& robot_pose, const Eigen::Vector3f& goal_xytheta, Eigen::Vector2f& velocities){

  bool reached = false;
  velocities = Eigen::Vector2f::Zero();
  
  Eigen::Vector2f robot_pose_xy(robot_pose.x(), robot_pose.y());
  Eigen::Vector2f goal_xy(goal_xytheta.x(), goal_xytheta.y());
  Eigen::Vector2f distance_goal = goal_xy - robot_pose_xy;

  // std::cerr << " -- goal: " << goal_xy.transpose()  << " -- robot: " << robot_pose_xy.transpose() 
  //      << " - distance_goal: " << distance_goal.transpose() << std::endl;

  float angle_goal;
  
  if (distance_goal.norm() < _goal_translation_tolerance){
    // We are close enough to the goal
    // Checking orientation with goal for final rotation movement
    
    angle_goal = normalize(goal_xytheta.z() - robot_pose.z());

    if (fabs(angle_goal) > _goal_rotation_tolerance){
      // Robot is not oriented, computing angular vel
      
      Eigen::Vector2f F(_force*cos(angle_goal), _force*sin(angle_goal));
      float v, w;
      _movementGenerator(F, v, w);

      if (fabs(w) > 0 && fabs(w) < _min_angular_vel)
	    w = (w < 0 ? -_min_angular_vel : _min_angular_vel);
      velocities.y() = w;
    } else {
      // We are close and oriented
      reached = true;
    }
  } else {
    // Far from target goal
    angle_goal = normalize(atan2(distance_goal.y(), distance_goal.x()) - robot_pose.z());

    // std::cerr << " -- angle_goal: " << angle_goal << std::endl;


    Eigen::Vector2f F(_force*cos(angle_goal), _force*sin(angle_goal));
    float v, w;
    _movementGenerator(F, v, w);

#if 1 // LI using new function
    adjustVel(v, w, angle_goal);
    velocities.x() = v;
    velocities.y() = w;
#else
    // Check if angle to the goal greater than a threshold
    if (fabs(angle_goal) > 1.2 * _goal_rotation_tolerance){
      // Switch to rotation-only behaviour to orientate towards the goal
      if (fabs(w) > 0 && fabs(w) < _min_angular_vel)
        w = (w < 0 ? -_min_angular_vel : _min_angular_vel);
      velocities.y() = w;
    }
    else {
      // Otherwise the velocities computed are returned
      velocities.x() = v;
      velocities.y() = w;
    }
#endif


  }
  
  _prev_linear_vel = velocities.x();
  _prev_angular_vel = velocities.y();

  return reached;
}
