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

  if (MOTION_DEBUG)
      std::cerr << "Force: " << F.transpose() << std::endl;

  v = (F.x() * _T + _prev_linear_vel) / (1 + 2 * _b * _T);
  w = (_k_i * _h * F.y() * _T + _prev_angular_vel) / (1 + 2 * _b * _k_i * _T);

  if (v < 0)
    v = 0;

  if (fabs(w) < 1e-4)
    w = 0;
}



bool MotionController::computeVelocities(const Eigen::Vector3f& robot_pose, const Eigen::Vector2f& goal_xy, Eigen::Vector2f& velocities){

  bool reached = false;
  velocities = Eigen::Vector2f::Zero();

  Eigen::Vector2f robot_pose_xy(robot_pose.x(), robot_pose.y());
  Eigen::Vector2f distance_goal = goal_xy - robot_pose_xy;

  if (distance_goal.norm() > _goal_translation_tolerance){
    float angle_goal = normalize(atan2(distance_goal.y(), distance_goal.x()) - robot_pose.z());
    
    float force = 1.5;
    Eigen::Vector2f F(force*cos(angle_goal), force*sin(angle_goal));

    float v, w;
    _movementGenerator(F, v, w);
    
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

  float angle_goal;
  float force = 1.5;
  if (distance_goal.norm() < _goal_translation_tolerance){
    // We are close enough to the goal
    // Checking orientation with goal for final rotation movement
    
    angle_goal = normalize(goal_xytheta.z() - robot_pose.z());

    if (fabs(angle_goal) > _goal_rotation_tolerance){
      // Robot is not oriented, computing angular vel
      
      Eigen::Vector2f F(force*cos(angle_goal), force*sin(angle_goal));
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
    angle_goal = normalize(atan2(distance_goal.y(), distance_goal.x()) - robot_pose.z());

    Eigen::Vector2f F(force*cos(angle_goal), force*sin(angle_goal));
    float v, w;
    _movementGenerator(F, v, w);

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
  }
  
  _prev_linear_vel = velocities.x();
  _prev_angular_vel = velocities.y();

  return reached;
}
