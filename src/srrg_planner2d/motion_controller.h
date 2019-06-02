/* 
   Motion controller inspired on the dynamic model presented by J.R. Asensio and L. Montano:
   "A Kinematic and Dynamic Model-Based Motion Controller for Mobile Robots",
   The 15th IFAC Triennial World Congress, July 21-26, 2002, Barcelona, Spain.

   In order to avoid over-oscilation problems reported on the paper, we adopt an strategy of 
   turning on the spot towards the goal before moving forward.

   Author: Maria T. Lazaro
*/
#pragma once

#include <Eigen/Core>
#include <iostream>

class MotionController {
 public:
  MotionController();

  //Set and get velocities and accelerations
  inline void setMaxLinearVel(float max_linear_vel) {
    _max_linear_vel = max_linear_vel;
    _updateParameters();
  }
  inline float maxLinearVel() const {return _max_linear_vel;}
  inline void setMaxAngularVel(float max_angular_vel) {
    _max_angular_vel = max_angular_vel;
    _updateParameters();
  }
  inline float maxAngularVel() const {return _max_angular_vel;}
  inline void setMaxLinearAcc(float max_linear_acc) {
    _max_linear_acc = max_linear_acc;
    _updateParameters();
  }
  inline float maxLinearAcc() const {return _max_linear_acc;}
  inline void setMaxAngularAcc(float max_angular_acc) {
    _max_angular_acc = max_angular_acc;
    _updateParameters();
  }
  inline float maxAngularAcc() const {return _max_angular_acc;}
  
  inline void setMinLinearVel(float min_linear_vel) {_min_linear_vel = min_linear_vel;}
  inline float minLinearVel() const {return _min_linear_vel;}
  inline void setMinAngularVel(float min_angular_vel) {_min_angular_vel = min_angular_vel;}
  inline float minAngularVel() const {return _min_angular_vel;}

  inline void resetVelocities() {
    _prev_linear_vel = 0;
    _prev_angular_vel = 0;
  }

  inline void setGoalTranslationTolerance(float goal_translation_tolerance) {
    _goal_translation_tolerance = goal_translation_tolerance;
  }
  inline float goalTranslationTolerance() const {return _goal_translation_tolerance;}
  inline void setGoalRotationTolerance(float goal_rotation_tolerance) {
    _goal_rotation_tolerance = goal_rotation_tolerance;
  }
  inline float goalRotationTolerance() const {return _goal_rotation_tolerance;}
  
  bool computeVelocities(const Eigen::Vector3f& robot_pose, const Eigen::Vector2f& goal_xy, Eigen::Vector2f& velocities);
  bool computeVelocities(const Eigen::Vector3f& robot_pose, const Eigen::Vector3f& goal_xytheta, Eigen::Vector2f& velocities);
  
protected:

  inline float normalize(const float angle) {
    return angle + (2*M_PI)*floor((M_PI-angle)/(2*M_PI));
  }
  
  void _updateParameters();
  void _movementGenerator(Eigen::Vector2f& F, float& v, float& w);
  
  void adjustVel(float &v, float &w, float angle_goal);

  float _f, _b, _k_i, _h, _T;

  float _max_linear_vel, _max_angular_vel, _max_linear_acc, _max_angular_acc;
  float _min_linear_vel, _min_angular_vel;
  float _prev_linear_vel, _prev_angular_vel;

  float _goal_translation_tolerance, _goal_rotation_tolerance;

  float _force;
  bool _orienting;
  
};
