#pragma once

#include <math.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>

using namespace std;
using namespace gtsam;

// Constructs a vehical that can drive 
class Vehical  {
public: 
    Pose2 p_;
    Vector2 v_;
    const int id_;

// Press accelerator and turn the wheel to theta angle
void drive(double accel, double theta, double dt)  {
  Rot2 new_rot(theta);
  v_ = v_ + accel*dt*((Vector(2) << cos(theta), sin(theta)).finished());
  Point2 new_pos= p_.translation() + v_*dt;
  p_ = Pose2(new_rot, new_pos);
  return;
}

// Constructor with inital heading, position, and velocity
Vehical(const double& theta, const Point2& x_y, const Vector2& v, const int id) :
    p_(Pose2(Rot2(theta), x_y)), v_(v), id_(id) {}
};  //Vehical 