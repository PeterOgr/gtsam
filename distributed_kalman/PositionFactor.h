/*
Defines a gps factor that works in only 2D
*/

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace std;
using namespace gtsam;

class PositionFactor: public NoiseModelFactorN<Pose2> {
private:
  typedef NoiseModelFactorN<Pose2> Base;
  Point2 nT_;   //Position measurement in cartesian coordinates

public:
  typedef PositionFactor This;

  /* Constructer from a measurment in the local coordinates NED
  convert coordinates from Latitude Longitude before passing to 
  constructor. Maybe use GeographicLib? */
  PositionFactor(Key key, const Point2& pos, const SharedNoiseModel& model) : 
      Base(model, key), nT_(pos)  {
  }
  
  // overrides evaluateError function. Defined in .cpp file
  Vector evaluateError(const Pose2& p,
      boost::optional<Matrix&> H = boost::none) const override;
}