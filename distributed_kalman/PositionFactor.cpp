/*
Implementation file for PositionFactor.h. Implements a 2D factor where 
position is given like a gps measurement.
*/

#include "PositionFactor.h"

using namespace std;
using namespace gtsam;

Vector evaluateError(const Pose3& nTb,
    OptionalMatrixType H) const {
  return nTb.translation(H) -nT_;
}