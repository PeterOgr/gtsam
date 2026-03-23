#pragma once

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>

using namespace std;
using namespace gtsam;


//Sending message of current vehicals state
struct packet {
  double sending_id;
  gtsam::Pose2 pose;
  double time;
};

//Measurement of neighboring vehical
struct Measurements {
  double time;
  double id;
};
