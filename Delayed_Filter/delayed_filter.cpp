/* Pseudocode
1. Have the vehical drive along a trajectory. Record the position, heading, 
and velocity over time into arrays. Additionally construct a timing array 
that holds the time each measurement was collected. 
2. Create a nonlinear factor graph and a iSAM2 estimator. Feed the measurments
into the factor graph as prior factors of the state. As the measurments are 
put into the graph, have the estimator produce incremental estimates of the 
position. 
3. Reset the factor graph, and repeat the same procedure. However, fed the 
factors into the factor graph in a semirandom order within some time window. 
4. Graph and compare the performance of the estimator 
*/

/* Pseudocode to get the system to work with arbitrary measurement order
1. Generate a list of measurements indexes
2. Randomize the list such that none of the elements are repeated ie [0 3 4 1 2 5 ..] 
3. Loop through the list of measurment indexes
4. If a measurment index is added to the factor graph in the future of the current estimate
    1. Add between factors until esti_index == meas_index
    2. Add values to the initial structure using constant velocity assumption
    3. Calculate an estimate

 */
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "Classes.h"

using namespace std;
using namespace gtsam;

using symbol_shorthand::V;  // Vel   (xdot,ydot)
using symbol_shorthand::X;  // Pose2 (theta,x,y)

namespace {

struct MeasurementSample {
  double time;
  Pose2 pose;
  Vector2 velocity;
  double omega;
};

struct RunSummary {
  vector<double> acc_pose_mse;
  vector<double> acc_vel_mse;
  vector<Pose2> pose_esti;
  vector<Vector2> velocity_esti;
};

RunSummary RunExperiment(const vector<MeasurementSample>& measurements,
                         bool random, double dt) {
  ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  ISAM2 isam(params);
  RunSummary summary;
  NonlinearFactorGraph graph;
  Values initial; 
  Values estimate;

  const auto pose_noise = noiseModel::Isotropic::Sigma(3, 0.05);
  const auto vel_noise = noiseModel::Isotropic::Sigma(2, 0.05);
  double pose_mse = 0.0;
  double vel_mse = 0.0;
  summary.acc_pose_mse.push_back(0.0);
  summary.acc_vel_mse.push_back(0.0);
  int rand_num;
  int esti_index = 0;
  int iter = 0;

  Pose2 pose_estimate(0.0,0.0,0.0);
  Vector2 vel_estimate(0.0,0.0);
  double between_x = 0.0;
  double between_y = 0.0;
  double bearing_change = 0.0;
  double x_prop = 0.0;
  double y_prop = 0.0;
  vector<int> meas_order;
  vector<int> delay = {0};
  for (int i=0;i<measurements.size();++i)  {
    meas_order.push_back(i);
  }

  if (random) {
    std::random_device rd;
    std::mt19937 g(rd());
    shuffle(meas_order.begin()+1, meas_order.end(), g);
  }

  for (auto meas_index: meas_order) {
    //Create the between factor based on the previous velocity
    if (meas_index>esti_index)  {
      while (esti_index<meas_index)  {
        esti_index++;
        between_x = vel_estimate[0]*dt;
        between_y = vel_estimate[1]*dt;
        bearing_change = 0.0;
        Pose2 delta_pose(between_x, between_y, bearing_change);
        auto between_noise = noiseModel::Diagonal::Sigmas((Vector(3) << between_x/3+1e-9, between_y/3+1e-9, bearing_change/3+1e-9).finished());
        graph.add(BetweenFactor<Pose2>(X(esti_index-1), X(esti_index), delta_pose, between_noise));
      
        x_prop = x_prop+between_x;
        y_prop = y_prop+between_y;
        double theta_prop = 0.0;
        Pose2 pose_prop(x_prop, y_prop, theta_prop);
        initial.insert(X(esti_index), pose_prop);
        initial.insert(V(esti_index), vel_estimate);
      }
    }

    //Add measurment to the graph
    graph.add(PriorFactor<Pose2>(X(meas_index), measurements[meas_index].pose, pose_noise));
    graph.add(PriorFactor<Vector2>(V(meas_index), measurements[meas_index].velocity, vel_noise));

    if (meas_index==0)  {
      initial.insert(X(meas_index), measurements[meas_index].pose);
      initial.insert(V(meas_index), measurements[meas_index].velocity);
    }

    isam.update(graph, initial);
    estimate = isam.calculateEstimate();

    pose_estimate = estimate.at<Pose2>(X(esti_index));
    vel_estimate = estimate.at<Vector2>(V(esti_index));
    
    summary.pose_esti.push_back(pose_estimate);
    summary.velocity_esti.push_back(vel_estimate);

    double traj_error = 0.0;   //Accumulated error in state estimates in 2norm
    for (int i=0; i<meas_index; ++i)  {
      traj_error += sqrt(pow(measurements[i].pose.x()-estimate.at<Pose2>(X(i)).x(), 2) +
                         pow(measurements[i].pose.y()-estimate.at<Pose2>(X(i)).y(), 2));
    }
    summary.acc_pose_mse.push_back(traj_error);
    graph.resize(0);
    initial.clear();
  }
  return summary;
}
}  // namespace

int main(int argc, char** argv[]) {
  const size_t kSteps = 30;
  const double dt = 0.1;
  const double accel = 0.25;
  const double steering = 0.05;

  Vehical car(steering, Point2(0.0, 0.0), Vector2(0.0, 0.0), 0);
  vector<MeasurementSample> measurements;
  measurements.reserve(kSteps);

  for (int i = 0; i < kSteps; ++i) {
    car.drive(accel, steering, dt);
    MeasurementSample sample;
    sample.time = i*dt;
    sample.pose = car.p_;
    sample.velocity = car.v_;
    measurements.push_back(sample);
  }
  RunSummary orderedResult = RunExperiment(measurements, false, dt);
  RunSummary semirandomResult = RunExperiment(measurements, true, dt);
  ofstream csv("Delayed_Filter_Comparison.csv");
  csv << "step,time,true_x,true_y,ordered_x,ordered_y,delay_x,delay_y,ordered_accumulated_pose_mse,rand_acc_pose_mse\n";
  for (int i = 0; i < measurements.size(); ++i) {
    csv << i << ',' << measurements[i].time << ','
        << measurements[i].pose.x() << ',' << measurements[i].pose.y() << ','
        << orderedResult.pose_esti[i].x() << ',' << orderedResult.pose_esti[i].y() << ','
        << semirandomResult.pose_esti[i].x() << ',' << semirandomResult.pose_esti[i].y() << ','
        << orderedResult.acc_pose_mse[i] << ',' << semirandomResult.acc_pose_mse[i] << '\n';
  }
  csv.close();
  return 0;
}