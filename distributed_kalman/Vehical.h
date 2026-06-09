
/*Pseudocode
Vehical Object
  private vars:
    true position velocity and heading
    ID number
  public methods
    Constructor

Auto-Vehical object, inherits vehical
  public vars:
    Estimated position, velocity and heading
    iSAM2 estimator
    factor graph
    inital values for estimation
    result of estimation
    list of other cars
      Estimated position velocity and heading
      ID number
      trust value
      factor graph
      iSAM2 instance
      inital values for estimation
      result of estimation
  public methods:
    Constructor:
      Initalizes variables and factor graph
    Drive forward(Accel, omega, dt)
      Generates IMU data
      Generates GPS data
    Estimate_self(IMU, odometry):
      Make state estimate
    Measure other car(ID)
      Returns distance between the two cars

  Sensors:
    odometry 
    GPS reciever
    Packet reciever
*/
#pragma once

#include <random>
#include <math.h>

#include "Structs.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::V;  // Vel   (xdot,ydot)
using symbol_shorthand::X;  // Pose2 (theta,x,y)

// Constructs a vehical that can drive 
class Vehical  {
public: 
    Pose2 p_;
    Vector2 v_;
     
const double id_;

// Press accelerator and turn the wheel to theta angle
void drive(double accel, double theta, double dt)  {
  Rot2 new_rot(theta);
  v_ = v_ + accel*dt*((Vector(2) << std::cos(theta), std::sin(theta)).finished());
  Point2 new_pos= p_.translation() + v_*dt;
  p_ = Pose2(new_rot, new_pos);
  return;
}

// Constructor with inital heading, position, and velocity
Vehical(const double& theta, const Point2& x_y, const Vector2& v, const double id) :
    p_(Pose2(Rot2(theta), x_y)), v_(v), id_(id) {}
};  //Vehical 

class Auto_Car : public Vehical  {
  private: 
    NonlinearFactorGraph graph_;
    ISAM2Params params_;
    ISAM2* isam = 0;
    Values value_;
    Values estimates_;
    Pose2 p_est;
    Vector2 v_est;
    std::map<double, Auto_Car> car_list;
    int est_count = 0;

  public:

  // Constructor with iSAM params, heading, position, velocity
  Auto_Car(ISAM2Params paramas, const Pose2 true_p, const Vector2 true_v, const double id)  :
            Vehical(true_p.theta(), true_p.translation(), true_v, id)  {
    //Set random generator
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1.0,1.0);  //Uniform distribution
    
    //Initial heading, position and velocity estimate
    double heading_sigma = 20*(M_PI)/180;
    double heading_est = true_p.theta() + heading_sigma*distribution(generator); //+-20deg uncertain
    double x_y_sigma = 1;   //+-1 meter uncertainty
    gtsam::Vector2 x_y_est = true_p.translation() + x_y_sigma*(Vector(2) << distribution(generator),
                                                                distribution(generator)).finished(); 
    p_est = Pose2(heading_est, x_y_est);
    double v_sigma = 1;     //+-1m/s uncertainty
    v_est = true_v + v_sigma*(Vector(2) << distribution(generator),
                                distribution(generator)).finished(); 
    
    //Initalize iSAM and factor graph
    //check if the nonlinear factorgraph is initalized to 0
    isam = new ISAM2(paramas);
    auto x_y_noise = noiseModel::Diagonal::Sigmas((Vector(3) << heading_sigma, x_y_sigma, x_y_sigma).finished()); // rad, m, m
    graph_.addPrior(X(est_count), p_est, x_y_noise);
    auto v_noise = noiseModel::Diagonal::Sigmas((Vector(2) << v_sigma, v_sigma).finished());
    graph_.addPrior(V(est_count), v_est, v_noise);

    //Key Value object that stores inital guesses for nonlinear optimization iterations
    value_.insert(X(est_count), p_est);
    value_.insert(V(est_count), v_est);

    isam->update(graph_, value_);
    estimates_ = isam->calculateEstimate();

    graph_.resize(0);
    value_.clear();
  }

  //Makes a state estimate using the accumulated factors
  void estimate() {
    isam->update(graph_, value_);
    estimates_ = isam->calculateEstimate();

    // reset the graph
    graph_.resize(0);
    value_.clear();

    // Extract state estiamte
    p_est = estimates_.at<Pose2>(X(est_count));
    v_est = estimates_.at<Vector2>(V(est_count));

    return; 
  }

  // Add IMU factors to graph
  void imu_factor(double dt[], int size_of_dt, double omega_meas[], const Eigen::ArrayX2f& vel_meas)  {
/*  Deal with IMU Noise. Use https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imuspecs
    for the equations and use this IMU https://www.digikey.com/en/products/detail/stmicroelectronics/ASM330LHBTR/18073041
    for the actual IMU model.  */
    double bearing_noise_density = 5; //milidegrees per sec/sqrt(Hz)
    double position_noise = 60; //ug/sqrt(Hz)
    double sampling_rate = 1/dt[1];   //TODO:Fix this probably
    double bearing_sigma = position_noise*sqrt(sampling_rate);
    double position_sigma = position_noise*sqrt(sampling_rate);
    double vel_sigma = 0.1;   //Made it up

    est_count++;
    //Accumulate position and bearing change
    double bearing_change = 0;
    Vector2 pos_change = {0,0};
    for (int i=0; i<size_of_dt; i++)  {
      pos_change[0] = pos_change[0] + vel_meas(i, 0)*dt[i];
      pos_change[1] = pos_change[1] + vel_meas(i, 1)*dt[i];
      bearing_change += omega_meas[i]*dt[i];
    }
    Pose2 delta_pose(pos_change[0], pos_change[1], bearing_change);
    Vector2 delta_vel = {v_est[0] - vel_meas(size_of_dt-1, 0), v_est[1] - vel_meas(size_of_dt-1,1)};
    auto pose_noise = noiseModel::Diagonal::Sigmas((Vector(3) << bearing_sigma, position_sigma, position_sigma).finished());
    auto vel_noise = noiseModel::Diagonal::Sigmas((Vector(2) << vel_sigma, vel_sigma).finished());
    auto measured_ = BetweenFactor(X(est_count-1), X(est_count), delta_pose, pose_noise).measured();

    // If you use the slow but correct between factor setting ON, this works fine 
    graph_.add(BetweenFactor<Pose2>(X(est_count-1), X(est_count), delta_pose, pose_noise));
    graph_.add(BetweenFactor<Vector2>(V(est_count-1), V(est_count), delta_vel, vel_noise));
    
    // Propagate position and velocity
    Pose2 pose_prop(p_est.x() + pos_change[0], p_est.y() + pos_change[1], p_est.theta() + bearing_change);
    Vector2 vel_prop = {vel_meas(size_of_dt-1, 0), vel_meas(size_of_dt-1, 1)};
    value_.insert(X(est_count), pose_prop);
    value_.insert(V(est_count), vel_prop);
    return;
  }

  //Add GPS factor. Called with a pose varaible, but will not constrain heading, only 2D position
  void gps_factor(int key, Pose2 position, Vector2 sigmas)  {
    auto gps_noise = noiseModel::Diagonal::Sigmas(sigmas);
    graph_.add(PriorFactor(X(key), position.translation(), gps_noise));
    return;
  }

  // Recieve transmision from another car  


  // Measure distance to other car
  Vector2 Measure_Distance(Vehical car2)  {
    double dist = p_.range(car2.p_);
    double angle = p_.bearing(car2.p_).theta();
    return (Vector(2) << angle, dist).finished();
  }

  /// get x
  inline double x()     const { return p_est.x(); }

  /// get y
  inline double y()     const { return p_est.y(); }

  /// get theta
  inline double theta() const { return p_est.theta(); }

};  //Class Auto_car