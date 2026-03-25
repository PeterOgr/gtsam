/*Estimator for a car surrounded by other cars
  Other cars send out their own position in global frame
  Main car measures other cars within vision range*/
  
#include <iostream>
#include <fstream>

#include <gtsam\geometry\Pose2.h>

#include "Vehical.h"
#include "Structs.h"

using namespace std;
using namespace gtsam;

/*Pseudocode
initalize some cars into a dictionary
Write their actual position velocity and heading to a .cvs
Have the cars drive at different speeds
Write their estimated position velocity and heading to a .cvs
Have the cars communicate to each other while moveing
*/

int main(int argc, char** argv[]) {
  //Open csv file to write results into
  FILE *test_drive_ptr = fopen("test_drive.csv", "w");

  //Inital car position and velocity
  double car1_init_x = 0;
  double car1_init_y = 0;
  double car1_init_theta = 0;
  Pose2 car1_init_pose(car1_init_x, car1_init_y, car1_init_theta);
  Vector2 car1_init_vel = {0,0};
  double car1_id = 1;

  //Initial car iSAM2 parameters
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  parameters.print();

  // Construct car1
  Auto_Car car1(parameters, car1_init_pose, car1_init_vel, car1_id);

  // Drive the car forward with constant acceleration of 1 over 5sec.
  for (int i=0; i<5; i++)  {
    car1.drive(1, 0, 1);
    fprintf(test_drive_ptr, "%f", car1.p_.x()); //Record true x position 
  }

  //Have car predict position based on imu measurments
  double dt[5] = {1,1,1,1,1};
  double angle_vel[5] = {0,0,0,0,0};
  Eigen::ArrayX2f vel(5,2);
  vel << 1,0, 2,0, 3,0, 4,0, 5,0;
  car1.imu_factor(dt, 5, angle_vel, vel);
  car1.estimate();
  fprintf(test_drive_ptr, "%f", car1.x());

  fclose(test_drive_ptr);
  return 0;
}