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
  FILE *test_drive_ptr = fopen("test_drive.csv", "w");
  Point2 car1_init_pos = {0,0};
  Vector2 car1_init_vel = {0,0};
  double car1_init_theta = 0;
  double car1_id = 1;
  Vehical car1(car1_init_theta, car1_init_pos, car1_init_vel, car1_id);
  for (int i=0; i<5; i++)  {
    car1.drive(1, 0, 1);
    fprintf(test_drive_ptr, "%f, %f, %f\n", car1.p_.x(), car1.p_.y(), car1.p_.theta());
  }
  fclose(test_drive_ptr);
  return 0;
}