#ifndef PLANNING_DATA_SYSTEM_CONSTRAINTS_H_
#define PLANNING_DATA_SYSTEM_CONSTRAINTS_H_

#include <string>
#include <vector>
#include <Eigen/Core>

namespace data{

struct SystemConstraints  {
  
  //bool checkParamsValid(std::string* error_message);

  // variables
  double v_max;             // m/s, maximum absolute velocity
  double a_max;             // m/s2, maximum absolute acceleration
  double yaw_rate_max;      // rad/s, maximum yaw rate
  double yaw_accel_max;     // rad/s2, maximum yaw acceleration
  double collision_radius;  // m

};





} //namespace data

#endif // PLANNING_DATA_SYSTEM_CONSTRAINTS_H_