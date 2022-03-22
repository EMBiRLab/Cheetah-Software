#ifndef PROJECT_TORSOPOSUSERPARAMETERS_H
#define PROJECT_TORSOPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class TorsoPosUserParameters : public ControlParameters {
public:
  TorsoPosUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(Kp_femur),
        INIT_PARAMETER(Kd_femur),
        INIT_PARAMETER(Kp_tibia),
        INIT_PARAMETER(Kd_tibia),
        INIT_PARAMETER(Kp_tarsus),
        INIT_PARAMETER(Kd_tarsus),
        INIT_PARAMETER(lon_gain),
        INIT_PARAMETER(lat_gain),
        INIT_PARAMETER(vert_gain)

  {}

  DECLARE_PARAMETER(double, Kp_femur);
  DECLARE_PARAMETER(double, Kd_femur);

  DECLARE_PARAMETER(double, Kp_tibia);
  DECLARE_PARAMETER(double, Kd_tibia);

  DECLARE_PARAMETER(double, Kp_tarsus);
  DECLARE_PARAMETER(double, Kd_tarsus);

  DECLARE_PARAMETER(double, lon_gain);
  DECLARE_PARAMETER(double, lat_gain);
  DECLARE_PARAMETER(double, vert_gain);

  // // Desired intial states - maybe add these back in later
  // DECLARE_PARAMETER(Vec3<double>, des_p);
  // DECLARE_PARAMETER(Vec3<double>, des_theta);
  // DECLARE_PARAMETER(Vec3<double>, des_dp);
  // DECLARE_PARAMETER(Vec3<double>, des_dtheta);
  // DECLARE_PARAMETER(Vec3<double>, des_theta_max);
  // DECLARE_PARAMETER(Vec3<double>, des_dp_max);
  // DECLARE_PARAMETER(Vec3<double>, des_dtheta_max);

};

#endif //PROJECT_TORSOPOSUSERPARAMETERS_H
