#ifndef PROJECT_TORSOPOSUSERPARAMETERS_H
#define PROJECT_TORSOPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class TorsoPosUserParameters : public ControlParameters {
public:
  TorsoPosUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(use_gravity_comp),
        INIT_PARAMETER(Kp_abad),
        INIT_PARAMETER(Kd_abad),
        INIT_PARAMETER(Kp_femur),
        INIT_PARAMETER(Kd_femur),
        INIT_PARAMETER(Kp_tibia),
        INIT_PARAMETER(Kd_tibia),
        INIT_PARAMETER(lon_gain),
        INIT_PARAMETER(lat_gain),
        INIT_PARAMETER(vert_gain),
        INIT_PARAMETER(pitch_gain),
        INIT_PARAMETER(roll_gain),
        INIT_PARAMETER(yaw_gain),
        INIT_PARAMETER(home_max_dq)

  {}

  DECLARE_PARAMETER(double, use_gravity_comp);

  DECLARE_PARAMETER(double, Kp_abad);
  DECLARE_PARAMETER(double, Kd_abad);

  DECLARE_PARAMETER(double, Kp_femur);
  DECLARE_PARAMETER(double, Kd_femur);

  DECLARE_PARAMETER(double, Kp_tibia);
  DECLARE_PARAMETER(double, Kd_tibia);

  DECLARE_PARAMETER(double, lon_gain);
  DECLARE_PARAMETER(double, lat_gain);
  DECLARE_PARAMETER(double, vert_gain);
  DECLARE_PARAMETER(double, pitch_gain);
  DECLARE_PARAMETER(double, roll_gain);
  DECLARE_PARAMETER(double, yaw_gain);
  DECLARE_PARAMETER(double, home_max_dq);

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
