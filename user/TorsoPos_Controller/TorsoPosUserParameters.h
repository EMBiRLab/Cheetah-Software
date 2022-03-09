#ifndef PROJECT_TORSOPOSUSERPARAMETERS_H
#define PROJECT_TORSOPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class TorsoPosUserParameters : public ControlParameters {
public:
  TorsoPosUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(Kp_body),
        INIT_PARAMETER(Kd_body),
        INIT_PARAMETER(Kp_ori),
        INIT_PARAMETER(Kd_ori),
        INIT_PARAMETER(Kp_foot),
        INIT_PARAMETER(Kd_foot),
        INIT_PARAMETER(Kp_joint),
        INIT_PARAMETER(Kd_joint),
        INIT_PARAMETER(des_p),
        INIT_PARAMETER(des_theta),
        INIT_PARAMETER(des_dp),
        INIT_PARAMETER(des_dtheta),
        INIT_PARAMETER(des_theta_max),
        INIT_PARAMETER(des_dp_max),
        INIT_PARAMETER(des_dtheta_max)

  {}

  DECLARE_PARAMETER(Vec3<double>, Kp_body);
  DECLARE_PARAMETER(Vec3<double>, Kd_body);

  DECLARE_PARAMETER(Vec3<double>, Kp_ori);
  DECLARE_PARAMETER(Vec3<double>, Kd_ori);

  DECLARE_PARAMETER(Vec3<double>, Kp_foot);
  DECLARE_PARAMETER(Vec3<double>, Kd_foot);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint);

  // Desired intial states
  DECLARE_PARAMETER(Vec3<double>, des_p);
  DECLARE_PARAMETER(Vec3<double>, des_theta);
  DECLARE_PARAMETER(Vec3<double>, des_dp);
  DECLARE_PARAMETER(Vec3<double>, des_dtheta);
  DECLARE_PARAMETER(Vec3<double>, des_theta_max);
  DECLARE_PARAMETER(Vec3<double>, des_dp_max);
  DECLARE_PARAMETER(Vec3<double>, des_dtheta_max);

};

#endif //PROJECT_TORSOPOSUSERPARAMETERS_H
