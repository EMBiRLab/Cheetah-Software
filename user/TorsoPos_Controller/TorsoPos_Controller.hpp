#ifndef TORSOPOS_CONTROLLER
#define TORSOPOS_CONTROLLER

#include <RobotController.h>
#include "TorsoPosUserParameters.h"

class TorsoPos_Controller : public RobotController {
public:
  TorsoPos_Controller():RobotController(){
    desired_q << -0.05, -0.5, 1.0, 0.05, -0.5, 1.0, -0.05, -0.5, 1.0, 0.05, -0.5, 1.0;
    home_q = desired_q;
    // computeLegJacobianAndPosition

    max_setpoint_speed_mag_rad_s = 0.05;
    max_setpoint_delta_mag_rad
      = max_setpoint_speed_mag_rad_s * _controlParameters->controller_dt;

  }
  virtual ~TorsoPos_Controller(){}

  virtual void initializeController(){}
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }
//   virtual void Estop(){ /*maybe just deactivate all joints?*/ }

protected:
  Vec12<float> desired_q;
  Vec12<float> desired_joint_qd;
  Vec12<float> desired_torso_qd;

  // Vec12<float> clamped_q;
  Vec12<float> home_q;
  bool home_pos_initialized = false;

  float max_setpoint_speed_mag_rad_s = 0;
  float max_setpoint_delta_mag_rad = 0;
  
  Vec2<float> joystickLeft, joystickRight;
  TorsoPosUserParameters userParameters;

  Vec2<float> clamp_setpoints(float nominal_qdes_rad, float nominal_qddes_rad, float cur_q);
};

#endif