#ifndef TORSOPOS_CONTROLLER
#define TORSOPOS_CONTROLLER

#include <RobotController.h>
#include "TorsoPosUserParameters.h"

class TorsoPos_Controller : public RobotController {
public:
  TorsoPos_Controller():RobotController(){
// <<<<<<< HEAD
    home << -0.05, -0.6, 1.2, 0.05, -0.6, 1.2, -0.05, -0.6, 1.2, 0.05, -0.6, 1.2;
    desired_q = home;

    max_setpoint_speed_mag_rad_s = 1.0;
    max_setpoint_delta_mag_rad
      = max_setpoint_speed_mag_rad_s * 0.002;//_controlParameters->controller_dt;
    std::cout << "ayo we constructed" << std::endl;
  }
  virtual ~TorsoPos_Controller(){}

  virtual void initializeController(){}
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }
//   virtual void Estop(){ /*maybe just deactivate all joints?*/ }
  void convert_from_torso_to_abad(Vec6<float> &torso_des);
  void clamp_joint_limits(int leg);
  void clamp_torso_limit();
  void find_dq_towards_home();

protected:
  Vec12<float> home;
  Vec12<float> desired_q;
  Vec12<float> desired_joint_qd;
  Vec12<float> manipulator_dq_des;
  // Vec6<float> torso_q;
  Vec6<float> torso_dq_des;
  Vec12<float> desired_torso_qd;

  // Vec12<float> clamped_q;
  // Vec12<float> home_q;
  bool home_pos_initialized = false;

  float max_setpoint_speed_mag_rad_s = 0;
  float max_setpoint_delta_mag_rad = 0;
  
  Vec2<float> joystickLeft, joystickRight;
  TorsoPosUserParameters userParameters;

  Vec2<float> clamp_setpoints(float nominal_qdes_rad, float nominal_qddes_rad, float cur_q);
};

#endif