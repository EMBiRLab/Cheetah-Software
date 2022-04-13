#ifndef TORSOPOS_CONTROLLER
#define TORSOPOS_CONTROLLER

#include <RobotController.h>
#include "TorsoPosUserParameters.h"

class TorsoPos_Controller : public RobotController {
public:
  TorsoPos_Controller():RobotController(){
    home << -0.05, -0.5, 1.0, 0.05, -0.5, 1.0, -0.05, -0.5, 1.0, 0.05, -0.5, 1.0;
    desired_q = home;
    // computeLegJacobianAndPosition
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
  Vec2<float> joystickLeft, joystickRight;
  TorsoPosUserParameters userParameters;
};

#endif