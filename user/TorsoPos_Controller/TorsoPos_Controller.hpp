#ifndef TORSOPOS_CONTROLLER
#define TORSOPOS_CONTROLLER

#include <RobotController.h>
#include "TorsoPosUserParameters.h"

class TorsoPos_Controller : public RobotController {
public:
  TorsoPos_Controller():RobotController(){
    desired_q << -0.05, -0.5, 1.0, 0.05, -0.5, 1.0, -0.05, -0.5, 1.0, 0.05, -0.5, 1.0;
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

protected:
  Vec12<float> desired_q;
  Vec12<float> desired_joint_qd;
  Vec12<float> desired_torso_qd;
  Vec2<float> joystickLeft, joystickRight;
  TorsoPosUserParameters userParameters;
};

#endif