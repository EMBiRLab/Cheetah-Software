#ifndef TORSOPOS_CONTROLLER
#define TORSOPOS_CONTROLLER

#include <RobotController.h>
#include <TorsoPosUserParameters.h>

class TorsoPos_Controller : public RobotController {
public:
//   TorsoPos_Controller():RobotController(),_jpos_ini(cheetah::num_act_joint){
//     _jpos_ini.setZero();
//   }
  TorsoPos_Controller();
  virtual ~TorsoPos_Controller(){}

  virtual void initializeController();
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }
//   virtual void Estop(){ _controlFSM->initialize(); }

protected:
  DVec<float> _jpos_ini;
  TorsoPosUserParameters userParameters;
};

#endif