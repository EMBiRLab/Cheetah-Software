/*============================= Sit Down ==============================*/
/**
 * Transitionary state that is called for the robot to sit down from
 * balance control mode.
 */

#include "FSM_State_SitDown.h"
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_SitDown<T>::FSM_State_SitDown(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::SIT_DOWN, "SIT_DOWN"),
_ini_foot_pos(4), _err_foot_pos(4), transition_cmd(4), transition_pos(4){
  
  // Set the pre controls safety checks
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;


  // Initialize GRF to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();

  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();

  _wbc_ctrl->setFloatingBaseWeight(1000.);
  
  
  // // Do nothing
  // // Set the pre controls safety checks
  // this->checkSafeOrientation = false;

  // // Post control safety checks
  // this->checkPDesFoot = false;
  // this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_SitDown<T>::onEnter() {
  std::cout << "Enterred onEnter()" << std::endl;

  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Always set the gait to be standing in this state
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
  
  _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;


  // if(_ini_body_pos[2] < 0.15) {
  //   _ini_body_pos[2] = 0.3;
  // }

  last_height_command = _ini_body_pos[2];

  _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
  // _ini_body_ori_rpy[0] = 0.; 
  // _ini_body_ori_rpy[1] = 0.; 
  // _body_weight = this->_data->_quadruped->_bodyMass * 9.81;
  _body_weight = (this->_wbc_ctrl->_model.totalNonRotorMass() + this->_wbc_ctrl->_model.totalRotorMass()) * 9.81;

  // Reset iteration counter
  iter = 0;
  // standup_iter = 0;
  // button_count = 0;


  // FOOT POS!!!! -------------------------------------------------------
  Vec3<T> des_foot_pos(0.0, 0.0, 0.3);
  // --------------------------------------------------------------------


  _avg_foot_err = Vec3<T>::Zero();

  for(size_t leg(0); leg<4; ++leg){
    _ini_foot_pos[leg] =  this->_data->_legController->datas[leg].p;
    _err_foot_pos[leg] =  _ini_foot_pos[leg] - des_foot_pos; 
    _avg_foot_err += _err_foot_pos[leg];
  }

  _avg_foot_err[0] = _avg_foot_err[0] / 4;
  _avg_foot_err[1] = _avg_foot_err[1] / 4;
  _avg_foot_err[2] = _avg_foot_err[2] / 4;

  _des_body_pos = _ini_body_pos + _avg_foot_err;

  std::cout << "made it to the end of OnEnter()!" << std::endl;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_SitDown<T>::run() {

  if(this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
    T hMax = 0.25;
    T progress = 2 * iter * this->_data->controlParameters->controller_dt;

    if (progress > 1.){ progress = 1.; }

    for(int i = 0; i < 4; i++) {
      this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();
      this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();

      this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
      this->_data->_legController->commands[i].pDes[2] = 
        progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
    }
  } else if(this->_data->_quadruped->_robotType == RobotType::MUADQUAD) {
    // Use WBC to get us to stand up correctly
    Vec4<T> contactState;
    contactState<< 0.5, 0.5, 0.5, 0.5;
    this->_data->_stateEstimator->setContactPhase(contactState);
    SitDownStep();
    
    
    // T progress = .75 * standup_iter * this->_data->controlParameters->controller_dt;
    // T hMax = 0.28;

    // if (this->_data->_desiredStateCommand->gamepadCommand->start){
    //   button_count++;
    // }
    // else{
    //   button_count = 0;
    // }

    // if (this->_data->_desiredStateCommand->gamepadCommand->start && stood_up && button_count >= 15){
    //   // pushup mode
    //   // we are in triangular wave "pushup" mode
    //   if(standing_up){ // change detection
    //     standing_up = false;
    //     standup_iter = 0;
    //     std::cout << "MODE TRANSITION DETECT: STANDING TO PUSHUP @ iter " << iter << "\n";
    //     for (int leg = 0; leg < 4; leg++){
    //       std::cout << "_ini_foot_pos of leg " << leg << " is " << _ini_foot_pos[leg] << std::endl;
    //     }
    //   }
    //   // std::cout << "pushup mode. ini z = " << _ini_foot_pos[0][2] << "";

    //   for(int i = 0; i < 4; i++) {
    //     // this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();
    //     this->_data->_legController->commands[i].kpCartesian = Vec3<T>(350, 350, 350).asDiagonal();
    //     // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();
    //     this->_data->_legController->commands[i].kdCartesian = Vec3<T>(2, 2, 2).asDiagonal();
    //     // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(3.25, 3.25, 3.25).asDiagonal();


    //     this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
    //     this->_data->_legController->commands[i].vDes[0] = 0;
    //     this->_data->_legController->commands[i].vDes[1] = 0;
    //     this->_data->_legController->commands[i].vDes[2] = 0;


    //     T A = 0.03;
    //     T pushup_freq = 0.333;
    //     T w = pushup_freq*2*M_PI;
    //     T t = standup_iter * this->_data->controlParameters->controller_dt - (1/pushup_freq)/4;


    //     this->_data->_legController->commands[i].pDes[0] = 0.0;
    //     this->_data->_legController->commands[i].pDes[2] = 
    //       -hMax + (4*A*(1-std::pow(-1,1)) / (std::pow(M_PI,2)*std::pow(1,2))) * std::cos(1*w*t) + 
    //               (4*A*(1-std::pow(-1,3)) / (std::pow(M_PI,2)*std::pow(3,2))) * std::cos(3*w*t) + 
    //               (4*A*(1-std::pow(-1,5)) / (std::pow(M_PI,2)*std::pow(5,2))) * std::cos(5*w*t);


    //     this->_data->_legController->commands[i].vDes[2] = 
    //       -(4*A*1*w*(1-std::pow(-1,1)) / (std::pow(M_PI,2)*std::pow(1,2))) * std::sin(1*w*t) - 
    //        (4*A*3*w*(1-std::pow(-1,3)) / (std::pow(M_PI,2)*std::pow(3,2))) * std::sin(3*w*t) - 
    //        (4*A*5*w*(1-std::pow(-1,5)) / (std::pow(M_PI,2)*std::pow(5,2))) * std::sin(5*w*t);

        
    //     transition_pos[i] = this->_data->_legController->datas[i].p;
    //     transition_cmd[i] = this->_data->_legController->commands[i].pDes;
    //   }

    //   // std::cout << ", cmd pdes z = " << this->_data->_legController->commands[0].pDes[2] 
    //   //  << ", actual z = " <<  this->_data->_legController->datas[0].p[2] << std::endl;
    //   // transition_pos = this->_data->_legController->datas[leg].p;
    //   // transition_cmd = this->_data->_legController->commands[leg].pDes;

    //   standup_iter++;

    // }
    // else {
    //   // go to "stand up" position from wherever you are
    //   if (!standing_up){
    //     standing_up = true;
    //     stood_up = false;
    //     standup_iter = 0;
    //     std::cout << "MODE TRANSITION DETECT: PUSHUP TO STANDING @ iter " << iter << ".... setting new ini\n";
    //     for(size_t leg(0); leg<4; ++leg){
    //       // _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
    //       _ini_foot_pos[leg] = transition_cmd[leg];
    //     }
    //   }

    //   // if (standing_up){
    //   // already standing up, no need to reset standup_iter
    //   // T hMax = 0.25;
    //   // T progress = .75 * standup_iter * this->_data->controlParameters->controller_dt;
    //   progress = 1.3 * standup_iter * this->_data->controlParameters->controller_dt;

    //   // std::cout << "standup mode. ini z = " << _ini_foot_pos[0][2] << "... progress = " << progress;

    //   if (progress > 1.){ 
    //     progress = 1.; 
    //     stood_up = true;
    //     // std::cout << "... FINISHED STANDING";
    //   }

    //   // for(int i = 0; i < 4; i++) {
    //   for(int i = 0; i < 4; i++) {
    //     // this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();
    //     this->_data->_legController->commands[i].kpCartesian = Vec3<T>(350, 350, 350).asDiagonal();
    //     // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();
    //     this->_data->_legController->commands[i].kdCartesian = Vec3<T>(2, 2, 2).asDiagonal();
    //     // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(3.25, 3.25, 3.25).asDiagonal();

    //     this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
    //     this->_data->_legController->commands[i].pDes[2] = 
    //       progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
    //     this->_data->_legController->commands[i].pDes[0] = 
    //       progress*(0) + (1. - progress) * _ini_foot_pos[i][0];
    //     transition_pos[i] = this->_data->_legController->datas[i].p;
    //     transition_cmd[i] = this->_data->_legController->commands[i].pDes;
    //   }
      
    //   // std::cout << ", cmd pdes z = " << this->_data->_legController->commands[0].pDes[2] << std::endl;
    //   // std::cout << "desired z of foot 0 is " << this->_data->_legController->commands[0].pDes[2] << std::endl;
    //   // }
      
    //   standup_iter++;
    // }

    // for(int i = 0; i < 4; i++){
    //   this->_data->_legController->commands[i].forceFeedForward[2] = -20;
    // }

    // for(int i = 2; i < 4; i++){
    //   this->_data->_legController->commands[i].forceFeedForward[2] = -25;
    // }
  }
}

template <typename T>
void FSM_State_SitDown<T>::SitDownStep() {

  T progress = .5 * iter * this->_data->controlParameters->controller_dt; // interpolation
  iter++;

  if (progress > 1.){ progress = 1.; }

  // if(this->_data->_desiredStateCommand->trigger_pressed) {
  //   _wbc_data->pBody_des[2] = 0.25;

  //   if(last_height_command - _wbc_data->pBody_des[2] > 0.001) {
  //     _wbc_data->pBody_des[2] = last_height_command - 0.0001; //speed up eventually. This is 5mm/s
  //   }
  // }
  // last_height_command = _wbc_data->pBody_des[2];


  _wbc_data->pBody_des = progress * _des_body_pos + (1-progress) * _ini_body_pos;
  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();

  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy;
  
  // // Orientation
  // _wbc_data->pBody_RPY_des[0] = 
  //   4*0.06* this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog[0];
  // _wbc_data->pBody_RPY_des[1] = 
  //   4*0.06*this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
  // _wbc_data->pBody_RPY_des[2] -= 
  //   0.3*this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[1];
  
  // // Height
  // _wbc_data->pBody_des[2] += 
  //   0*0.012 * this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];

  // TODO: @Michael you should probably clamp these to not break the robot
  
  _wbc_data->vBody_Ori_des.setZero();

  for(size_t i(0); i<4; ++i){
    _wbc_data->pFoot_des[i].setZero();
    _wbc_data->vFoot_des[i].setZero();
    _wbc_data->aFoot_des[i].setZero();
    _wbc_data->Fr_des[i].setZero();
    _wbc_data->Fr_des[i][2] = _body_weight/4.;
    _wbc_data->contact_state[i] = true;
  }
  
  // if(this->_data->_desiredStateCommand->trigger_pressed) {
  //   _wbc_data->pBody_des[2] = 0.25;

  //   if(last_height_command - _wbc_data->pBody_des[2] > 0.001) {
  //     _wbc_data->pBody_des[2] = last_height_command - 0.0001; //speed up eventually. This is 5mm/s
  //   }
  // }
  // last_height_command = _wbc_data->pBody_des[2];

  // std::cout << "Desired <_wbc_data->pBody_des> is: " << _wbc_data->pBody_des[0] << ", " << 
  //                                                       _wbc_data->pBody_des[1] << ", " << 
  //                                                       _wbc_data->pBody_des[2] << std::endl;

  // std::cout << "Desired <_wbc_data->pBody_RPY_des> is: " << _wbc_data->pBody_RPY_des[0] << ", " << 
  //                                                       _wbc_data->pBody_RPY_des[1] << ", " << 
  //                                                       _wbc_data->pBody_RPY_des[2] << std::endl;

  // for(size_t i(0); i<4; ++i){
  //   std::cout << "Desired <_wbc_data->Fr_des> for leg " << i << " is: " << _wbc_data->Fr_des[i][0] << ", " << 
  //                                                                          _wbc_data->Fr_des[i][1] << ", " <<
  //                                                                          _wbc_data->Fr_des[i][2] << std::endl;
  // }                                                        

  _wbc_ctrl->run(_wbc_data, *this->_data);
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_SitDown<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_SIT_DOWN:
      break;
    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM - XXXXXXXXXXXX] Bad Request: Cannot transition from "
                << K_SIT_DOWN << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_SitDown<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM -- Sit] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_SitDown<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_SitDown<float>;
