/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandUp.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
_ini_foot_pos(4), transition_cmd(4), transition_pos(4){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_StandUp<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;
  standup_iter = 0;
  button_count = 0;

  for(size_t leg(0); leg<4; ++leg){
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_StandUp<T>::run() {

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
    T progress = .75 * standup_iter * this->_data->controlParameters->controller_dt;
    T hMax = 0.28;

    if (this->_data->_desiredStateCommand->gamepadCommand->start){
      button_count++;
    }
    else{
      button_count = 0;
    }

    if (this->_data->_desiredStateCommand->gamepadCommand->start && stood_up && button_count >= 15){
      // pushup mode
      // we are in triangular wave "pushup" mode
      if(standing_up){ // change detection
        standing_up = false;
        standup_iter = 0;
        std::cout << "MODE TRANSITION DETECT: STANDING TO PUSHUP @ iter " << iter << "\n";
        for (int leg = 0; leg < 4; leg++){
          std::cout << "_ini_foot_pos of leg " << leg << " is " << _ini_foot_pos[leg] << std::endl;
        }
      }
      // std::cout << "pushup mode. ini z = " << _ini_foot_pos[0][2] << "";

      for(int i = 0; i < 4; i++) {
        // this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();
        this->_data->_legController->commands[i].kpCartesian = Vec3<T>(300, 300, 300).asDiagonal();
        // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();
        this->_data->_legController->commands[i].kdCartesian = Vec3<T>(2, 2, 2).asDiagonal();
        // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(3.25, 3.25, 3.25).asDiagonal();


        this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
        this->_data->_legController->commands[i].vDes[0] = 0;
        this->_data->_legController->commands[i].vDes[1] = 0;
        this->_data->_legController->commands[i].vDes[2] = 0;


        T A = 0.03;
        T pushup_freq = 0.333;
        T w = pushup_freq*2*M_PI;
        T t = standup_iter * this->_data->controlParameters->controller_dt - (1/pushup_freq)/4;


        this->_data->_legController->commands[i].pDes[0] = 0.0;
        this->_data->_legController->commands[i].pDes[2] = 
          -hMax + (4*A*(1-std::pow(-1,1)) / (std::pow(M_PI,2)*std::pow(1,2))) * std::cos(1*w*t) + 
                  (4*A*(1-std::pow(-1,3)) / (std::pow(M_PI,2)*std::pow(3,2))) * std::cos(3*w*t) + 
                  (4*A*(1-std::pow(-1,5)) / (std::pow(M_PI,2)*std::pow(5,2))) * std::cos(5*w*t);


        this->_data->_legController->commands[i].vDes[2] = 
          -(4*A*1*w*(1-std::pow(-1,1)) / (std::pow(M_PI,2)*std::pow(1,2))) * std::sin(1*w*t) - 
           (4*A*3*w*(1-std::pow(-1,3)) / (std::pow(M_PI,2)*std::pow(3,2))) * std::sin(3*w*t) - 
           (4*A*5*w*(1-std::pow(-1,5)) / (std::pow(M_PI,2)*std::pow(5,2))) * std::sin(5*w*t);

        
        transition_pos[i] = this->_data->_legController->datas[i].p;
        transition_cmd[i] = this->_data->_legController->commands[i].pDes;
      }

      // std::cout << ", cmd pdes z = " << this->_data->_legController->commands[0].pDes[2] 
      //  << ", actual z = " <<  this->_data->_legController->datas[0].p[2] << std::endl;
      // transition_pos = this->_data->_legController->datas[leg].p;
      // transition_cmd = this->_data->_legController->commands[leg].pDes;

      standup_iter++;

    }
    else {
      // go to "stand up" position from wherever you are
      if (!standing_up){
        standing_up = true;
        stood_up = false;
        standup_iter = 0;
        std::cout << "MODE TRANSITION DETECT: PUSHUP TO STANDING @ iter " << iter << ".... setting new ini\n";
        for(size_t leg(0); leg<4; ++leg){
          // _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
          _ini_foot_pos[leg] = transition_cmd[leg];
        }
      }

      // if (standing_up){
      // already standing up, no need to reset standup_iter
      // T hMax = 0.25;
      // T progress = .75 * standup_iter * this->_data->controlParameters->controller_dt;
      progress = 1.3 * standup_iter * this->_data->controlParameters->controller_dt;

      // std::cout << "standup mode. ini z = " << _ini_foot_pos[0][2] << "... progress = " << progress;

      if (progress > 1.){ 
        progress = 1.; 
        stood_up = true;
        // std::cout << "... FINISHED STANDING";
      }

      // for(int i = 0; i < 4; i++) {
      for(int i = 0; i < 4; i++) {
        // this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();
        this->_data->_legController->commands[i].kpCartesian = Vec3<T>(300, 300, 300).asDiagonal();
        // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();
        this->_data->_legController->commands[i].kdCartesian = Vec3<T>(2, 2, 2).asDiagonal();
        // this->_data->_legController->commands[i].kdCartesian = Vec3<T>(3.25, 3.25, 3.25).asDiagonal();

        this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
        this->_data->_legController->commands[i].pDes[2] = 
          progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
        this->_data->_legController->commands[i].pDes[0] = 
          progress*(0) + (1. - progress) * _ini_foot_pos[i][0];
        transition_pos[i] = this->_data->_legController->datas[i].p;
        transition_cmd[i] = this->_data->_legController->commands[i].pDes;
      }
      
      // std::cout << ", cmd pdes z = " << this->_data->_legController->commands[0].pDes[2] << std::endl;
      // std::cout << "desired z of foot 0 is " << this->_data->_legController->commands[0].pDes[2] << std::endl;
      // }
      
      standup_iter++;
    }

    for(int i = 0; i < 4; i++){
      this->_data->_legController->commands[i].forceFeedForward[2] = -21;
    }

    for(int i = 2; i < 4; i++){
      this->_data->_legController->commands[i].forceFeedForward[2] = -27;
    }
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_STAND_UP:
      break;
    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      break;


    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_PASSIVE << " to "
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
TransitionData<T> FSM_State_StandUp<T>::transition() {
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
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_StandUp<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;
