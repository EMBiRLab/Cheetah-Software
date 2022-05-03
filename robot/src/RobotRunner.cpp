/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <unistd.h>
#include <eigen3/Eigen/Geometry>

#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/MuadQuad.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"

RobotRunner::RobotRunner(RobotController* robot_ctrl, 
    PeriodicTaskManager* manager, 
    float period, std::string name):
  PeriodicTask(manager, period, name),
  _lcm(getLcmUrl(255)) {

    _robot_ctrl = robot_ctrl;

    // for (int dof = 0; dof < 12; dof++){
    //   LCMData->q[dof] = std::numeric_limits<float>::quiet_NaN();
    //   LCMData->qd[dof] = std::numeric_limits<float>::quiet_NaN();
    //   LCMData->tau_est[dof] = std::numeric_limits<float>::quiet_NaN();
    // }
    // LCMData->fsm_state=0;
  }

void RobotRunner::handlelcm() {
  while (true){
    _responseLCM.handle();
  }
}

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  printf("[RobotRunner] initialize\n");

  // Build the appropriate Quadruped object
  if (robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else if (robotType == RobotType::CHEETAH_3){
    _quadruped = buildCheetah3<float>();
  } else if (robotType == RobotType::MUADQUAD){
    //Running the LCMhandler to recieve the messages from robot_server_response
    std::cout<<"-----------------------------SUBSCRIBINGGGGGG!!!-----------------------------\n";
    if (!_responseLCM.good()) {
      // Need Some error statements here
      std::cout<<"[RobotRunner] Response LCM is NOT working!\n";
    } else {
      std::cout<<"[RobotRunner] Response LCM is working great!\n";
    }
    _responseLCM.subscribe("robot_server_response", &RobotRunner::handleresponseLCM, this);
    printf("[RobotRunner] Start Response LCM handler\n");
    _responselcmthread = std::thread(&RobotRunner::handlelcm, this);
    _quadruped = buildMuadQuad<float>();
  }

  // Initialize the model and robot data
  _model = _quadruped.buildModel();
  _jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt); //TODO @MICHAEL: Give a reasonable power-on trajectory for muadquad

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);
  initializeStateEstimator(false);

  memset(&rc_control, 0, sizeof(rc_control_settings));
  // Initialize the DesiredStateCommand object
  _desiredStateCommand =
    new DesiredStateCommand<float>(driverCommand,
        &rc_control,
        controlParameters,
        &_stateEstimate,
        controlParameters->controller_dt);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  _robot_ctrl->_visualizationData= visualizationData;
  _robot_ctrl->_robotType = robotType;
  _robot_ctrl->_driverCommand = driverCommand;
  _robot_ctrl->_controlParameters = controlParameters;
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();

}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {
  // Run the state estimator step
  //_stateEstimator->run(cheetahMainVisualization);
  _stateEstimator->run();
  //cheetahMainVisualization->p = _stateEstimate.position;
  visualizationData->clear();

  // std::cout << "ENTERING SETUP STEP..\n";
  // Update the data from the robot
  setupStep();

  static int count_ini(0);
  ++count_ini;

  // std::cout << "count_ini is: " << count_ini << "\n";

  if (count_ini < 10) {
    _legController->setEnabled(false);
  } else if (20 < count_ini && count_ini < 30) {
    _legController->setEnabled(false);
  } else if (40 < count_ini && count_ini < 50) {
    _legController->setEnabled(false);
  } else {
    _legController->setEnabled(true);

    if( (rc_control.mode == RC_mode::OFF) && controlParameters->use_rc ) {
      if(count_ini%1000 ==0) {
        printf("ESTOP!\n");
        // std::cout << "rc_control.mode=" << rc_control.mode << 
                      // ", controlParameters->use_rc=" << controlParameters->use_rc << 
                      // "\n";
      }
      for (int leg = 0; leg < 4; leg++) {
        _legController->commands[leg].zero();
      }
      _robot_ctrl->Estop();
    }else {
      // Controller
      if (!_jpos_initializer->IsInitialized(_legController)) {
        Mat3<float> kpMat;
        Mat3<float> kdMat;
        // Update the jpos feedback gains
        if (robotType == RobotType::MINI_CHEETAH) {
          kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
          kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
        } else if (robotType == RobotType::CHEETAH_3) {
          kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
          kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        } else if (robotType == RobotType::MUADQUAD){
          //Need to redefine these for MUADQUAD
          kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
          kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
        } else {
          assert(false);
        } 

        for (int leg = 0; leg < 4; leg++) {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
      } else {
        // Run Control 
        _robot_ctrl->runController();
        cheetahMainVisualization->p = _stateEstimate.position;

        // Update Visualization
        _robot_ctrl->updateVisualization();
        cheetahMainVisualization->p = _stateEstimate.position;
      }
    }

  }

  // printf("[_robotRunner] above visualization!\n");

  // Visualization (will make this into a separate function later)
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      cheetahMainVisualization->q[leg * 3 + joint] =
        _legController->datas[leg].q[joint];
    }
  }
  cheetahMainVisualization->p.setZero();
  cheetahMainVisualization->p = _stateEstimate.position;
  cheetahMainVisualization->quat = _stateEstimate.orientation;

  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep() {
  // Update the leg data
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateData(spiData);
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateData(tiBoardData);
  } else if (robotType == RobotType::MUADQUAD) {
    // std::cout<<"Running the setup step!\n";
    // _responseLCM.subscribe("robot_server_response", &RobotRunner::handleresponseLCM, this);
    // std::cout<<"Updating the legController!\n";
    
    // _legController->updateData(LCMData);
    _legController->updateData(robServData);
  } else {
    assert(false);
  }

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

  // state estimator
  // check transition to cheater mode:
  if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode:
  if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }

  get_rc_control_settings(&rc_control);

  // todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner::finalizeStep() {
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateCommand(spiCommand);
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateCommand(tiBoardCommand);
  } else if (robotType == RobotType::MUADQUAD) {
    // _legController->updateCommand(LCMCommand);
    // _commandLCM.publish("robot_server_command", LCMCommand);
    // std::cout << "Going to update the command before publishing!" << std::endl;
    _legController->updateCommand(robServCommand);
    // std::cout << "Updated the command first time!" << std::endl;

    robot_server_command_lcmt LCMCommandfix;

    // static int leg_reordering[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
    std::cout << "[robot_server_command->tau_ff[";
    for(int leg = 0; leg < 4; leg++) {
      for(int axis = 0; axis < 3; axis++) {
        int idx = leg*3 + axis;
        idx = muadquad_leg_reordering[idx];
        int sign = 1;
        // std::cout << "[robServCommand->qd_des[" << idx << "]]----->" << robServCommand->qd_des[idx] << std::endl;
        if (leg%2 == 0)
          sign = -1;
        // if (axis == 0)
        //   sign *= -1;
        LCMCommandfix.tau_ff[idx] = sign*robServCommand->tau_ff[idx];
        //lcmcommand->f_ff[idx] = commands[leg].forceFeedForward[axis];
        LCMCommandfix.q_des[idx]  = sign*robServCommand->q_des[idx];
        LCMCommandfix.qd_des[idx] = sign*robServCommand->qd_des[idx];
        //lcmcommand->p_des[idx] = commands[leg].pDes[axis];
        //lcmcommand->v_des[idx] = commands[leg].vDes[axis];
        //lcmcommand->kp_cartesian[idx] = commands[leg].kpCartesian(axis, axis);
        //lcmcommand->kd_cartesian[idx] = commands[leg].kdCartesian(axis, axis);
        LCMCommandfix.kp_joint[idx] = robServCommand->kp_joint[idx];
        LCMCommandfix.kd_joint[idx] = robServCommand->kd_joint[idx];
        // std::cout << LCMCommandfix.q_des[idx] << ", ";
      }
    }
    std::cout << LCMCommandfix.tau_ff[muadquad_leg_reordering[1]] << ", ";
    std::cout << LCMCommandfix.tau_ff[muadquad_leg_reordering[2]];
    std::cout << "]" << std::endl;
    // std::cout << "Updated the command second time!" << std::endl;
    _lcm.publish("robot_server_command", &LCMCommandfix);
    // _commandLCM.publish("robot_server_command", &LCMCommandfix);
    // std::cout << "FINALLY PUBLISHED!" << std::endl;
  } else {
    assert(false);
  }
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  _stateEstimate.setLcm(state_estimator_lcm);
  _lcm.publish("leg_control_command", &leg_control_command_lcm);
  _lcm.publish("leg_control_data", &leg_control_data_lcm);
  _lcm.publish("state_estimator", &state_estimator_lcm);
  _iterations++;
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  if (cheaterMode) {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
    _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
  } else {
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  }
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
  delete _jpos_initializer;
}

void RobotRunner::cleanup() {}

//Handling the response LCM
void RobotRunner::handleresponseLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const robot_server_response_lcmt* msg){
  (void)rbuf;
  (void)chan;
  // std::cout<<"Getting the messages!! \n";
  // for (int i = 0; i<12;i++){
  //   LCMData->q[i] = msg->q[i];
  //   LCMData->qd[i] = msg->qd[i];
  //   LCMData->tau_est[i] = msg->tau_est[i];
  // }
  // LCMData->fsm_state = msg->fsm_state;
  // std::cout<<"fsm_state is "<<LCMData->fsm_state;
  // std::cout<<"Getting the messages!! \n";
  int sign = 0;
  // std::cout << "robServData->q[";
  for (int i = 0; i<12; i++){
    sign = 1;
    if ((i / 3) % 2 == 0)
      sign = -1;
    // if ((i % 3) == 0)
    //   sign *= -1;
    int idx = muadquad_leg_reordering[i];
    robServData->q[i] = sign*msg->q[idx];
    robServData->qd[i] = sign*msg->qd[idx];
    robServData->tau_est[i] = sign*msg->tau_est[idx];
    // std::cout << robServData->q[i] << ", ";
  }
  // std::cout << "]\n";
  robServData->fsm_state = msg->fsm_state;
  // std::cout<<"fsm_state is "<< (int)robServData->fsm_state;

  // Populate vectorNavData here from the lcm bc we receive IMU 
  // updates via lcm from robot_server
  // Firstly, rotate the quaternion by 180 about y bc its mounted
  // upside-down
  // static Eigen::Quaternionf y_180(0,0,1,0);
  static Eigen::Quaternionf y_90(std::sqrt(2)/2.0,0,std::sqrt(2)/2.0,0);
  Eigen::Quaternionf robserv_quat(msg->quat[0],msg->quat[1],msg->quat[2],msg->quat[3]);
  // robserv_quat = y_180 * robserv_quat;
  robserv_quat = y_90 * robserv_quat;

  vectorNavData->accelerometer(2) =  msg->accelerometer[0];
  vectorNavData->accelerometer(1) =  msg->accelerometer[1];
  vectorNavData->accelerometer(0) = -msg->accelerometer[2];

  vectorNavData->gyro(2) =  msg->gyro[0];
  vectorNavData->gyro(1) =  msg->gyro[1];
  vectorNavData->gyro(0) = -msg->gyro[2];


  // for (int i = 0; i < 3; i++){
  //   vectorNavData->accelerometer(i) = msg->accelerometer[i];
  //   vectorNavData->gyro(i) = msg->gyro[i];
  // }
  // get the quaternion
  vectorNavData->quat(0) = robserv_quat.w();
  vectorNavData->quat.segment(1,3) = robserv_quat.vec();

  // // negate the linear and angular data to match account for mounting of IMU
  // vectorNavData->accelerometer(0) = -vectorNavData->accelerometer(0);
  // vectorNavData->accelerometer(2) = -vectorNavData->accelerometer(2);
  // vectorNavData->gyro(0) = -vectorNavData->gyro(0);
  // vectorNavData->gyro(2) = -vectorNavData->gyro(2);
}
