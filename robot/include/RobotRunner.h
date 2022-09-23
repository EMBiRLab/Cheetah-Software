/*!
 * @file RobotRunner.h
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "Controllers/StateEstimatorContainer.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/RobServBoard.h"
#include "rt/rt_rc_interface.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include "JPosInitializer.h"

#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/VisualizationData.h"
#include "Utilities/PeriodicTask.h"
#include "cheetah_visualization_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "RobotController.h"
#include <lcm-cpp.hpp>

#define MQ_MOT_ROT        2 * M_PI / 7.5
#define MQ_MOT_ROT_X3_NEW 2 * M_PI / 12.0232

class RobotRunner : public PeriodicTask {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotRunner(RobotController* , PeriodicTaskManager*, float, std::string);
  using PeriodicTask::PeriodicTask;
  void init() override;
  void run() override;
  void cleanup() override;

  // Initialize the state estimator with default no cheaterMode
  void initializeStateEstimator(bool cheaterMode = false);
  virtual ~RobotRunner();

  RobotController* _robot_ctrl;

  GamepadCommand* driverCommand;
  RobotType robotType;
  VectorNavData* vectorNavData;
  CheaterState<double>* cheaterState;
  SpiData* spiData;
  SpiCommand* spiCommand;
  TiBoardCommand* tiBoardCommand;
  TiBoardData* tiBoardData;
  RobotControlParameters* controlParameters;
  VisualizationData* visualizationData;
  CheetahVisualization* cheetahMainVisualization;
  // Need to define these, also defined in hardware bridge.h appropriately
  // Need to update legcontroller.h also to use the data accordingly (updatedata and updatecommand functions)
  

  //Updating LCM Data and Command (Robot_server) MUADQUAD
  int   muadquad_leg_reordering[12] = {3,4,5,0,1,2,9,10,11,6,7,8}; // todo: place into a good spot like mq quadruped
  // following written in MIT angle convention
  // temp note: to run robot in x-type, the back legs need their offsets flipped!
  // float muadquad_angle_offsets[12]  = {0,-1*MQ_MOT_ROT,4*MQ_MOT_ROT_X3_NEW,
  //                                      0,-1*MQ_MOT_ROT,4*MQ_MOT_ROT_X3_NEW,
  //                                      0,-1*MQ_MOT_ROT,4*MQ_MOT_ROT_X3_NEW, // for xtype back legs offsets flipped 
  //                                      0,-1*MQ_MOT_ROT,4*MQ_MOT_ROT_X3_NEW}; // todo: place into a good spot like mq quadruped // for xtype back legs offsets flipped 
  // // robot_server_response_lcmt* LCMData;
  float muadquad_angle_offsets[12]  = {0,-1*MQ_MOT_ROT,5*MQ_MOT_ROT_X3_NEW,
                                       -1*MQ_MOT_ROT,-1*MQ_MOT_ROT,2*MQ_MOT_ROT_X3_NEW,
                                       1*MQ_MOT_ROT,0,5*MQ_MOT_ROT_X3_NEW, // for xtype back legs offsets flipped 
                                       0,0,4*MQ_MOT_ROT_X3_NEW}; 
  RobServData* robServData;
  void handleresponseLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const robot_server_response_lcmt* msg);
  void handlelcm();
  std::thread _responselcmthread;
  lcm::LCM _responseLCM;
  robot_server_command_lcmt* LCMCommand;
  RobServCommand* robServCommand;
  lcm::LCM _commandLCM;

 private:
  float _ini_yaw;

  int iter = 0;

  void setupStep();
  void finalizeStep();

  JPosInitializer<float>* _jpos_initializer;
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;
  bool _cheaterModeEnabled = false;
  DesiredStateCommand<float>* _desiredStateCommand;
  rc_control_settings rc_control;
  lcm::LCM _lcm;
  leg_control_command_lcmt leg_control_command_lcm;
  state_estimator_lcmt state_estimator_lcm;
  leg_control_data_lcmt leg_control_data_lcm;
  // Contact Estimator to calculate estimated forces and contacts

  FloatingBaseModel<float> _model;
  u64 _iterations = 0;
};

#endif  // PROJECT_ROBOTRUNNER_H
