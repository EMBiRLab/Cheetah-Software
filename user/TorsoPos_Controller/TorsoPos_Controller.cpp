#include "TorsoPos_Controller.hpp"
#include <iostream>

void TorsoPos_Controller::runController(){
  Mat3<float> kpMat;
  Mat3<float> kdMat;
  kpMat << userParameters.Kp_femur, 0, 0, 0, userParameters.Kp_tibia, 0, 0, 0, userParameters.Kp_tarsus;
  kdMat << userParameters.Kd_femur, 0, 0, 0, userParameters.Kd_tibia, 0, 0, 0, userParameters.Kd_tarsus;
  
  static int iter = 0;
  iter++;

  // This section eventually won't be needed after state estimate
  // is auto propagated to the model
  // {
  FBModelState<float> state;
  
  state.bodyOrientation = _stateEstimate->orientation;
  state.bodyPosition    = _stateEstimate->position;
  state.bodyVelocity.head(3) = _stateEstimate->omegaBody;
  state.bodyVelocity.tail(3) = _stateEstimate->vBody;

  state.q.setZero(12);
  state.qd.setZero(12);

  for (int i = 0; i < 4; ++i) {
    state.q(3*i+0) = _legController->datas[i].q[0];
    state.q(3*i+1) = _legController->datas[i].q[1];
    state.q(3*i+2) = _legController->datas[i].q[2];
    state.qd(3*i+0)= _legController->datas[i].qd[0];
    state.qd(3*i+1)= _legController->datas[i].qd[1];
    state.qd(3*i+2)= _legController->datas[i].qd[2];
  }
  _model->setState(state);
  // }

  // Calculate the current time
  float t = _controlParameters->controller_dt*iter;

  // Need to get the desired orientation of the torso (6DOF position)
  //   1) we start in the default configuration and hold that
  //   2) the joysticks on the gamepad map to desired velocities of the roll, pitch, and yaw. 
  //      (A & Y might map to dz while forward/backward arrow map to dy and left/right arrow map to dx)


  // Once we have desired orientation, we must calculate the desired actuator positions and the tau_ff to compensate for gravity
  Vec18<float> tau_grav;
  tau_grav = _model->generalizedGravityForce(); // [base_force ; joint_torques]
  Vec12<float> tau_ff = tau_grav.tail(12); // prune away base force

  // Periodically print out info about the feedforward term to see that it changes
  if (iter % 2000 == 0) {
    std::cout << "tau_ff @ time=" << t << "\n" << tau_ff << "\n";
  }

  // Read in the gamepad commands
  joystickLeft = _driverCommand->leftStickAnalog;
  joystickRight = _driverCommand->rightStickAnalog;
  
  //TODO @Michael include the cartesian in addition to pitch,roll,yaw soon
  desired_torso_qd = Vec12<float>::Zero();
  if(fabs(joystickLeft(1)) > 0.1f){
    // We are commanding an angular velocity about the pitch axis
    desired_torso_qd(2) -= userParameters.lon_gain*joystickLeft(1);
    desired_torso_qd(5) -= userParameters.lon_gain*joystickLeft(1);
    desired_torso_qd(8) += userParameters.lon_gain*joystickLeft(1);
    desired_torso_qd(11) += userParameters.lon_gain*joystickLeft(1);
  }
  if(fabs(joystickLeft(0)) > 0.1f){
    // We are commanding an angular velocity about the roll axis
    desired_torso_qd(2) -= userParameters.lat_gain*joystickLeft(0);
    desired_torso_qd(5) += userParameters.lat_gain*joystickLeft(0);
    desired_torso_qd(8) -= userParameters.lat_gain*joystickLeft(0);
    desired_torso_qd(11) += userParameters.lat_gain*joystickLeft(0);
  }
  if(fabs(joystickRight(0)) > 0.1f){
    // We are commanding an angular velocity about the yaw axis
    desired_torso_qd(1)  += userParameters.lat_gain*joystickRight(0);
    desired_torso_qd(4)  += userParameters.lat_gain*joystickRight(0);
    desired_torso_qd(7)  -= userParameters.lat_gain*joystickRight(0);
    desired_torso_qd(10) -= userParameters.lat_gain*joystickRight(0);
  }
  if(_driverCommand->y){
    // We are commanding a linear velocity along the (for-aft) x axis
    desired_torso_qd(0) += userParameters.lon_gain;
    desired_torso_qd(3) += userParameters.lon_gain;
    desired_torso_qd(6) += userParameters.lon_gain;
    desired_torso_qd(9) += userParameters.lon_gain;
  }
  if(_driverCommand->a){
    // We are commanding a linear velocity along the (for-aft) x axis
    desired_torso_qd(0) -= userParameters.lon_gain;
    desired_torso_qd(3) -= userParameters.lon_gain;
    desired_torso_qd(6) -= userParameters.lon_gain;
    desired_torso_qd(9) -= userParameters.lon_gain;
  }
  if(_driverCommand->b){
    // We are commanding a linear velocity along the (left-right) y axis
    desired_torso_qd(1) += -userParameters.lat_gain;
    desired_torso_qd(4) += -userParameters.lat_gain;
    desired_torso_qd(7) += -userParameters.lat_gain;
    desired_torso_qd(10) += -userParameters.lat_gain;
  }
  if(_driverCommand->x){
    // We are commanding a linear velocity along the (left-right) y axis
    desired_torso_qd(1) -= -userParameters.lat_gain;
    desired_torso_qd(4) -= -userParameters.lat_gain;
    desired_torso_qd(7) -= -userParameters.lat_gain;
    desired_torso_qd(10) -= -userParameters.lat_gain;
  }
  if(_driverCommand->rightBumper){
    // We are commanding a linear velocity along the (up-down) z axis
    desired_torso_qd(2) += userParameters.vert_gain;
    desired_torso_qd(5) += userParameters.vert_gain;
    desired_torso_qd(8) += userParameters.vert_gain;
    desired_torso_qd(11) += userParameters.vert_gain;
  }
  if(_driverCommand->leftBumper){
    // We are commanding a linear velocity along the (up-down) z axis
    desired_torso_qd(2) -= userParameters.vert_gain;
    desired_torso_qd(5) -= userParameters.vert_gain;
    desired_torso_qd(8) -= userParameters.vert_gain;
    desired_torso_qd(11) -= userParameters.vert_gain;
  }


  // Set the desired commands for the _legController
  if (iter < 2000) {
    // If we are below 2500 iterations, then we don't allow for the gamepad's joystick commands
    // to do anything 
    int dof = 0;
    for(int leg(0); leg<4; ++leg){
      for(int j_idx(0); j_idx<3; ++j_idx){
        _legController->commands[leg].qDes[j_idx] = desired_q(dof);
        _legController->commands[leg].qdDes[j_idx] = 0.;
        _legController->commands[leg].tauFeedForward[j_idx] = tau_ff(dof);
        dof++;
      }
      _legController->commands[leg].kpJoint = kpMat;
      _legController->commands[leg].kdJoint = kdMat;
      std::cout << "Hip location of leg " << leg << " in robot frame is " << _quadruped->getHipLocation(leg) << "\n";
    }
  } else {
    // Nominal operation --- joystick commands are used

    int dof = 0;
    for(int leg(0); leg<4; ++leg){
      // Calculate the Jacobian
      // J and p
      computeLegJacobianAndPosition<float>(*_quadruped, _legController->datas[leg].q, &(_legController->datas[leg].J),
                                      &(_legController->datas[leg].p), leg);
      
      desired_joint_qd.segment(3*leg,3) =  -_legController->datas[leg].J.inverse()*desired_torso_qd.segment(3*leg,3);

      std::cout << "lefto joystick commands are: " << joystickLeft  << std::endl;
      std::cout << "right joystick commands are: " << joystickRight << std::endl;
      std::cout << "---------------------------------------------------------" << std::endl;
      std::cout << "Q variables of leg " << leg << " are:" << _legController->datas[leg].q << std::endl;
      std::cout << "Positions of leg " << leg << " are:" << _legController->datas[leg].p << std::endl;
      std::cout << "Jacobian of leg " << leg << " is:" << _legController->datas[leg].J << std::endl;
      std::cout << "Jacobian inverse of leg " << leg << " is:" << _legController->datas[leg].J.inverse() << std::endl;

      // set the commands
      for(int j_idx(0); j_idx<3; ++j_idx){
        _legController->commands[leg].qDes[j_idx] = desired_q(dof);
        _legController->commands[leg].tauFeedForward[j_idx] = tau_ff(dof);
        dof++;
      }
      _legController->commands[leg].qdDes = desired_joint_qd.segment(3*leg,3);
      std::cout << "Desired qd of leg " << leg << " is:" << desired_joint_qd.segment(3*leg,3) << std::endl;
      _legController->commands[leg].kpJoint = kpMat;
      _legController->commands[leg].kdJoint = kdMat;
    } // for leg
  } // else
}

