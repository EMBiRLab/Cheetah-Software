#include "TorsoPos_Controller.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

void TorsoPos_Controller::runController(){
  Mat3<float> kpMat;
  Mat3<float> kdMat;
  kpMat << userParameters.Kp_abad, 0, 0, 0, userParameters.Kp_femur, 0, 0, 0, userParameters.Kp_tibia;
  kdMat << userParameters.Kd_abad, 0, 0, 0, userParameters.Kd_femur, 0, 0, 0, userParameters.Kd_tibia;
  
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
  //   1) we start in the default "home" configuration and try to maintain that
  //   2) the joysticks on the gamepad map to desired velocities of the roll, pitch, and yaw.
  //      Left  Stick->fore/aft:   pitch
  //      Left  Stick->left/right: roll
  //      Right Stick->left/right: yaw
  //      Y Button:                linear forward
  //      A Button:                linear backward
  //      X Button:                linear left
  //      B Button:                linear right
  //      LB Button:               linear up
  //      RB Button:               linear down
  //      Start Button:            move towards home position


  // Calculate the desired actuator positions and the tau_ff to compensate for gravity
  Vec12<float> tau_ff = Vec12<float>::Zero();
  if(userParameters.use_gravity_comp){
    Vec18<float> tau_grav;
    tau_grav = _model->generalizedGravityForce(); // [base_force ; joint_torques]
    tau_ff = tau_grav.tail(12); // prune away base force
  }

  // Read in the gamepad commands
  joystickLeft = _driverCommand->leftStickAnalog;
  joystickRight = _driverCommand->rightStickAnalog;
  
  torso_dq_des = Vec6<float>::Zero(); //[v; w]
  if(fabs(joystickLeft(1)) > 0.1f){
    // We are commanding an angular velocity about the pitch axis
    torso_dq_des(4) = userParameters.pitch_gain*joystickLeft(1);
  }
  if(fabs(joystickLeft(0)) > 0.1f){
    // We are commanding an angular velocity about the roll axis
    torso_dq_des(3) = userParameters.roll_gain*joystickLeft(0);
  }
  if(fabs(joystickRight(0)) > 0.1f){
    // We are commanding an angular velocity about the yaw axis
    torso_dq_des(5) = userParameters.yaw_gain*joystickRight(0);
  }
  if(_driverCommand->y){
    // We are commanding a linear velocity along the (fore-aft) x axis
    torso_dq_des(0) += userParameters.lon_gain;
  }
  if(_driverCommand->a){
    // We are commanding a linear velocity along the (fore-aft) x axis
    torso_dq_des(0) -= userParameters.lon_gain;
  }
  if(_driverCommand->b){
    // We are commanding a linear velocity along the (left-right) y axis
    torso_dq_des(1) += -userParameters.lat_gain;
  }
  if(_driverCommand->x){
    // We are commanding a linear velocity along the (left-right) y axis
    torso_dq_des(1) -= -userParameters.lat_gain;
  }
  if(_driverCommand->rightBumper){
    // We are commanding a linear velocity along the (up-down) z axis
    torso_dq_des(2) += userParameters.vert_gain;
  }
  if(_driverCommand->leftBumper){
    // We are commanding a linear velocity along the (up-down) z axis
    torso_dq_des(2) -= userParameters.vert_gain;
  }

  if(_driverCommand->start){
    // We are moving back toward the home position
    find_dq_towards_home();

    // set the commands for moving towards home position
    int dof = 0;
    for(int leg(0); leg<4; leg++){
      for(int j_idx(0); j_idx<3; ++j_idx){
        _legController->commands[leg].qDes[j_idx] = desired_q(dof);
        if(std::isnan(tau_ff(dof))){
          _legController->commands[leg].tauFeedForward[j_idx] = 0.0;
        }else{
        _legController->commands[leg].tauFeedForward[j_idx] = tau_ff(dof);
        }
        _legController->commands[leg].qdDes[j_idx] = 0;
        dof++;
      }
      _legController->commands[leg].kpJoint = kpMat;
      _legController->commands[leg].kdJoint = kdMat;
    }
    return;

  } else{
    // Convert the desired torso v and w into desired deltas for the abad/hip joints
    // from their current locations
    convert_from_torso_to_abad(torso_dq_des); // populates manipulator_dq_des
  }

  // Set the desired commands for the _legController
  if (iter < 2000) {
    // If we are below 2000 iterations, then we don't allow for the gamepad's joystick commands
    // to do anything 
    int dof = 0;
    for(int leg(0); leg<4; ++leg){
      for(int j_idx(0); j_idx<3; ++j_idx){
        _legController->commands[leg].qDes[j_idx] = home(dof);
        _legController->commands[leg].qdDes[j_idx] = 0.;
        _legController->commands[leg].tauFeedForward[j_idx] = tau_ff(dof);
        dof++;
      }
      _legController->commands[leg].kpJoint = kpMat;
      _legController->commands[leg].kdJoint = kdMat;
    }
  } else {
    
    // Nominal operation --- joystick commands are used
    int dof = 0;
    for(int leg(0); leg<4; ++leg){
      // Calculate the Jacobian and foot position
      computeLegJacobianAndPosition<float>(*_quadruped, _legController->datas[leg].q, &(_legController->datas[leg].J),
                                      &(_legController->datas[leg].p), leg);

      // TODO: Potentially check to see the COM is outside of the rectangle created by the feet. If so, clamp the desired
      //       positions of the joints to be "safe"  
      // ASSUMPTION: COM of the quadruped is the center of the torso (all actuators are evenly distributed around it)                                    
      
      desired_joint_qd.segment(3*leg,3) =  -_legController->datas[leg].J.inverse()*manipulator_dq_des.segment(3*leg,3);

      // std::cout << "left  joystick commands are: " << joystickLeft  << std::endl;
      // std::cout << "right joystick commands are: " << joystickRight << std::endl;
      // std::cout << "---------------------------------------------------------" << std::endl;
      // std::cout << "Q variables of leg " << leg << " are:" << _legController->datas[leg].q << std::endl;
      // std::cout << "Positions of leg " << leg << " are:" << _legController->datas[leg].p << std::endl;
      // std::cout << "Jacobian of leg " << leg << " is:" << _legController->datas[leg].J << std::endl;
      // std::cout << "Jacobian inverse of leg " << leg << " is:" << _legController->datas[leg].J.inverse() << std::endl;

      desired_q.segment(3*leg,3) += desired_joint_qd.segment(3*leg,3);
      
      // Clamp the desired joint angles according to their hardware limits
      // clamp_joint_limits(leg);

      // set the commands
      for(int j_idx(0); j_idx<3; ++j_idx){
        // _legController->commands[leg].qDes[j_idx] = desired_q(dof) + desired_joint_qd(dof);
        // _legController->commands[leg].qDes[j_idx] = _model->_state.q(dof) + desired_joint_qd(dof);
        // desired_q(dof) += desired_joint_qd(dof);
        _legController->commands[leg].qDes[j_idx] = desired_q(dof);
        if(std::isnan(tau_ff(dof))){
          _legController->commands[leg].tauFeedForward[j_idx] = 0.0;
        }else{
        _legController->commands[leg].tauFeedForward[j_idx] = tau_ff(dof);
        }
        _legController->commands[leg].qdDes[j_idx] = 0;
        dof++;
      }
      _legController->commands[leg].kpJoint = kpMat;
      _legController->commands[leg].kdJoint = kdMat;
    } // for leg
  } // else
}


void TorsoPos_Controller::convert_from_torso_to_abad(Vec6<float> &torso_des){
  // We got the [v;w] of the torso, and need to change that to cartesian displacements for
  // the 4 hip joints
  manipulator_dq_des = Vec12<float>::Zero();

  // linear x axis motion
  manipulator_dq_des(0) = torso_des(0);
  manipulator_dq_des(3) = torso_des(0);
  manipulator_dq_des(6) = torso_des(0);
  manipulator_dq_des(9) = torso_des(0);

  // linear y axis motion
  manipulator_dq_des(1)  = torso_des(1);
  manipulator_dq_des(4)  = torso_des(1);
  manipulator_dq_des(7)  = torso_des(1);
  manipulator_dq_des(10) = torso_des(1);

  // linear z axis motion
  manipulator_dq_des(2)  = torso_des(2);
  manipulator_dq_des(5)  = torso_des(2);
  manipulator_dq_des(8)  = torso_des(2);
  manipulator_dq_des(11) = torso_des(2);

  // angular x axis motion (only use "major-axis", not tangent of circle)
  manipulator_dq_des(2)   -= torso_des(3);
  manipulator_dq_des(5)   += torso_des(3);
  manipulator_dq_des(8)   -= torso_des(3);
  manipulator_dq_des(11)  += torso_des(3);

  // angular y axis motion (only use "major-axis", not tangent of circle)
  manipulator_dq_des(2)  -= torso_des(4);
  manipulator_dq_des(5)  -= torso_des(4);
  manipulator_dq_des(8)  += torso_des(4);
  manipulator_dq_des(11) += torso_des(4);

  // angular z axis motion (only use "major-axis", not tangent of circle)
  manipulator_dq_des(1)  += torso_des(5);
  manipulator_dq_des(4)  += torso_des(5);
  manipulator_dq_des(7)  -= torso_des(5);
  manipulator_dq_des(10) -= torso_des(5);

}


void::TorsoPos_Controller::clamp_joint_limits(int leg){
  for(int j_idx(0); j_idx<3; j_idx++){
      
    // Clamp abad
    if(j_idx == 0 && desired_q(3*leg+j_idx) > _quadruped->_abadMax_q(leg))
      desired_q(3*leg+j_idx) = _quadruped->_abadMax_q(leg);
    if(j_idx == 0 && desired_q(3*leg+j_idx) < _quadruped->_abadMin_q(leg))
      desired_q(3*leg+j_idx) = _quadruped->_abadMin_q(leg);

    // Clamp hip
    if(j_idx == 1 && desired_q(3*leg+j_idx) > _quadruped->_hipMax_q(leg))
      desired_q(3*leg+j_idx) = _quadruped->_hipMax_q(leg);
    if(j_idx == 1 && desired_q(3*leg+j_idx) < _quadruped->_hipMin_q(leg))
      desired_q(3*leg+j_idx) = _quadruped->_hipMin_q(leg);

    // Clamp knee
    if(j_idx == 2 && desired_q(3*leg+j_idx) > _quadruped->_kneeMax_q(leg))
      desired_q(3*leg+j_idx) = _quadruped->_kneeMax_q(leg);
    if(j_idx == 2 && desired_q(3*leg+j_idx) < _quadruped->_kneeMin_q(leg))
      desired_q(3*leg+j_idx) = _quadruped->_kneeMin_q(leg);
      
  }
}


void::TorsoPos_Controller::clamp_torso_limit(){
  return;
}


void::TorsoPos_Controller::find_dq_towards_home(){
  // We want to move incrementally back towards home position if this was called
  Vec12<float> error = desired_q - home;

  for(int dof(0); dof<12; dof++){
    error(dof) = std::max(-(float)userParameters.home_max_dq, std::min(error(dof), (float)userParameters.home_max_dq));
  }

  desired_q = desired_q - error;
}