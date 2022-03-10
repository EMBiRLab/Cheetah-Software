// printf("go home\n");
//   FBModelState<double> homeState;
//   homeState.bodyOrientation << 1, 0, 0, 0;
//   homeState.bodyPosition = Vec3<double>(0, 0, 0.4);
//   homeState.bodyVelocity = SVec<double>::Zero();
//   homeState.q = DVec<double>(12);
//   homeState.q << -0.05, -0.8, 1.7, 0.05, -0.8, 1.7, -0.05, -0.8, 1.7, 0.05, -0.8, 1.7;
//   homeState.qd = homeState.q;

//   _simulation->setRobotState(homeState);


#include "TorsoPos_Controller.hpp"
#include <iostream>

void TorsoPos_Controller::runController(){
  Mat3<float> kpMat;
  Mat3<float> kdMat;
  kpMat << userParameters.Kp_joint(0), 0, 0, 0, userParameters.Kp_joint(1), 0, 0, 0, userParameters.Kp_joint(2);
  kdMat << userParameters.Kd_joint(0), 0, 0, 0, userParameters.Kd_joint(1), 0, 0, 0, userParameters.Kd_joint(2);
  
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

  if (iter % 2000 == 0) {
    std::cout << "tau_ff @ time=" << t << "\n" << tau_ff << "\n";
  }



  Vec2<float> joystickLeft, joystickRight;

  std::cout << "Read in the gamepad commands\n";
  joystickLeft = gamepadCommand->leftStickAnalog;
  joystickRight = gamepadCommand->rightStickAnalog;
  std::cout << "We read them in!\n";
  //include the cartesian in addition to pitch,roll,yaw soon
  if(fabs(joystickLeft(0)) > 0.1f){
    // We are commanding an angular velocity about the roll axis
    std::cout << "ROLL\n";
  }
  if(fabs(joystickLeft(1)) > 0.1f){
    // We are commanding an angular velocity about the pitch axis
    std::cout << "PITCH\n";
  }
  if(fabs(joystickRight(0)) > 0.1f){
    // We are commanding an angular velocity about the yaw axis
    std::cout << "YAW\n";
  }



  if (iter < 4000) {
    // If we are below 3000 iterations, then we don't allow for the gamepad's joystick commands
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
    }
  } else {
    // Nominal operation --- joystick commands are interpreted
    // Vec2<float> joystickLeft, joystickRight;

    joystickLeft = gamepadCommand->leftStickAnalog;
    joystickRight = gamepadCommand->rightStickAnalog;
    //include the cartesian in addition to pitch,roll,yaw soon
    if(fabs(joystickLeft(0)) > 0.1f){
      // We are commanding an angular velocity about the roll axis
      std::cout << "ROLL\n";
    }
    if(fabs(joystickLeft(1)) > 0.1f){
      // We are commanding an angular velocity about the pitch axis
      std::cout << "PITCH\n";
    }
    if(fabs(joystickRight(0)) > 0.1f){
      // We are commanding an angular velocity about the yaw axis
      std::cout << "YAW\n";
    }





  }




  // // Send joint feed-forward torques and desired positions to leg controllers
  // int dof = 0;
  // for(int leg(0); leg<4; ++leg){
  //   for(int j_idx(0); j_idx<3; ++j_idx){
  //     _legController->commands[leg].qDes[j_idx] = 0;
  //     _legController->commands[leg].qdDes[j_idx] = 0.;
  //     _legController->commands[leg].tauFeedForward[j_idx] = tau_ff(dof);
  //     dof++;
  //   }
  //   _legController->commands[leg].kpJoint = kpMat;
  //   _legController->commands[leg].kdJoint = kdMat;
  // }
}





// void Leg_InvDyn_Controller::runController(){
//   static int iter = 0;
//   iter++;

  

//   // This section eventually won't be needed after state estimate
//   // is auto propagated to the model
//   // {
  
//   FBModelState<float> state;
  
//   state.bodyOrientation = _stateEstimate->orientation;
//   state.bodyPosition    = _stateEstimate->position;
//   state.bodyVelocity.head(3) = _stateEstimate->omegaBody;
//   state.bodyVelocity.tail(3) = _stateEstimate->vBody;

//   state.q.setZero(12);
//   state.qd.setZero(12);

//   for (int i = 0; i < 4; ++i) {
//     state.q(3*i+0) = _legController->datas[i].q[0];
//     state.q(3*i+1) = _legController->datas[i].q[1];
//     state.q(3*i+2) = _legController->datas[i].q[2];
//     state.qd(3*i+0)= _legController->datas[i].qd[0];
//     state.qd(3*i+1)= _legController->datas[i].qd[1];
//     state.qd(3*i+2)= _legController->datas[i].qd[2];
//   }
//   _model->setState(state);
//   // }

//   float t = _controlParameters->controller_dt*iter;

//   // Desired trajectory parameters for a simple joint-space trajectory
//   float freq_Hz = 1;
//   float freq_rad = freq_Hz * 2* 3.14159;
//   float amplitude = 3.1415/3;


//   // Desired angles, angular velocities, and angular accelerations. 
//   // We'll be lazy and use the same desired trajectory for each moving joint.
//   Vec12<float> qDes, qdDes, qddDes;
//   qDes.setZero();
//   qdDes.setZero();
//   qddDes.setZero();

//   // Desired trajecotry for each joint is a sin wave
//   float desired_angle = sin(t*freq_rad)*amplitude;
//   float desired_rate = freq_rad*cos(t*freq_rad)*amplitude;
//   float desired_acceleration = freq_rad*freq_rad*sin(t*freq_rad)*amplitude;

//   // Set desired for a subset of legs
//   for( int i = 0 ; i < (int) userParameters.num_moving_legs ; i++) {
//     qDes.segment(3*i,3).setConstant(desired_angle);
//     qdDes.segment(3*i,3).setConstant(desired_rate);
//     qddDes.segment(3*i,i).setConstant(desired_acceleration);
//   }

//   // Opposite Ab/ad for L & R Legs!!
//   qDes(0)*=-1; qdDes(0)*=-1; qddDes(0)*=-1;
//   qDes(6)*=-1; qdDes(6)*=-1; qddDes(6)*=-1;


//   // Construct commanded acceleration
//   FBModelStateDerivative<float> commandedAccleration;
//   commandedAccleration.dBodyVelocity.setZero();
//   commandedAccleration.qdd = qddDes + 25*(qdDes - state.qd) + 150*(qDes - state.q);
 
//   // Run RNEA inverse dynamics
//   Vec18<float> generalizedForce = _model->inverseDynamics(commandedAccleration); // [base_force ; joint_torques]
//   Vec12<float> jointTorques = generalizedForce.tail(12); // prune away base force


//   // Alternate strategy: Assemble equations of motion
//   Mat18<float> H;
//   Vec18<float> Cqd, tau_grav;
  
//   H = _model->massMatrix();
//   Cqd = _model->generalizedCoriolisForce();
//   tau_grav = _model->generalizedGravityForce();

//   Vec18<float> generalizedAcceleration;
//   generalizedAcceleration.head(6)  = commandedAccleration.dBodyVelocity;
//   generalizedAcceleration.tail(12) = commandedAccleration.qdd;
//   Vec18<float> generalizedForce2 = H*generalizedAcceleration + Cqd + tau_grav;

//   // Make sure they match
//   Vec18<float> err = generalizedForce - generalizedForce2;
//   assert( err.norm() < 1e-4 );

//   // Send joint torques to leg controllers
//   int dof = 0;
//   for(int leg(0); leg<4; ++leg){
//     for(int j_idx(0); j_idx<3; ++j_idx){
//       _legController->commands[leg].qDes[j_idx] = 0;
//       _legController->commands[leg].qdDes[j_idx] = 0.;
//       _legController->commands[leg].tauFeedForward[j_idx] = jointTorques(dof);
//       dof++;
//     }
//     _legController->commands[leg].kpJoint = Mat3<float>::Zero();
//     _legController->commands[leg].kdJoint = Mat3<float>::Zero();
//   }
// }
