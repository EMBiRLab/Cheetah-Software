/*! @file MuadQuad.h
 *  @brief Utility function to build a MuadQuad Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD. <--outdated description. 
 * 
 * TODO: update to muadquad constants!!!
 */

#ifndef PROJECT_MUADQUAD_H
#define PROJECT_MUADQUAD_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMuadQuad() {
  Quadruped<T> muadquad;
  muadquad._robotType = RobotType::MUADQUAD;

  muadquad._bodyMass = 3.3;
  muadquad._bodyLength = 0.2 * 2;
  muadquad._bodyWidth = 0.1 * 2;
  muadquad._bodyHeight = 0.035;

  muadquad._abadGearRatio = 7.5;
  muadquad._hipGearRatio = 7.5;
  muadquad._kneeGearRatio = 7.5;
  muadquad._abadLinkLength = 0.025;
  muadquad._hipLinkLength = 0.20;
  //muadquad._kneeLinkLength = 0.175;
  //muadquad._maxLegLength = 0.384;
  muadquad._kneeLinkY_offset = 0.0095;
  //muadquad._kneeLinkLength = 0.20;
  muadquad._kneeLinkLength = 0.2;
  muadquad._maxLegLength = 0.38; // we think this is maybe safety to avoid singularities


  muadquad._motorTauMax = 1.6f;
  muadquad._batteryV = 31;
  muadquad._motorKT = .105;  // this is flux linkage * pole pairs
  muadquad._motorR = 0.7;
  muadquad._jointDamping = .01;
  muadquad._jointDryFriction = .2;
  //muadquad._jointDamping = .0;
  //muadquad._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 3, 0, 0, 0, 3, 0, 0, 0, 6;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(muadquad._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  muadquad._abadInertia = abadInertia;
  muadquad._hipInertia = hipInertia;
  muadquad._kneeInertia = kneeInertia;
  muadquad._abadRotorInertia = rotorInertiaX;
  muadquad._hipRotorInertia = rotorInertiaY;
  muadquad._kneeRotorInertia = rotorInertiaY;
  muadquad._bodyInertia = bodyInertia;

  // locations
  muadquad._abadRotorLocation = Vec3<T>(muadquad._bodyLength, muadquad._bodyWidth, -0.068)*0.5;
  muadquad._abadLocation = Vec3<T>(muadquad._bodyLength, muadquad._bodyWidth, -0.068) * 0.5;
//   muadquad._hipLocation = Vec3<T>(-muadquad._abadLinkLength, 0, 0);
  muadquad._hipLocation = Vec3<T>(0, muadquad._abadLinkLength, 0);
  muadquad._hipRotorLocation = Vec3<T>(0, 0, 0);
  muadquad._kneeLocation = Vec3<T>(0, 0, -muadquad._hipLinkLength);
  muadquad._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return muadquad;
}

#endif  // PROJECT_MUADQUAD_H
