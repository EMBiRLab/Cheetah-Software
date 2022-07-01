/*! @file ContactEstimator.h
 *  @brief All Contact Estimation Algorithms
 *
 *  This file will contain all contact detection algorithms. For now, it just
 * has a pass-through algorithm which passes the phase estimation to the state
 * estimator.  This will need to change once we move contact detection to C++
 *
 *  We also still need to establish conventions for "phase" and "contact".
 */

#ifndef PROJECT_CONTACTESTIMATOR_H
#define PROJECT_CONTACTESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"

// BELOW IS THE ORIGINAL MIT ContactEstimator. COULD POTENTIALLY TURN THIS ESTIMATOR INTO A CHEATERESTIMATOR
/*!
 * A "passthrough" contact estimator which returns the expected contact state
 */
template <typename T>
class ContactEstimator : public GenericEstimator<T> {
 public:

  /*!
   * Set the estimated contact by copying the exptected contact state into the
   * estimated contact state
   */
  virtual void run() {
    this->_stateEstimatorData.result->contactEstimate =
        *this->_stateEstimatorData.contactPhase;
  }

  /*!
   * Set up the contact estimator
   */
  virtual void setup() {}
};

// /*!
//  * A foot contact estimator that is based on the torque signals that are returned by the legs
//  */
// template <typename T>
// class ContactEstimator : public GenericEstimator<T> {
//   public:
//   virtual void run();
//   virtual void setup() {
//     // store the robot mass to calculate a threshold for contact estimation
//     _robotMass = this->_stateEstimatorData.legControllerData->quadruped->_totalMass;
//     // threshold for contact is 20% of robot mass
//     _threshold = (_robotMass * 9.81) * 0.2; // TODO: MAKE THIS THRESHOLD PERCENTAGE A YAML FILE PARAMETER
//   }

//   protected:
//   T _robotMass;
//   T _threshold;

// };

#endif  // PROJECT_CONTACTESTIMATOR_H
