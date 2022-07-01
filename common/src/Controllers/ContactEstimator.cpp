/*! @file ContactEstimator.cpp
 *  @brief All Contact Estimation Algorithms
 *
 *  This file will contain all contact detection algorithms. For now, it just
 * has a pass-through algorithm which passes the phase estimation to the state
 * estimator.  This will need to change once we move contact detection to C++
 */

#include "Controllers/ContactEstimator.h"

// template <typename T>
// void ContactEstimator<T>::run() {

//     // Re-zero the contact estimate
//     this->_stateEstimatorData.result->contactEstimate = Vec4<T>::Zero();

//     // iterate through the legs, and if the grf for a leg is great 
//     // enough in magnitude, then we can say that it's in contact with the ground
//     for (int leg = 0; leg < 4; leg++) {

//         // TODO: I AM ASSUMING USE OF WALKING2 GAIT. THAT IS BAD. I NEED TO GENERALIZE THIS CODE!!!
//         if ((*this->_stateEstimatorData.contactPhase)(leg) < .075 || (*this->_stateEstimatorData.contactPhase)(leg) > 0.225) {
//             // calculate the GRF of the leg so that we can check if its greather than our threshold
//             Mat3<T> J_transpose = this->_stateEstimatorData.legControllerData[leg].J.transpose();
//             Vec3<T> leg_grf =  J_transpose.inverse() * this->_stateEstimatorData.legControllerData[leg].tauEstimate;

//             if (leg_grf[2] < _threshold) {
//                 this->_stateEstimatorData.result->contactEstimate(leg) = .5; // 0.5 for now to match rest of code
//             }
//         }




//         // this->_stateEstimatorData.result->contactEstimate =
//         //     *this->_stateEstimatorData.contactPhase;
        
//     }

//     // std::cout << "Contact estimate is: " << this->_stateEstimatorData.result->contactEstimate << "\n";
// }

// template class ContactEstimator<float>;
// template class ContactEstimator<double>;
