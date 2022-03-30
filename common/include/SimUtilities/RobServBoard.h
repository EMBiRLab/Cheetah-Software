/*! @file ROBSERVBoard.h
 *  @brief Robot Server "Board" Code, used to simulate the Robot Server.
 *
 *  This is mostly a copy of what the Robot Server does
 */

#ifndef PROJECT_ROBSERVBOARD_H
#define PROJECT_ROBSERVBOARD_H

#include "cTypes.h"

/*!
 * Command to Robot Server
 */
struct RobServCommand {
  // TODO: Restructure this to actually match with robot server command structure
  float q_des[12];
  // float q_des_hip[4];
  // float q_des_knee[4];

  float qd_des[12];
  // float qd_des_hip[4];
  // float qd_des_knee[4];

  float kp_joint[12];
  // float kp_hip[4];
  // float kp_knee[4];

  float kd_joint[12];
  // float kd_hip[4];
  // float kd_knee[4];

  float tau_ff[12];
  // float tau_ff_hip[4];
  // float tau_ff_knee[4];

//   int32_t flags[4];
};

/*!
 * Data from robot server
 */
struct RobServData {
  // TODO: Restructure this to actually match with robot server response structure
  float q[12];
  // float q_hip[4];
  // float q_knee[4];
  float qd[12];
  // float qd_hip[4];
  // float qd_knee[4];
  float tau_est[12];
  // int32_t flags[4];
  uint8_t fsm_state;
};

/*!
 * Robot server control logic
 */
class RobServBoard {
 public:
  RobServBoard() {}
  // void init(float side_sign, s32 board);
  void init(float sides[]);
  void run();
  void resetData();
  void resetCommand();
  RobServCommand* cmd = nullptr;
  RobServData* data = nullptr;
  float torque_out[12];

 private:
  float side_sign[4];
  // s32 board_num;
  // Soft limits per leg
  const float max_torque[12] = {17.f, 17.f, 26.f, 17.f, 17.f, 26.f, 17.f, 17.f, 26.f, 17.f, 17.f, 26.f};  // TODO CHECK WITH BEN
  const float wimp_torque[12] = {6.f, 6.f, 6.f, 6.f, 6.f, 6.f, 6.f, 6.f, 6.f, 6.f, 6.f, 6.f};    // TODO CHECK WITH BEN
  const float disabled_torque[12] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  const float q_limit_p[12] = {1.5f, 5.0f, 0.f, 1.5f, 5.0f, 0.f, 1.5f, 5.0f, 0.f, 1.5f, 5.0f, 0.f};
  const float q_limit_n[12] = {-1.5f, -5.0f, 0.f, -1.5f, -5.0f, 0.f, -1.5f, -5.0f, 0.f, -1.5f, -5.0f, 0.f};
  const float kp_softstop = 100.f;
  const float kd_softstop = 0.4f;
  s32 iter_counter = 0;
};

#endif  // PROJECT_ROBSERVBOARD_H
