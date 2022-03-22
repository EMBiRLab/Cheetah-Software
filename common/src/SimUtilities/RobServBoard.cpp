/*! @file RobServBoard.cpp
 *  @brief RobServ Board Code, used to simulate the RobServBoard.
 */

#include <stdio.h>
#include <algorithm>
#include "SimUtilities/RobServBoard.h"

/*!
 * RobServ board setup (per board)
 */
void RobServBoard::init(float sides[]) {
  for(int leg = 0; leg < 4; leg++){
    side_sign[leg] = sides[leg];
  }
}

/*!
 * Reset all data for the board
 */
void RobServBoard::resetData() {
  if (data == nullptr) {
    printf(
        "[ERROR: RobServBoard] reset_robserv_board_data called when "
        "muadquadlcm_robserv_data_t* was null\n");
    return;
  }

  std::fill(data->q, data->q+12, 0.f);
  std::fill(data->qd, data->qd+12, 0.f);
  std::fill(data->tau_est, data->tau_est+12, 0.f);
  data->fsm_state = 0;
}

/*!
 * Reset all commands for the board
 */
void RobServBoard::resetCommand() {
  if (cmd == nullptr) {
    printf(
        "[ERROR: RobServBoard] reset_robserv_board_command called when "
        "muadquadlcm_robserv_command_t* was null\n");
    return;
  }

  std::fill(cmd->q_des, cmd->q_des+12, 0.f);
  std::fill(cmd->qd_des, cmd->qd_des+12, 0.f);
  std::fill(cmd->tau_ff, cmd->tau_ff+12, 0.f);
  std::fill(cmd->kp_joint, cmd->kp_joint+12, 0.f);
  std::fill(cmd->kd_joint, cmd->kd_joint+12, 0.f);
}

/*!
 * Run robot server board control
 */
void RobServBoard::run() {
  iter_counter++;
  if (cmd == nullptr || data == nullptr) {
    printf(
        "[ERROR: RobServBoard] run_robserv_board_iteration called with null "
        "command or data!\n");
    // torque_out[0] = 0.f;
    // torque_out[1] = 0.f;
    // torque_out[2] = 0.f;
    std::fill(torque_out, torque_out+12, 0.f);
    return;
  }

  /// Check abad softstop ///
  for (int leg = 0; leg < 4; leg++){
    if (data->q[leg*3] > q_limit_p[0]) {
      torque_out[leg*3] = kp_softstop * (q_limit_p[0] - data->q[leg*3]) -
                          kd_softstop * (data->qd[leg*3]) +
                          cmd->tau_ff[leg*3];
    } else if (data->q[leg*3] < q_limit_n[0]) {
      torque_out[leg*3] = kp_softstop * (q_limit_n[0] - data->q[leg*3]) -
                          kd_softstop * (data->qd[leg*3]) +
                          cmd->tau_ff[leg*3];
    } else {
      torque_out[leg*3] = cmd->kp_joint[leg*3] * (cmd->q_des[leg*3] - data->q[leg*3]) +
                          cmd->kd_joint[leg*3] * (cmd->qd_des[leg*3] - data->qd[leg*3]) +
                          cmd->tau_ff[leg*3];
    }
  }

  /// Check hip softstop ///
  for (int leg = 0; leg < 4; leg++){
    if (data->q[leg*3+1] > q_limit_p[1]) {
      torque_out[leg*3+1] = kp_softstop * (q_limit_p[1] - data->q[leg*3+1]) -
                            kd_softstop * (data->qd[leg*3+1]) +
                            cmd->tau_ff[leg*3+1];
    } else if (data->q[leg*3+1] < q_limit_n[1]) {
      torque_out[leg*3+1] = kp_softstop * (q_limit_n[1] - data->q[leg*3+1]) -
                            kd_softstop * (data->qd[leg*3+1]) +
                            cmd->tau_ff[leg*3+1];
    } else {
      torque_out[leg*3+1] = cmd->kp_joint[leg*3+1] * (cmd->q_des[leg*3+1] - data->q[leg*3+1]) +
                            cmd->kd_joint[leg*3+1] * (cmd->qd_des[leg*3+1] - data->qd[leg*3+1]) +
                            cmd->tau_ff[leg*3+1];
    }
  }

  /// No knee softstop right now ///
  for (int leg = 0; leg < 4; leg++){
    torque_out[leg*3+2] = cmd->kp_joint[leg*3+2] * (cmd->q_des[leg*3+2] - data->q[leg*3+2]) +
                          cmd->kd_joint[leg*3+2] * (cmd->qd_des[leg*3+2] - data->qd[leg*3+2]) +
                          cmd->tau_ff[leg*3+2];
  }

  const float* torque_limits = disabled_torque;

  // if (cmd->flags[board_num] & 0b1) {
  //   if (cmd->flags[board_num] & 0b10)
  //     torque_limits = wimp_torque;
  //   else
  //     torque_limits = max_torque;
  // }
  torque_limits = max_torque;

  for (int i = 0; i < 12; i++) {
    if (torque_out[i] >  torque_limits[i]) torque_out[i] =  torque_limits[i];
    if (torque_out[i] < -torque_limits[i]) torque_out[i] = -torque_limits[i];
  }

}
