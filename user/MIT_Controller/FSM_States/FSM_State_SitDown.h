#ifndef FSM_STATE_SITDOWN_H
#define FSM_STATE_SITDOWN_H

#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

/**
 *
 */
template <typename T>
class FSM_State_SitDown : public FSM_State<T> {
 public:
  FSM_State_SitDown(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

 private:

  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  T last_height_command = 0;

  Vec3<T> _ini_body_pos;
  Vec3<T> _des_body_pos;
  Vec3<T> _ini_body_ori_rpy;
  T _body_weight;

  void SitDownStep();

  // Keep track of the control iterations
  int iter = 0;
  std::vector< Vec3<T> > _ini_foot_pos;
  std::vector< Vec3<T> > _err_foot_pos;
  Vec3<T> _avg_foot_err;

  bool standing_up = true;
  bool stood_up = false;
  int standup_iter = 0;
  int button_count = 0;

  std::vector< Vec3<T> > transition_cmd;
  std::vector< Vec3<T> > transition_pos;
};

#endif  // FSM_STATE_SitDown_H
