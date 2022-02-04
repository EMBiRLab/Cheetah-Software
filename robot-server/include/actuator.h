#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "moteus_controller.h"

#define PI 3.1415926

class Actuator : public MoteusController {
public:
	Actuator(id_t id, uint8_t bus, float gear_ratio, float trq_efficiency);
	inline float get_gear_ratio() {return gear_ratio_;}
	inline void set_gear_ratio(float new_ratio) {gear_ratio_ = new_ratio;}
	inline void make_act_position(float pos_rad, float ff_trq_Nm) {
		make_mot_position(
			pos_rad/(2*PI),
			kp_, kd_,
			ff_trq_Nm/(trq_efficiency_));
	}
	inline void make_act_velocity(float vel_rad_s, float ff_trq_Nm) {
		make_mot_velocity(
			vel_rad_s/(2*PI),
			kp_, kd_,
			ff_trq_Nm/(trq_efficiency_));
	}
	inline void make_act_torque(float trq_Nm) {
		make_mot_torque(trq_Nm/(trq_efficiency_));
	}
	inline void make_act_full_pos(float pos_rad, float vel_rad_s, float trq_Nm) {
		make_mot_full_pos(
			pos_rad/(2*PI),
			vel_rad_s/(2*PI),
			trq_Nm/(trq_efficiency_),
			kp_, kd_);
	}
	inline void set_kp_scale(float kp_in) {
		kp_ = kp_in;
	}
	inline void set_kd_scale(float kd_in) {
		kd_ = kd_in;
	}

	inline float get_position_rad() {return prev_reply_.result.position * 2 * PI;}
	inline float get_position_cmd_rad() {return curr_cmd_.position.position * 2 * PI;}
	inline float get_velocity_rad_s() {return prev_reply_.result.velocity * 2 * PI;}
	inline float get_velocity_cmd_rad_s() {return curr_cmd_.position.velocity * 2 * PI;}
	inline float get_torque_Nm() {return prev_reply_.result.torque;}
	inline float get_torque_cmd_Nm() {return curr_cmd_.position.feedforward_torque;}

	std::string stringify_actuator();
	std::string stringify_actuator_header();
private:
	float gear_ratio_ = 1;
	float trq_efficiency_ = 1;
	float kp_ = 1;
	float kd_ = 1;
};

#endif