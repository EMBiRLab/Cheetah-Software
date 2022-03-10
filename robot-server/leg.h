#ifndef LEG_H
#define LEG_H

#include <ostream>
#include <vector>
#include <functional>

#include "actuator.h"
#include "urdf.h"
#include "utils.h"

#include "cxxopts/cxxopts.hpp"
#include "nlohmann/json.hpp"


class Leg {
public:

	// low level "gate" allowing the leg to operate
	enum FSMState : uint8_t {
		kIdle,
		kRunning, // allow other actions to happen, depending on ActionMode
		kRecovery,
		kQuitting,
	};

	enum ActionMode : uint8_t {
		kNone,
		kSafeZoneTest,
		kStand,
		kCyclicCrouch,
		kTorquePlayback,
		kOpenLoopTorque,
		kJump,
		kHapticsDemo,
	};

	struct LegSettings {

		LegSettings(cxxopts::ParseResult& leg_opts_in);

		float period_s;
		float duration_s;
		float gear_femur;
		float gear_tibia;
		float reset_height;

		uint8_t act_femur_id;
		uint8_t act_femur_bus;
		uint8_t act_tibia_id;
		uint8_t act_tibia_bus;

		uint8_t main_cpu;
		uint8_t can_cpu;

		cxxopts::ParseResult leg_opts;

		uint32_t status_period_us;

		float playback_vel_scale;
		float playback_trq_scale;

		int lpf_order;
		float lpf_cutoff_freq_Hz;

		ActionMode action_mode;
		float action_delay;

		std::string playback_file;
		float playback_delay;

		bool skip_cal = false;
		bool skip_zero = false;
		bool skip_cfg_load = false;
		std::string cfg_path = "/home/pi/embir-modular-leg/moteus-setup/moteus-cfg/a_gen.cfg";

	};

	struct CyclicCrouchSettings {
		float center_m = 0;
		float amplitude_m = 0;
		float frequency_Hz = 0;
		inline CyclicCrouchSettings() {}
		inline CyclicCrouchSettings(nlohmann::json cyclic_crouch_j) {
			center_m = cyclic_crouch_j["center_m"];
			amplitude_m = cyclic_crouch_j["amplitude_m"];
			frequency_Hz = cyclic_crouch_j["frequency_Hz"];
		}
	};

	struct OpenLoopTorqueSettings {
		float fem_a0 = 0;
		float fem_a1 = 0;
		float fem_b1 = 0;
		float fem_a2=0;
		float	fem_b2=0;
		float	fem_w=0;
		float	tibia_a0=0;
		float	tibia_a1=0;
		float	tibia_b1=0;
		float	tibia_a2=0;
		float	tibia_b2=0;
		float	tibia_w=0;
		bool time_restart=true;
		inline OpenLoopTorqueSettings() {}
		inline OpenLoopTorqueSettings(nlohmann::json openloop_torques_j) {
			fem_a0 = openloop_torques_j["fem_a0"];
			fem_a1 = openloop_torques_j["fem_a1"];
			fem_a2 = openloop_torques_j["fem_a2"];
			fem_b1 = openloop_torques_j["fem_b1"];
			fem_b2 = openloop_torques_j["fem_b2"];
			fem_w = openloop_torques_j["fem_w"];
			tibia_a0=openloop_torques_j["tibia_a0"];
			tibia_a1=openloop_torques_j["tibia_a1"];
			tibia_b1=openloop_torques_j["tibia_b1"];
			tibia_a2=openloop_torques_j["tibia_a2"];
			tibia_b2=openloop_torques_j["tibia_b2"];
			tibia_w=openloop_torques_j["tibia_w"];
			time_restart = openloop_torques_j["time_restart"]; 

		}
	};

	struct JumpSettings {
		float final_vel_m_s = 0;
		float initial_ext_m = 0;
		float final_ext_m = 0;
		float horizontal_offset_m = 0;

		float accel_s_m_s2 = 0;

		float time_0 = -1;
		inline JumpSettings() {}
		inline JumpSettings(nlohmann::json jump_j) {
			final_vel_m_s = jump_j["final_vel_m_s"];
			initial_ext_m = jump_j["initial_ext_m"];
			final_ext_m = jump_j["final_ext_m"];
			horizontal_offset_m = jump_j["horizontal_offset_m"];

			accel_s_m_s2 = 0.5 * (final_vel_m_s*final_vel_m_s) / (final_ext_m - initial_ext_m);
		
			std::cout 
				<< "\n\nfinal_vel_m_s = " << final_vel_m_s << ", "
				<< "initial_ext_m = " << initial_ext_m << ", "
				<< "final_ext_m = " << final_ext_m << "\n";
		}
	};

	class LegKinematics {
	public:
		struct JointAngles {
			float femur_angle_rad;
			float tibia_angle_rad;

			friend JointAngles operator+(JointAngles lhs, const JointAngles& rhs) {
				return {lhs.femur_angle_rad + rhs.femur_angle_rad,
								lhs.tibia_angle_rad + rhs.tibia_angle_rad};
			}
			JointAngles operator-() {return {-femur_angle_rad, -tibia_angle_rad};}
		};
		struct AlphaAngles {
			float a1_rad;
			float a2_rad;

			friend AlphaAngles operator+(AlphaAngles lhs, const AlphaAngles& rhs) {
				return {lhs.a1_rad + rhs.a1_rad,
								lhs.a2_rad + rhs.a2_rad};
			}
			AlphaAngles operator-() {return {-a1_rad, -a2_rad};}
		};
		struct Position {
			float y_m;
			float z_m;
			friend Position operator+(Position lhs, const Position& rhs) {
				return {lhs.y_m + rhs.y_m,
								lhs.z_m + rhs.z_m};
			}
			friend Position operator-(Position lhs, const Position& rhs) {
				return {lhs.y_m - rhs.y_m,
								lhs.z_m - rhs.z_m};
			}
			friend Position operator*(float lhs, const Position& rhs) {
				return {lhs*rhs.y_m,
								lhs*rhs.z_m};
			}
			friend Position operator*(const Position& lhs, float rhs) {
				return {rhs*lhs.y_m,
								rhs*lhs.z_m};
			}
			friend Position operator/(const Position& lhs, float rhs) {
				return {lhs.y_m/rhs,
								lhs.z_m/rhs};
			}
			Position operator-() {return {-y_m, -z_m};}
			float magnitude() {return std::sqrt(y_m*y_m + z_m*z_m);}
		};
		struct Jacobian {
			float J11, J12, J21, J22;
			Jacobian operator-() {return {-J11, -J12, -J21, -J22};}
			Jacobian transpose() {return {J11, J21, J12, J22};}
		};
		LegKinematics(URDF& leg_urdf);
		inline LegKinematics() {}

		AlphaAngles joint2alpha(JointAngles angles);
		JointAngles	alpha2joint(AlphaAngles angles);
		std::vector<Position> fk_vec(JointAngles angles);
		std::vector<Position> fk_2link(JointAngles angles);
		JointAngles ik_2link(Position pos);
		Jacobian jacobian_alpha(AlphaAngles angles);
		Jacobian jacobian_joint(JointAngles angles);

		JointAngles delta_task2joint(Position task, JointAngles angles);
		JointAngles grf2torque(Position grf, JointAngles angles);
		Position torque2grf(JointAngles torques, JointAngles angles);
		// JointAngles delta_task2joint(Position& task, JointAngles& angles);

	private:
		float l1_, l2_pll_, l2_perp_, l3_pll_, l3_perp_, r1_, r2_, gamma1_, gamma2_;
	};

	Leg(LegSettings& legset, std::ostream& datastream, std::string urdf_file);

	void iterate_fsm();

	void log_data();
	void log_headers();

	void print_status_update();

	inline FSMState get_fsm_curr_state() {return curr_state_;}
	inline FSMState get_fsm_next_state() {return next_state_;}
	inline bool is_ready_to_quit() {return ready_to_quit;}
	inline bool leg_fault() {return ((bool)act_femur_.fault() || (bool)act_tibia_.fault());}
	inline LegSettings& get_legset() {return legset_;}

	inline void set_time0(std::chrono::steady_clock::time_point t0) {time0_s_ = t0;}
	inline float get_time_prog() {return time_prog_s_;}

	// inline void share_commands(std::vector<
	// 	mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>& curr_commands,
	// 	std::vector<
	// 	mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>& prev_commands) {
	// 	act_femur_.share_curr_cmd(curr_commands[0]);
	// 	act_tibia_.share_curr_cmd(curr_commands[1]);
	// 	act_femur_.share_prev_cmd(prev_commands[0]);
	// 	act_tibia_.share_prev_cmd(prev_commands[1]);
	// }

	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_femur_cmd() {
		return act_femur_.get_curr_cmd();
	}
	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_tibia_cmd() {
		return act_tibia_.get_curr_cmd();
	}
	
	inline void retrieve_replies(std::vector<
		mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& prev_replies) {
		act_femur_.retrieve_reply(prev_replies);
		act_tibia_.retrieve_reply(prev_replies);
	}

	inline LegKinematics::JointAngles get_joint_angles() {
		return {act_femur_.get_position_rad(), act_tibia_.get_position_rad()};
	}

	LegKinematics leg_kinematics;
private:
	Actuator act_femur_;
	Actuator act_tibia_;
	std::ostream& datastream_;

	// *** LOW LEVEL ***

	// These are vectors of references to ServoCommand's that basically act like
	// vectors of ServoCommand's. This means that the command data is only stored
	// in the MoteusController objects and everyone else just has references to
	// that data.
	std::vector<
		std::reference_wrapper<
			mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>> commands_;
	std::vector<
		std::reference_wrapper<
			mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>> prev_commands_;

	LegSettings legset_;

	FSMState prev_state_ = FSMState::kIdle;
	FSMState curr_state_ = FSMState::kIdle;
	FSMState next_state_ = FSMState::kIdle;

	FSMState recovery_return_state_ = FSMState::kIdle;

  std::chrono::steady_clock::time_point time0_s_;
	float time_prog_s_ = 0;
	float time_prog_old_s_ = 0;
	float time_fcn_s_ = 0;
	float trial_time =-2;
	size_t cycles_ = 0;

	uint8_t quit_cycle = 0;
	uint8_t quit_cycle_thresh = 3;
	bool ready_to_quit = false;

	uint8_t recovery_cycle = 0;
	uint8_t recovery_cycle_thresh = 3;

	// *** HIGH LEVEL ***

	URDF leg_urdf_;
	ActionMode action_mode_ = ActionMode::kCyclicCrouch;

  nlohmann::json cyclic_crouch_j;
	CyclicCrouchSettings cyclic_crouch_s;

	nlohmann::json jump_j;
	JumpSettings jump_s;

	nlohmann::json openloop_torques_j;
	OpenLoopTorqueSettings openloop_torques_s;

	LowPassFilter lpf_femur_;
	LowPassFilter lpf_tibia_;

	std::vector<float> femur_trq;
	std::vector<float> tibia_trq;
	std::vector<float> femur_trq_temp; 
	std::vector<float> tibia_trq_temp; 
	size_t playback_idx = 0;

	bool arrived_at_action_latch = false;
	bool time_restart = true;

	void setup_playback();

	void run_action();
};

cxxopts::Options leg_opts();
Leg::LegSettings parse_settings(cxxopts::ParseResult& leg_opts);

#endif