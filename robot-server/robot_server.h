#ifndef ROBOT_SERVER_H
#define ROBOT_SERVER_H

#include <ostream>
#include <vector>
#include <functional>
#include <random>

#include "actuator.h"
#include "urdf.h"
#include "utils.h"
#include "color.h"

#include "sensors/Adafruit_ADS1X15.h"
#include "sensors/Adafruit_INA260.h"

#include "cxxopts/cxxopts.hpp"
#include "nlohmann/json.hpp"


class RobotServer {
public:

	struct SensorData {
		float ina1_current_A;
		float ina1_voltage_V;
		float ina1_power_W;
		float ina2_current_A;
		float ina2_voltage_V;
		float ina2_power_W;
	};

	// low level "gate" allowing the leg to operate
	enum FSMState : uint8_t {
		kIdle,
		kStartup,
		kRunning, // allow other actions to happen, depending on TestMode
		kRecovery,
		kQuitting,
	};

	struct RobotServerSettings {

		RobotServerSettings(cxxopts::ParseResult& rs_opts_in);

		float period_s;
		float duration_s;

		uint8_t main_cpu;
		uint8_t can_cpu;

		cxxopts::ParseResult rs_opts;

		uint32_t status_period_us;

		bool skip_cal = false;
		bool skip_zero = false;
		bool skip_cfg_load = false;

	};

	class Timer {
		private:
		float current_time = 0;
		float time_end = 0;

		public:
		inline Timer() {}
		inline void update_time(float now) {
			current_time = now;
		}
		inline void start_timer(float duration) {
			time_end = current_time + duration;
		}
		inline bool timer_done() {
			return current_time > time_end;
		}
		inline float time_left() {
			return time_end-current_time;
		}
	};

	RobotServer(RobotServerSettings& rs_set,
		std::ostream& datastream);

	void iterate_fsm();

	void log_data();
	void log_headers();

	void print_status_update();

	inline FSMState get_fsm_curr_state() {return curr_state_;}
	inline FSMState get_fsm_next_state() {return next_state_;}
	inline bool is_ready_to_quit() {return ready_to_quit;}
	// returns true if either actuator is in trouble
	// or some other unsafe condition is present
	inline bool dynamometer_fault() {
		return ((bool)a1_.fault() || (bool)a2_.fault()) || !dynamometer_safe;}
	inline RobotServerSettings& get_rs_set() {return rs_set_;}

	inline void set_time0(std::chrono::steady_clock::time_point t0) {time0_s_ = t0;}
	inline float get_time_prog() {return time_prog_s_;}

	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_a1_cmd() {
		return a1_.get_curr_cmd();
	}
	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_a2_cmd() {
		return a2_.get_curr_cmd();
	}
	
	inline void retrieve_replies(std::vector<
		mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& prev_replies) {
		a1_.retrieve_reply(prev_replies);
		a2_.retrieve_reply(prev_replies);
	}

	void sample_random();
	float filtered_random();

	void sample_sensors();
	std::string stringify_sensor_data();
	std::string stringify_sensor_data_headers();

	void iterate_durability_fsm();
	void iterate_efficiency_fsm();

	inline void swap_actuators() {std::swap(a_driving_ptr, a_loading_ptr);}
	void load_playback_data(std::string file);

	bool safety_check();

	inline float get_a1_kt() {return a1_.get_torque_constant();}
	inline float get_a2_kt() {return a2_.get_torque_constant();}

	inline float get_ts_torque() {return sd_.torque_Nm;}

private:
	char cstr_buffer[128];
	
	Actuator a1_;
	Actuator a2_;
	Actuator* a_driving_ptr = &a1_;
	Actuator* a_loading_ptr = &a2_;
	int num_swaps = 0;
	std::ostream& datastream_;

	Adafruit_ADS1015 ads_;
	Adafruit_INA260 ina1_;
	Adafruit_INA260 ina2_;

	SensorData sd_;

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

	RobotServerSettings rs_set_;

	FSMState prev_state_ = FSMState::kIdle;
	FSMState curr_state_ = FSMState::kIdle;
	FSMState next_state_ = FSMState::kIdle;

	FSMState recovery_return_state_ = FSMState::kIdle;

	float encoder_offset = 0;
	bool encoder_offset_set = false;

	std::chrono::steady_clock::time_point time0_s_;
	float time_prog_s_ = 0;
	float time_prog_old_s_ = 0;
	float time_fcn_s_ = 0;
	size_t cycles_ = 0;

	uint8_t quit_cycle = 0;
	uint8_t quit_cycle_thresh = 3;
	bool ready_to_quit = false;

	uint8_t recovery_cycle = 0;
	uint8_t recovery_cycle_thresh = 10;
	size_t num_recoveries = 0;

	// *** HIGH LEVEL ***

	// URDF leg_urdf_;
	TestMode test_mode_ = TestMode::kNone;

	nlohmann::json grp_j;
	GRPSettings grp_s;
	std::random_device rd_;
	std::uniform_real_distribution<> realdist;
	float random_sample = 0;
	LowPassFilter lpf_grp_;

	nlohmann::json durability_j;
	DurabilitySettings durability_s;
	// uint32_t sweep_idx = 0;
	size_t num_efficiency_sweeps = 0;
	size_t vel_damp_idx = 0;
	size_t num_vel_sweeps = 0;
	size_t trq_damp_idx = 0;
	size_t num_trq_sweeps = 0;

	float durability_cycle_s = 0;
	bool initial_follow_delay = false;
	
	double fsm_program_timer_end = 0;
	double fsm_function_timer_end = 0;

	Timer durability_timer;
	Timer efficiency_timer;
	
	DurabilityTestState dts = DurabilityTestState::kDurabilityIdle;
	DurabilityTestState dts_after_idle = DurabilityTestState::kDurabilityFollow;
		

	nlohmann::json efficiency_j;
	EfficiencySettings efficiency_s;

	EfficiencyTestState ets = EfficiencyTestState::kEfficiencyIdle;
	size_t efficiency_vel_idx = 0;
	size_t efficiency_torque_idx = 0;
	float cond_vel = 0;
	float cond_torque = 0;
	size_t num_tested = 0;

	bool efficiency_done = false;

	bool positive_work = true;
	inline bool exceeds_limit(float torque, float velocity) {
		auto &e_s = efficiency_s;
		float velocity_lim = (e_s.nom_VDC / sqrt(3)) / (e_s.KT*e_s.ratio);
		return torque > (velocity_lim-velocity)*e_s.limit_slope_Nm_rad_s + 0.5
			|| velocity > velocity_lim;
	}
	inline void iterate_condition() {
		efficiency_torque_idx++;
		if (efficiency_torque_idx >= efficiency_s.torques_Nm.size()) {
			efficiency_torque_idx = 0;
			efficiency_vel_idx++;
		}
		if (efficiency_vel_idx >= efficiency_s.velocities_rad_s.size()) {
			std::cout << "\nefficiency sweep complete!" << std::endl;
			efficiency_done = true;
		}
		num_tested++;
	}
	inline void reset_efficiency() {
		efficiency_torque_idx = 0;
		efficiency_vel_idx = 0;
		ets = EfficiencyTestState::kEfficiencyIdle;
		efficiency_done = false;
	}

	std::vector<float> playback_vel;
	std::vector<float> playback_trq;

	nlohmann::json step_j;
	StepSettings step_s;

	nlohmann::json torque_step_j;
	TorqueStepSettings torque_step_s;

	nlohmann::json safety_j;
	SafetySettings safety_s;
	bool overtemp_latch = false;
	bool dynamometer_safe = true;

	LowPassFilter lpf_a1_;
	LowPassFilter lpf_a2_;

	std::vector<float> a1_trq;
	std::vector<float> a2_trq;
	std::vector<float> a1_trq_temp; 
	std::vector<float> a2_trq_temp; 
	size_t playback_idx = 0;

	bool arrived_at_action_latch = false;

	void setup_playback();

	void run_test();
};

cxxopts::Options rs_opts();
RobotServer::RobotServerSettings parse_settings(cxxopts::ParseResult& rs_opts);

#endif