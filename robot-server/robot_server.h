#ifndef ROBOT_SERVER_H
#define ROBOT_SERVER_H

#include <ostream>
#include <vector>
#include <functional>
#include <random>
#include <memory>
#include <cstdint>
#include <stdint.h>

#include "actuator.h"
#include "utils.h"
#include "color.h"

// #include "third-party/adafruit-sensors/Adafruit_ADS1X15.h"
// #include "third-party/adafruit-sensors/Adafruit_INA260.h"

#include "third-party/cxxopts/cxxopts.hpp"
#include "third-party/nlohmann/json.hpp"


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

		size_t skip_count_max;

		bool skip_cal = false;
		bool skip_zero = false;
		bool skip_cfg_load = false;

		std::vector<uint8_t> moteus_ids;
		std::vector<uint8_t> moteus_buses;

		size_t num_actuators = 0;

		std::string moteus_cfg_path_prefix;
		std::vector<std::string> moteus_cfg_filenames;
		std::string moteus_cal_path_prefix;
		std::vector<std::string> moteus_cal_filenames;

		std::vector<float> gear_ratios;

		bool load_success = false;
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
	// returns true if any actuator is in trouble
	// or some other unsafe condition is present
	bool actuator_fault_check();
	inline RobotServerSettings& get_rs_set() {return rs_set_;}

	inline void set_time0(std::chrono::steady_clock::time_point t0) {time0_s_ = t0;}
	inline float get_time_prog() {return time_prog_s_;}

	// inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_a1_cmd() {
	// 	return a1_.get_curr_cmd();
	// }
	// inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_a2_cmd() {
	// 	return a2_.get_curr_cmd();
	// }
	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_actuator_cmd(size_t idx) {
		return actuator_ptrs_[idx]->get_curr_cmd();
	}
	
	inline void retrieve_replies(std::vector<
		mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& prev_replies) {
		
		for (auto a_ptr : actuator_ptrs_) {
			a_ptr->retrieve_reply(prev_replies);
		}
	}

	void sample_random();
	float filtered_random();

	void sample_sensors();
	std::string stringify_sensor_data();
	std::string stringify_sensor_data_headers();

	bool safety_check();

	void make_stop_all();

	std::map<int, int> create_servo_bus_map();

	inline size_t num_actuators() {return actuator_ptrs_.size();}

private:
	char cstr_buffer[128];
	
	std::vector<std::shared_ptr<Actuator>> actuator_ptrs_;
	std::ostream& datastream_;

	// Adafruit_ADS1015 ads_;
	// Adafruit_INA260 ina1_;
	// Adafruit_INA260 ina2_;

	SensorData sd_;

	// *** LOW LEVEL ***

	RobotServerSettings rs_set_;

	FSMState prev_state_ = FSMState::kIdle;
	FSMState curr_state_ = FSMState::kIdle;
	FSMState next_state_ = FSMState::kIdle;

	FSMState recovery_return_state_ = FSMState::kIdle;

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

	double fsm_program_timer_end = 0;
	double fsm_function_timer_end = 0;

	size_t playback_idx = 0;

	bool arrived_at_action_latch = false;
};

cxxopts::Options rs_opts();
RobotServer::RobotServerSettings parse_settings(cxxopts::ParseResult& rs_opts);

#endif