#ifndef DYNAMOMETER_H
#define DYNAMOMETER_H

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


class Dynamometer {
public:
  
  struct SensorData {
    float torque_Nm;
    float ina1_current_A;
    float ina1_voltage_V;
    float ina1_power_W;
    float ina2_current_A;
    float ina2_voltage_V;
    float ina2_power_W;
    float temp1_C;
    float temp2_C;
  };

  // interpret command line options to settings class
  enum TorqueSensor : uint8_t {
    kTRD605_18,
    kTRS605_5
  };
	// low level "gate" allowing the leg to operate
	enum FSMState : uint8_t {
		kIdle,
		kStartup,
		kRunning, // allow other actions to happen, depending on TestMode
		kRecovery,
		kQuitting,
	};

	enum TestMode : uint8_t {
    kTorqueConstant,
    kGRP,
    kDirectDamping,
    kTorqueVelSweep,
		kEfficiency,
    kDurability,
    kStep,
		kTorqueStep,
    kManual,
    kNone
  };

	enum DurabilityTestState : uint8_t {
    kDurabilityIdle,
    kDurabilityFollow,
    kDurabilityGRP,
    kDurabilityEfficiency,
		kDurabilityTorqueDamping,
		kDurabilityVelocityDamping,
    kDurabilityNone
  };

	enum EfficiencyTestState : uint8_t {
    kEfficiencyIdle,
    kEfficiencyRampUpVel,
		kEfficiencyLagUp,
		kEfficiencyRampUpTorque,
		kEfficiencyHoldCondition,
		kEfficiencyRampDownTorque,
		kEfficiencyLagDown,
		kEfficiencyRampDownVelocity,
    kEfficiencyNone
  };

  struct GRPSettings {
		float lpf_order_ = 0;
		float lpf_fc_ = 0;
		float torque_amplitude_Nm = 0;
		float vel_amplitude_rad_s = 0;
		float pos_amplitude_rad = 0;
		std::string control_mode;
		enum GRPControlMode : uint8_t {
			kGRPTorque,
			kGRPVelocity,
			kGRPPosition,
			kGRPCurrent,
			kGRPNone
		};
		GRPControlMode grp_control_mode = kGRPNone;
		float amplitude = 0;
		inline GRPSettings() {}
		inline GRPSettings(nlohmann::json grp_j) {
			lpf_order_ = grp_j["butterworth_order"];
      lpf_fc_ = grp_j["cutoff_frequency_Hz"];
      torque_amplitude_Nm = grp_j["torque_amplitude_Nm"];
      vel_amplitude_rad_s = grp_j["vel_amplitude_rad_s"];
      pos_amplitude_rad = grp_j["pos_amplitude_rad"];
			control_mode = grp_j["control_mode"];
			std::cout << "using GRP control mode " << control_mode << "; ";
			if (control_mode == "torque") {
				grp_control_mode = kGRPTorque; amplitude = torque_amplitude_Nm;
			}
			else if (control_mode == "velocity") {
				grp_control_mode = kGRPVelocity; amplitude = vel_amplitude_rad_s;
			}
			else if (control_mode == "position") {
				grp_control_mode = kGRPPosition; amplitude = pos_amplitude_rad;
			}

			else if (control_mode == "current") {
				grp_control_mode = kGRPCurrent; amplitude = torque_amplitude_Nm;
			}
			else {
				// std::cout << "\tcontrol mode not recognized!!\n";
				throw std::runtime_error("Control mode not recognized! Exiting...");
			}
			std::cout << "GRP amplitude is " << amplitude << "... ";
		}
	};

  struct DurabilitySettings {
    float follow_duration_s = 0;
    float GRP_duration_s = 0;
    float condition_duration_s = 0;
		float intercondition_idle_duration_s = 0;
		float interphase_idle_duration_s = 10;
    std::vector<float> velocities_rad_s;
    std::vector<float> torques_Nm;
		float velocity_max_rad_s = 0;
  	float torque_max_Nm = 0;

		float damping_total_duration_s = 0;
		inline DurabilitySettings() {}
		inline DurabilitySettings(nlohmann::json durability_j) {
			follow_duration_s = durability_j["follow_duration_s"];
      GRP_duration_s = durability_j["GRP_duration_s"];
      condition_duration_s = durability_j["condition_duration_s"];
      interphase_idle_duration_s = durability_j["interphase_idle_duration_s"];
			intercondition_idle_duration_s = durability_j["intercondition_idle_duration_s"];
      velocities_rad_s = durability_j["velocities_rad_s"].get<std::vector<float>>();
      torques_Nm = durability_j["torques_Nm"].get<std::vector<float>>();
			velocity_max_rad_s = durability_j["velocity_max_rad_s"];
			torque_max_Nm = durability_j["torque_max_Nm"];

			damping_total_duration_s
				= (torques_Nm.size()+velocities_rad_s.size()) *
				(intercondition_idle_duration_s + condition_duration_s);
			
			std::cout << torques_Nm.size() << " trqs, "
				<< velocities_rad_s.size() << " vels, "
				<< "damping time = " 
				<< damping_total_duration_s << " s" << std::endl;
		}
	};

	struct EfficiencySettings {
		float idle_duration_s = 0;
		float ramp_up_vel_duration_s = 0;
		float lag_up_duration_s = 0;
		float ramp_up_torque_duration_s = 0;
		float hold_condition_duration_s = 0;
		float ramp_down_torque_duration_s = 0;
		float lag_down_duration_s = 0;
		float ramp_down_vel_duration_s = 0;
		std::vector<float> velocities_rad_s;
		std::vector<float> torques_Nm;
		float velocity_max_rad_s = 0;
		float torque_max_Nm = 0;
		float limit_slope_Nm_rad_s = 0;
		float KT = 0;	
		float nom_VDC = 0;
		float ratio = 0;

		float efficiency_sweep_duration_s = 0;

		inline EfficiencySettings() {}
		inline EfficiencySettings(nlohmann::json efficiency_j) {
			idle_duration_s = efficiency_j["idle_duration_s"];
			ramp_up_vel_duration_s = efficiency_j["ramp_up_vel_duration_s"];
			lag_up_duration_s = efficiency_j["lag_up_duration_s"];
			ramp_up_torque_duration_s = efficiency_j["ramp_up_torque_duration_s"];
			hold_condition_duration_s = efficiency_j["hold_condition_duration_s"];
			ramp_down_torque_duration_s = efficiency_j["ramp_down_torque_duration_s"];
			lag_down_duration_s = efficiency_j["lag_down_duration_s"];
			ramp_down_vel_duration_s = efficiency_j["ramp_down_vel_duration_s"];
			velocities_rad_s = efficiency_j["velocities_rad_s"].get<std::vector<float>>();
			torques_Nm = efficiency_j["torques_Nm"].get<std::vector<float>>();
			velocity_max_rad_s = efficiency_j["velocity_max_rad_s"];
			torque_max_Nm = efficiency_j["torque_max_Nm"];
			limit_slope_Nm_rad_s = efficiency_j["limit_slope_Nm_rad_s"];
			KT = efficiency_j["KT"];
			ratio = efficiency_j["ratio"];
			nom_VDC = efficiency_j["nom_VDC"];
			
			efficiency_sweep_duration_s
				= torques_Nm.size()*velocities_rad_s.size() *
				(idle_duration_s +
				ramp_up_vel_duration_s +
				lag_up_duration_s +
				ramp_up_torque_duration_s +
				hold_condition_duration_s +
				ramp_down_torque_duration_s +
				lag_down_duration_s +
				ramp_down_vel_duration_s) * 2;

			std::cout << torques_Nm.size() << " trqs, "
				<< velocities_rad_s.size() << " vels, "
				<< torques_Nm.size()*velocities_rad_s.size() << " conds, sweep time = " 
				<< efficiency_sweep_duration_s << " s" << std::endl;
			
			
			return;
		}
	};

  struct StepSettings {
		float step_mag_Nm = 0;
		float step_temp_ceiling_C = 0;
		float step_temp_floor_C = 0;
	  bool step_temp_latch = false;
		inline StepSettings() {}
		inline StepSettings(nlohmann::json step_j) {
			step_mag_Nm = step_j["torque_step_mag_Nm"];
      step_temp_ceiling_C = step_j["temp_ceiling_C"];
      step_temp_floor_C = step_j["temp_floor_C"];
		}
	};

	struct TorqueStepSettings {
		float step_time_s = 0;
		float pause_time_s = 0;
		bool loop = false;
    std::vector<float> torques_Nm;
		size_t torque_idx = 0;
		float next_transition_s = 0;
		bool is_paused = false;
		inline TorqueStepSettings() {}
		inline TorqueStepSettings(nlohmann::json torque_step_j) {
			step_time_s = torque_step_j["step_time_s"];
      pause_time_s = torque_step_j["pause_time_s"];
      loop = torque_step_j["loop"];
      torques_Nm = torque_step_j["torques_Nm"].get<std::vector<float>>();
		}
	};

  struct SafetySettings {
		float max_motor_temp_C_ = 0;
    float max_housing_temp_C_ = 0;
    float trs605_5_max_torque_Nm_ = 0;
    float trd605_18_max_torque_Nm_ = 0;
    float actuator_torque_disparity_ratio_ = 0;
		float ts_max_torque_Nm_ = 0;
		float max_bus_VDC = 0;
		float min_bus_VDC = 0;

    std::vector<float> motor_temp_buffer; 
    std::vector<float> housing_temp_buffer; 
    std::vector<float> disparity_buffer;
		inline SafetySettings() {}
		inline SafetySettings(nlohmann::json safety_j) {
			max_motor_temp_C_ = safety_j["max_motor_temp_C"];
      max_housing_temp_C_ = safety_j["max_housing_temp_C"];
      motor_temp_buffer.resize(8);
      housing_temp_buffer.resize(8);
      trs605_5_max_torque_Nm_ = safety_j["trs605-5_max_torque_Nm"];
      trd605_18_max_torque_Nm_ = safety_j["trd605-18_max_torque_Nm"];
      actuator_torque_disparity_ratio_ = safety_j["actuator_torque_disparity_ratio"];
			max_bus_VDC = safety_j["max_bus_VDC"];
			min_bus_VDC = safety_j["min_bus_VDC"];
      disparity_buffer.resize(8);
    }
	};

	struct DynamometerSettings {

		DynamometerSettings(cxxopts::ParseResult& leg_opts_in);

		float period_s;
		float duration_s;
		float gear_a1;
		float gear_a2;

		uint8_t a1_id;
		uint8_t a1_bus;
		uint8_t a2_id;
		uint8_t a2_bus;

		uint8_t main_cpu;
		uint8_t can_cpu;

		cxxopts::ParseResult dyn_opts;

		uint32_t status_period_us;

		float playback_vel_scale;
		float playback_trq_scale;

		int lpf_order;
		float lpf_cutoff_freq_Hz;
    uint32_t grp_sampling_period_us;

		TestMode test_mode;
		float test_delay;
		TorqueSensor tqsen;

		std::string playback_file;
		float playback_delay;
		float durability_resume;

		bool skip_cal = false;
		bool skip_zero = false;
		bool skip_cfg_load = false;
		std::string cfg_path = "/home/pi/embir-modular-leg/moteus-setup/moteus-cfg/a_gen.cfg";

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

	Dynamometer(DynamometerSettings& dynset,
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
	inline DynamometerSettings& get_dynset() {return dynset_;}

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

	DynamometerSettings dynset_;

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

cxxopts::Options dyn_opts();
Dynamometer::DynamometerSettings parse_settings(cxxopts::ParseResult& dyn_opts);

#endif