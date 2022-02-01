#include <iomanip>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <utility>

#include <cmath>
#include <math.h>

#include "robot_server.h"
// #include "color.h"
#include "rapidcsv/rapidcsv.h"

namespace chron = std::chrono;

float clamp(float angle) {
	while (angle > PI) 
			angle -= 2 * PI;
	while (angle <= -PI) 
			angle += 2 * PI;
	return angle;
}

RobotServer::RobotServer(RobotServer::RobotServerSettings& rs_set, std::ostream& datastream) : 
	datastream_(datastream),
	rs_set_(rs_set) {

	std::cout << "setting up sensors... ";
	ads_.begin(0x48);
	ads_.setGain(adsGain_t::GAIN_ONE);
	ads_.setDataRate(RATE_ADS1015_3300SPS);
	ina1_.begin(0x40);
	ina1_.prime_i2c();
	ina1_.setCurrentConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
	ina1_.setVoltageConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
	ina2_.begin(0x41);
	ina2_.prime_i2c();
	ina2_.setCurrentConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
	ina2_.setVoltageConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
	std::cout << "done.\n";

	std::cout << "constructing actuator objects... ";
	actuator_ptrs_.resize(rs_set_.num_actuators);
	for (size_t a_idx = 0; a_idx < rs_set_.num_actuators; a_idx++) {
		// construct Actuator objects in dynamic memory with access by shared_ptr,
		// throw straight into vector
		actuator_ptrs_[a_idx] = std::make_shared<Actuator>(
			rs_set_.moteus_ids[a_idx], rs_set_.moteus_buses[a_idx],
			rs_set_.gear_ratios.size() == 1 ? rs_set_.gear_ratios[0] : rs_set_.gear_ratios[a_idx],
			1
		)
	}
	std::cout << "done.\n";
	std::cout << "restoring configs and calibrations...\n";
	for (size_t a_idx = 0; a_idx < rs_set_.num_actuators; a_idx++) {
		if (!rs_set_.skip_cal)
			actuator_ptrs_[a_idx]->restore_cal(rs_set_.moteus_cal_path_prefix + rs_set_.moteus_cal_filenames[a_idx]);
		if (!rs_set_.skip_cfg_load) {
			std::string fname_local =
				rs_set_.moteus_cfg_filenames.size() == 1
					? rs_set_.moteus_cfg_filenames[0] : rs_set_.moteus_cfg_filenames[a_idx];
			actuator_ptrs_[a_idx]->restore_cfg(rs_set_.moteus_cfg_path_prefix + fname_local);
		}
		if (!rs_set_.skip_zero)
			actuator_ptrs_[a_idx]->zero_offset();
	}
	std::cout << "\ndone.\n";

}

void RobotServer::iterate_fsm() {
	cycles_++;
	sample_sensors();

	// immediately respond to fault condition
	if (robotserver_fault()) next_state_ = 
		FSMState::kRecovery;

	curr_state_ = next_state_;

	auto time_span = chron::steady_clock::now() - time0_s_;
	time_prog_s_ = double(time_span.count()) * chron::steady_clock::period::num / 
		chron::steady_clock::period::den;
	if (curr_state_ == FSMState::kRunning) time_fcn_s_ += time_prog_s_ - 
		time_prog_old_s_;
	time_prog_old_s_ = time_prog_s_;


	if ((time_prog_s_ > rs_set_.test_delay) && (curr_state_ == FSMState::kIdle)) 
		next_state_ = FSMState::kRunning;

	switch (curr_state_) {
		case FSMState::kIdle: {
			break;}
		case FSMState::kStartup: {
			break;}
		case FSMState::kRunning: {
			// ***TODO***: Implement your operation here
			// feel free to expand into multiple states

			make_stop_all();
			break;}
		case FSMState::kRecovery: {
			// if coming from non-recovery, store the state so we can go back
			if(prev_state_ != FSMState::kRecovery) {
				recovery_return_state_ = prev_state_;
				recovery_cycle = 0;
				num_recoveries++;
			}
			make_stop_all();
			recovery_cycle++;
			
			// wait at least a few cycles
			if(recovery_cycle < recovery_cycle_thresh)
				break;

			// if fault has cleared
			if(!robotserver_fault()) next_state_ = recovery_return_state_;

			// if recovery hasn't occurred, quit
			if(robotserver_fault()) next_state_ = FSMState::kQuitting;
			
			// else, recovery has occurred, and we go back to running
			else next_state_ = recovery_return_state_;
			break;}
		case FSMState::kQuitting: {
			make_stop_all();
			quit_cycle++;
			// wait at least a few cycles, then raise flag (main will have to quit)
			if(quit_cycle > quit_cycle_thresh) {
				ready_to_quit = true;
				// std::cout << "\n\nerror codes on exit: a1 = " << int(a1_.fault()) << "; a2 = " << int(a2_.fault()) << std::endl;
			}
			break;}
		
		default: {
			next_state_ = curr_state_;
			break;}
	}
	prev_state_ = curr_state_;
	log_data();
	return;
}

void RobotServer::log_data() {
	datastream_ << std::setw(10) << std::setprecision(4) << std::fixed
		<< time_prog_s_ << ",";
	datastream_ << a1_.stringify_actuator() << ",";
	// std::cout << " 3.2 " << std::flush;
	datastream_ << a1_.stringify_moteus_reply() << ",";
	// std::cout << " 3.3 " << std::flush;
	datastream_ << a2_.stringify_actuator() << ",";
	// std::cout << " 3.4 " << std::flush;
	datastream_ << a2_.stringify_moteus_reply() << ",";
	// std::cout << " 3.5 " << std::flush;
	datastream_ << stringify_sensor_data() << ",";
	datastream_ << (int)a_driving_ptr->get_id() << ",";
	datastream_ << (int)curr_state_;
	
	datastream_ << "\n";
	
	return;
}

void RobotServer::log_headers() {
	datastream_ << "# driving actuator id: " << a_driving_ptr->get_id() << "\n";
	datastream_ << "time [s],";
	datastream_ << a1_.stringify_actuator_header() << ","
	<< a1_.stringify_moteus_reply_header() << ","
	<< a2_.stringify_actuator_header() << ","
	<< a2_.stringify_moteus_reply_header() << ","
	<< stringify_sensor_data_headers() << ","
	<< "driving id" << ","
	<< "robot server fsm state";
	
	datastream_ << "\n";
	
	return;
}

void RobotServer::print_status_update() {
	Color::Modifier bg_temp_m(Color::BG_DEFAULT);
	Color::Modifier bg_temp_h(Color::BG_DEFAULT);
	Color::Modifier bg_safe(Color::BG_DEFAULT);
	
	Color::Modifier bg_temp_latch(Color::BG_DEFAULT);

	Color::Modifier color_factory(Color::Code::FG_DEFAULT);

	std::cout << color_factory.fg_blk() << color_factory.bg_wht() << "t_p:"
		<< std::setw(7) << std::setprecision(1) << std::fixed << time_prog_s_
		<< "|t_f:"
		<< std::setw(7) << std::setprecision(1) << std::fixed << time_fcn_s_
		<< color_factory.fg_def() << color_factory.bg_def() << "|" ;
	std::cout << "FSM:"
		<< std::setw(2) << std::setprecision(2) << std::fixed << (int)curr_state_ << "|";
	
	std::cout << "NR:"
		<< std::setw(2) << std::setprecision(2) << std::fixed << (int)num_recoveries << "|";

	if (!(bool)robotserver_fault()) std::cout << color_factory.bg_grn() << color_factory.fg_blk() << " **SAFE**";
	else std::cout << "|" << color_factory.bg_red() << color_factory.fg_blk() << "**FAULT**";
	std::cout << color_factory.fg_def() << color_factory.bg_def() << "\r";
	std::cout.flush();
	return;
}

void RobotServer::sample_sensors() {
	// TODO: Add temp read and calc
	sd_.torque_Nm = 0;
	float t1 = 0;
	float t2 = 0;
	float alpha = 0.1;
	// sd_.temp1_C = 0;
	// sd_.temp2_C = 0;

	sd_.ina1_voltage_V = 0;
	sd_.ina1_current_A = 0;
	sd_.ina1_power_W = 0;

	sd_.ina2_voltage_V = 0;
	sd_.ina2_current_A = 0;
	sd_.ina2_power_W = 0;

}

std::string RobotServer::stringify_sensor_data() {
	std::ostringstream result;

	sprintf(cstr_buffer, ",%f,%f,", sd_.temp1_C, sd_.temp2_C);
	result << cstr_buffer;
	sprintf(cstr_buffer, "%f,%f,%f,", sd_.ina1_voltage_V, sd_.ina1_current_A, sd_.ina1_power_W);
	result << cstr_buffer;
	sprintf(cstr_buffer, "%f,%f,%f", sd_.ina2_voltage_V, sd_.ina2_current_A, sd_.ina2_power_W);
	result << cstr_buffer;
	return result.str();
}

std::string RobotServer::stringify_sensor_data_headers() {
	std::ostringstream result;

	result << ",motor temp [C],housing temp [C],ina1 voltage [V],ina1 current [A],ina1 power [W],ina2 voltage [V],ina2 current [A],ina2 power [W]";
	return result.str();
}


bool RobotServer::safety_check() {
	//
	bool safe = true;

	if (robotserver_fault()) return false;
	
	// Put conditions to check here

	return safe;
}

bool RobotServer::actuator_fault_check() {
	bool fault = false;
	for (auto a_ptr : actuator_ptrs_) {
		fault |= a_ptr->fault();
	}
	return fault;
}

void RobotServer::make_stop_all() {
	for (auto a_ptr : actuator_ptrs_) {
		a_ptr->make_stop();
	}
}

std::map<int, int> RobotServer::create_servo_bus_map() {
	std::map<int, int> servomap;
	for (auto a_ptr : actuator_ptrs_) {
		servomap.insert({a_ptr->get_id(), a_ptr->get_bus()});
	}
	return servomap;
}

cxxopts::Options rs_opts() {
	cxxopts::Options options(
		"leg", "Run 2D leg");

	options.add_options()
		("rs-cfg-path", "path to robot server config JSON", 
			cxxopts::value<std::string>()->default_value(
				"/home/pi/quadruped-software/robot-server/robot_server_config.json"))
		("c,comment", "enter comment string to be included in output csv.", 
			cxxopts::value<std::string>())
		("p,path", "path to output csv.",
			cxxopts::value<std::string>()->default_value(
				"/home/pi/embir-modular-leg/leg-data/"))
		("main-cpu", "main CPU", cxxopts::value<uint8_t>()->default_value("1"))
		("can-cpu", "CAN CPU", cxxopts::value<uint8_t>()->default_value("2"))
		("duration", "test duration in seconds", cxxopts::value<float>())
		("frequency-override", "test sampling and command frequency in Hz, overriding value in robot_server_config.json", 
			cxxopts::value<float>())
		("skip-cal", "skip recalibration")
		("skip-zero", "skip setting actuator zero")
		("skip-cfg-load", "skip loading actuator config")
		("h,help", "Print usage")
	;

	return options;
}

RobotServer::RobotServerSettings::RobotServerSettings(cxxopts::ParseResult& rs_opts_in) {
	rs_opts = rs_opts_in;

	std::ifstream rs_cfg_if(rs_opts["rs-cfg-path"].as<std::string>());
	nlohmann::json rs_cfg_j = nlohmann::json::parse(
		rs_cfg_if,
		nullptr, true, true); // parse with `//`-led comments

	float frq_override = rs_opts["frequency-override"].as<float>();
	period_s = frq_override >= 0 ? 1.0/frq_override : 1.0/rs_cfg_j["loop_frequency"];
	duration_s = rs_opts["duration"].as<float>();

	main_cpu = rs_opts["main-cpu"].as<uint8_t>();
	can_cpu = rs_opts["can-cpu"].as<uint8_t>();
	
	skip_count_max = rs_cfg_j["skip_count_max"];

	// test_delay = rs_opts["test-delay"].as<float>();

	skip_cal = rs_opts["skip-cal"].as<bool>();
	skip_zero = rs_opts["skip-zero"].as<bool>();
	skip_cfg_load = rs_opts["skip-cfg-load"].as<bool>();

	moteus_ids = rs_cfg_j["moteus_ids"].get<std::vector<uint8_t>>();
	num_actuators = moteus_ids.size();

	moteus_buses = rs_cfg_j["moteus_buses"].get<std::vector<uint8_t>>();

	moteus_cfg_path_prefix = rs_cfg_j["moteus_cfg_path_prefix"];
	moteus_cfg_filenames = rs_cfg_j["moteus_cfg_filenames"].get<std::vector<std::string>>();
	moteus_cal_path_prefix = rs_cfg_j["moteus_cal_path_prefix"];
	moteus_cal_filenames = rs_cfg_j["moteus_cal_filenames"].get<std::vector<std::string>>();

	gear_ratios = rs_cfg_j["gear_ratios"].get<std::vector<float>>();
	if (
			moteus_buses.size() != num_actuators ||
			(moteus_cfg_filenames.size() != 1 && moteus_cfg_filenames.size() != num_actuators) ||
			moteus_cal_filenames.size() != num_actuators ||
			(gear_ratios.size() != 1 && gear_ratios.size() != num_actuators)) {
		std::cerr << "size mismatch in config parameters!!" << std::endl;
		load_success = false;
	}
	else {
		load_success = true;
	}

}
