#include <iomanip>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <utility>
#include <string>

#include <cmath>
#include <math.h>

#include <fmt/core.h>
#include <fmt/printf.h>

#include "robot_server.h"
#include "third-party/rapidcsv/rapidcsv.h"
// #include "src/utils.h"

// #include "Utilities/utilities.h"

// TODO: this is copy/paste from "Utilities/utilities.h" -- change to build with that code
/*!
 * Get the LCM URL with desired TTL.
 */
std::string getLcmUrl(int64_t ttl) {
  assert(ttl >= 0 && ttl <= 255);
  return "udpm://239.255.76.67:7667?ttl=" + std::to_string(ttl);
}

namespace chron = std::chrono;

float clamp(float angle) {
	while (angle > PI) 
			angle -= 2 * PI;
	while (angle <= -PI) 
			angle += 2 * PI;
	return angle;
}

// Constructor
RobotServer::RobotServer(RobotServer::RobotServerSettings& rs_set, std::ostream& datastream) : 
	datastream_(datastream),
	rs_set_(rs_set),
	commandLCM_(getLcmUrl(255)),
	responseLCM_(getLcmUrl(255)) {

	// Construct the lowpass filter for the incoming command data
	// lpf_command(rs_set_.lpf_order, rs_set_.lpf_freq, rs_set_.period_s);

	// construct variable number of actuator objects depending on what is in
	// robot_server_config.json
	std::cout << "constructing actuator objects... ";
	actuator_ptrs_.resize(rs_set_.num_actuators);
	for (size_t a_idx = 0; a_idx < rs_set_.num_actuators; a_idx++) {
		// construct Actuator objects in dynamic memory with access by shared_ptr,
		// throw straight into vector
		actuator_ptrs_[a_idx] = std::make_shared<Actuator>(
			rs_set_.moteus_ids[a_idx], rs_set_.moteus_buses[a_idx],
			rs_set_.gear_ratios.size() == 1 ? rs_set_.gear_ratios[0] : rs_set_.gear_ratios[a_idx],
			1
		);
	}
	std::cout << "done; constructed " << num_actuators() <<  " actuators.\n";
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
		requested_command.q_des[a_idx] = std::numeric_limits<float>::quiet_NaN();
		requested_command.qd_des[a_idx] = std::numeric_limits<float>::quiet_NaN();
		requested_command.tau_ff[a_idx] = std::numeric_limits<float>::quiet_NaN();
	}
	std::cout << "\ndone.\n";

	init_LCM();

}

void RobotServer::init_LCM() {
	commandLCM_.subscribe(
		"robot_server_command",
		&RobotServer::handle_robot_server_command, this);
	// launch thread to handle LCM on loop
	serverLCMThread = std::thread(&RobotServer::handle_serverLCM, this);
}

void RobotServer::handle_serverLCM() {
	while (0 == commandLCM_.handle()) {}
}

void RobotServer::handle_robot_server_command(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const robot_server_command_lcmt* msg) {
	(void)rbuf;
	(void)chan;
	std::lock_guard<std::mutex> commandguard(commandmutex); // wrapper for mutex
	requested_command = *msg;
	// lpf_command.iterate_filter TODO MICHAEL FINISH THIS LATER
	last_rx_ = time_prog_s_;
}

// sends commands to "freeze" all actuators
void RobotServer::send_actuator_position_hold() {
	for (size_t a_idx = 0; a_idx < num_actuators(); ++a_idx) {
		float pos = actuator_ptrs_[a_idx]->get_position_rad();
		actuator_ptrs_[a_idx]->make_act_position(pos, 0);
	} 
}

void RobotServer::iterate_fsm() {
	cycles_++;
	sample_sensors();

	curr_state_ = next_state_;

	// immediately respond to fault condition
	if (!safety_check() && curr_state_ != FSMState::kQuitting) next_state_ = 
		FSMState::kRecovery;

	auto time_span = chron::steady_clock::now() - time0_s_;
	time_prog_s_ = double(time_span.count()) * chron::steady_clock::period::num / 
		chron::steady_clock::period::den;
	if (curr_state_ == FSMState::kRunning) time_fcn_s_ += time_prog_s_ - 
		time_prog_old_s_;
	time_prog_old_s_ = time_prog_s_;


	if ((curr_state_ == FSMState::kIdle)) 
		next_state_ = FSMState::kRunning;

	switch (curr_state_) {
		case FSMState::kIdle: {
			break;}
		case FSMState::kStartup: {
			break;}
		case FSMState::kRunning: {
			// ***TODO***: Implement your operation here
			// feel free to expand into multiple states

			// if (num_actuators() != 12) {
			// 	std::cout << "ERROR: num_actuators() = " << num_actuators() <<
			// 		" (only 12 actuator systems currently supported)\n";
			// 	ready_to_quit = true;
			// }
			
			// extract values from requested command and infer relevant control mode
			// stopped mode: all of position, velocity, and torque NAN
			// torque mode: position and velocity NAN
			// velocity mode: position NAN
			// position mode: nothing NAN

			bool rx_timed_out =  (time_prog_s_ - last_rx_ > 1/100); // detects if we haven't gotten a reply recently

			auto& rc = requested_command;
			for (size_t a_idx = 0; a_idx < num_actuators(); ++a_idx) {
				if (std::isnan(rc.q_des[a_idx]) && !rs_set_.ignore_cmds) {
					// either velocity or torque
					if (std::isnan(rc.qd_des[a_idx])) {
						if (std::isnan(rc.tau_ff[a_idx]) || std::fabs(rc.tau_ff[a_idx]) < 0.001) {
							actuator_ptrs_[a_idx]->make_stop();
						}
						else
							// torque feedforward with no pos or vel setpoints
							actuator_ptrs_[a_idx]->make_act_torque(rc.tau_ff[a_idx]);
					}
					else {
						// velocity control with a torque feedforward
						actuator_ptrs_[a_idx]->make_act_velocity(
							rc.qd_des[a_idx], rc.tau_ff[a_idx]);
					}
				}
				else if (!rs_set_.ignore_cmds) {
					// this mode always runs if there was a pos setpoint!!

					// clamps position set points to valid angle range (set in JSON)
					// kill velocity setpoint if attempting to operate outside
					// (does not prevent actuators from being backdriven out of range)
					// (does not kill torque ff... may need to)
					float q_clamped = rc.q_des[a_idx];
					float qd_clamped = rc.qd_des[a_idx];
					// float ul = rs_set_.upper_limits_rad[a_idx];
					// float ll = rs_set_.lower_limits_rad[a_idx];
					// if (q_clamped > ul) {q_clamped = ul; qd_clamped = 0;}
					// if (q_clamped < ll) {q_clamped = ll; qd_clamped = 0;}
					// if missing reply, set actuators to damping mode
					if (actuator_ptrs_[a_idx]->fault() == MoteusController::errc::kMissingReply)
						actuator_ptrs_[a_idx]->make_act_velocity(0,0);
					else 
						actuator_ptrs_[a_idx]->make_act_full_pos(
							q_clamped, qd_clamped, rc.tau_ff[a_idx]);
				}
				// else if (rx_timed_out) { //
				// 	send_actuator_position_hold();
				// }
				else { // if rs_set ignore commands is set
					actuator_ptrs_[a_idx]->make_stop();
				}
			}

			// make_stop_all();
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
			
			// send stops for at least a few cycles
			if(recovery_cycle < recovery_cycle_thresh)
				break;

			// if fault has cleared
			if(safety_check()) next_state_ = recovery_return_state_;

			// if recovery hasn't occurred, quit
			if(!safety_check() && recovery_cycle > recovery_quit) next_state_ = FSMState::kQuitting;
			
			// else, recovery has occurred, and we go back to running
			// else next_state_ = recovery_return_state_;
			break;}
		case FSMState::kQuitting: {
			make_stop_all();
			quit_cycle++;
			// wait at least a few cycles, then raise flag (main will have to quit)
			if(quit_cycle > quit_cycle_thresh) {
				std::cout << "\nsent stop commands... ready to quit" << std::endl;
				ready_to_quit = true;
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
	for (auto a_ptr : actuator_ptrs_) {
		datastream_ << a_ptr->stringify_actuator() << ",";
	}
	datastream_ << stringify_attitude_data() << ",";
	// std::cout << " 3.5 " << std::flush;
	// datastream_ << stringify_sensor_data() << ",";
	datastream_ << (int)curr_state_;
	datastream_ << "\n";
	
	return;
}

void RobotServer::log_headers() {
	std::cout << "creating log headers... ";
	datastream_ << "time [s],";
	for (auto a_ptr : actuator_ptrs_) {
		datastream_ << a_ptr->stringify_actuator_header() << ",";
	}
	datastream_ << stringify_attitude_data_headers() << ",";
	// datastream_ << stringify_sensor_data_headers() << "," << "robot server fsm state";
	datastream_ << "robot server fsm state\n";
	datastream_.flush();
	// datastream_ << "\n";
	std::cout << "   done. " << std::endl;
	
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

	if (safety_check()) std::cout << color_factory.bg_grn() << color_factory.fg_blk() << " **SAFE**";
	else std::cout << "|" << color_factory.bg_red() << color_factory.fg_blk() << "**FAULT**";
	std::cout << color_factory.fg_def() << color_factory.bg_def();
	if (time_prog_s_ - last_rx_ < 1/100) {
		std::cout << " | rx lcm";
	}
	else {
		std::cout << " | NO RX!";
	}
	std::cout << "\r";
	std::cout.flush();
	return;
}

void RobotServer::sample_sensors() {

	sd_.ina1_voltage_V = 0;
	sd_.ina1_current_A = 0;
	sd_.ina1_power_W = 0;

	sd_.ina2_voltage_V = 0;
	sd_.ina2_current_A = 0;
	sd_.ina2_power_W = 0;

}

std::string RobotServer::stringify_sensor_data() {
	return fmt::sprintf("%f,%f,%f,%f,%f,%f",
		sd_.ina1_voltage_V, sd_.ina1_current_A, sd_.ina1_power_W,
		sd_.ina2_voltage_V, sd_.ina2_current_A, sd_.ina2_power_W);
}

std::string RobotServer::stringify_sensor_data_headers() {
	// std::ostringstream result;

	return "ina1 voltage [V],ina1 current [A],ina1 power [W],ina2 voltage [V],ina2 current [A],ina2 power [W]";
	// return result.str();
}

std::string RobotServer::stringify_attitude_data() {
	auto& pa = pi3hat_attitude_;
	return fmt::sprintf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		pa.attitude.w, pa.attitude.x, pa.attitude.y, pa.attitude.z,
		pa.accel_mps2.x, pa.accel_mps2.y, pa.accel_mps2.z,
		pa.rate_dps.x*(M_PI/180), pa.rate_dps.y*(M_PI/180), pa.rate_dps.z*(M_PI/180),
		pa.bias_dps.x*(M_PI/180), pa.bias_dps.y*(M_PI/180), pa.bias_dps.z*(M_PI/180));
}

std::string RobotServer::stringify_attitude_data_headers() {
	// std::ostringstream result;

	return "IMU quat w [],IMU quat x [],IMU quat y [],IMU quat z [],IMU accel x [m/s/s],IMU accel y [m/s/s],IMU accel z [m/s/s],IMU omega x [rad/s],IMU omega y [rad/s],IMU omega z [rad/s],IMU bias x [rad/s],IMU bias y [rad/s],IMU bias z [rad/s]";
	// return result.str();
}


bool RobotServer::safety_check() {
	//
	bool safe = true;

	if (actuator_fault_check()) return false;
	
	// Put more conditions to check here

	return safe;
}

bool RobotServer::actuator_fault_check() {
	bool fault = false;
	for (auto a_ptr : actuator_ptrs_) {
		fault |= bool(a_ptr->fault()) && (a_ptr->fault() != MoteusController::errc::kMissingReply);
	}
	return fault;
}

void RobotServer::publish_LCM_response() {
	// auto& server_response = server_response;
	for (size_t a_idx = 0; a_idx < num_actuators(); ++a_idx) {
		server_response.q[a_idx] = actuator_ptrs_[a_idx]->get_position_rad();
		if (std::isnan(server_response.q[a_idx])){
			std::cout << "\npos nan a" << a_idx << " LCM; get pos = " << actuator_ptrs_[a_idx]->get_position_rad();
			// std::cout.flush();
		}
		server_response.qd[a_idx] = actuator_ptrs_[a_idx]->get_velocity_rad_s();
		if (std::isnan(server_response.qd[a_idx])){
			std::cout << "\nvel nan a" << a_idx << " LCM; get vel = " << actuator_ptrs_[a_idx]->get_velocity_rad_s();
			// std::cout.flush();
		}
		server_response.tau_est[a_idx] = actuator_ptrs_[a_idx]->get_torque_Nm();
	}
	server_response.fsm_state = uint8_t(curr_state_);
	server_response.accelerometer[0] = pi3hat_attitude_.accel_mps2.x;
	server_response.accelerometer[1] = pi3hat_attitude_.accel_mps2.y;
	server_response.accelerometer[2] = pi3hat_attitude_.accel_mps2.z;

	server_response.gyro[0] = pi3hat_attitude_.rate_dps.x * (M_PI/180.0);
	server_response.gyro[1] = pi3hat_attitude_.rate_dps.y * (M_PI/180.0);
	server_response.gyro[2] = pi3hat_attitude_.rate_dps.z * (M_PI/180.0);

	server_response.quat[0] = pi3hat_attitude_.attitude.w;
	server_response.quat[1] = pi3hat_attitude_.attitude.x;
	server_response.quat[2] = pi3hat_attitude_.attitude.y;
	server_response.quat[3] = pi3hat_attitude_.attitude.z;

	robot_server_response_lcmt* data = &server_response;
	responseLCM_.publish("robot_server_response", data);
	// commandLCM_.publish("robot_server_response", data);
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
				"/home/pi/Quadruped-Software/robot-server/robot_server_config.json"))
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
		("ignore-cmds", "ignore incoming commands and set actuators to idle")
		("h,help", "Print usage")
	;

	return options;
}

RobotServer::RobotServerSettings::RobotServerSettings(cxxopts::ParseResult& rs_opts_in) {
	rs_opts = rs_opts_in;

	std::cout << "parsing RobotServer config from " << rs_opts["rs-cfg-path"].as<std::string>() << "..." << std::endl;
	std::ifstream rs_cfg_if(rs_opts["rs-cfg-path"].as<std::string>());
	nlohmann::json rs_cfg_j = nlohmann::json::parse(
		rs_cfg_if,
		nullptr, true, true); // parse with `//`-led comments

	float frq_override = rs_opts["frequency-override"].as<float>();
	period_s = frq_override >= 0 ? 1.0/frq_override : 1.0/double(rs_cfg_j["loop_frequency"]);
	duration_s = rs_opts["duration"].as<float>();

	main_cpu = rs_opts["main-cpu"].as<uint8_t>();
	can_cpu = rs_opts["can-cpu"].as<uint8_t>();
	
	skip_count_max = rs_cfg_j["skip_count_max"];

	// test_delay = rs_opts["test-delay"].as<float>();

	skip_cal = rs_opts["skip-cal"].as<bool>();
	skip_zero = rs_opts["skip-zero"].as<bool>();
	skip_cfg_load = rs_opts["skip-cfg-load"].as<bool>();

	ignore_cmds = rs_opts["ignore-cmds"].as<bool>();

	moteus_ids = rs_cfg_j["moteus_ids"].get<std::vector<uint8_t>>();
	num_actuators = moteus_ids.size();

	moteus_buses = rs_cfg_j["moteus_buses"].get<std::vector<uint8_t>>();

	moteus_cfg_path_prefix = rs_cfg_j["moteus_cfg_path_prefix"];
	moteus_cfg_filenames = rs_cfg_j["moteus_cfg_filenames"].get<std::vector<std::string>>();
	moteus_cal_path_prefix = rs_cfg_j["moteus_cal_path_prefix"];
	moteus_cal_filenames = rs_cfg_j["moteus_cal_filenames"].get<std::vector<std::string>>();

	upper_limits_rad = rs_cfg_j["upper_limits_rad"].get<std::vector<float>>();
	lower_limits_rad = rs_cfg_j["lower_limits_rad"].get<std::vector<float>>();

	lpf_order = rs_cfg_j["lpf_order"].get<float>();
	lpf_freq = rs_cfg_j["lpf_freq"].get<float>();

	mounting_roll_deg = rs_cfg_j["mounting_roll_deg"].get<float>();
	mounting_pitch_deg = rs_cfg_j["mounting_pitch_deg"].get<float>();
	mounting_yaw_deg = rs_cfg_j["mounting_yaw_deg"].get<float>();

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
