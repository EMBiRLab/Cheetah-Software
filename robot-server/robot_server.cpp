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
	a1_(rs_set.a1_id, rs_set.a1_bus, rs_set.gear_a1, 1.0),
	a2_(rs_set.a2_id, rs_set.a2_bus, rs_set.gear_a2, 1.0),
	datastream_(datastream),
	rs_set_(rs_set),
	commands_(),
	lpf_a1_(rs_set.lpf_order, rs_set.lpf_cutoff_freq_Hz, rs_set.period_s),
	lpf_a2_(rs_set.lpf_order, rs_set.lpf_cutoff_freq_Hz, rs_set.period_s),
	lpf_grp_(rs_set.lpf_order, rs_set.lpf_cutoff_freq_Hz, rs_set.period_s) {

	if (rs_set_.a1_id == rs_set_.a2_id) {
		throw std::runtime_error("The servos must have unique IDs! Exiting...");
		ready_to_quit = true;
	}
	a_driving_ptr = &a1_;
	a_loading_ptr = &a2_;
	a1_.set_pos_lower_bound(std::numeric_limits<float>::quiet_NaN()); a1_.set_pos_upper_bound(std::numeric_limits<float>::quiet_NaN());
	a2_.set_pos_lower_bound(std::numeric_limits<float>::quiet_NaN()); a2_.set_pos_upper_bound(std::numeric_limits<float>::quiet_NaN());

	if (rs_set_.rs_opts["swap-actuators"].as<bool>()) swap_actuators();
	test_mode_ = rs_set_.test_mode;
	if (rs_set_.test_mode == RobotServer::TestMode::kDurability) {
		if (rs_set_.playback_file == "") {
			throw std::runtime_error("No replay file specified! Exiting...");
			ready_to_quit = true;
		}
		else {
			// Load mini cheetah test file.
			load_playback_data(rs_set_.playback_file);
		}
	}

	if (!rs_set_.skip_cal) {
		a1_.restore_cal("/home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_1_dyn.log");
		a2_.restore_cal("/home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_2_dyn.log");
	}
	if (!rs_set_.skip_zero) {
		a1_.zero_offset();
		a2_.zero_offset();
	}
	if (!rs_set_.skip_cfg_load) {
		a1_.restore_cfg("/home/pi/embir-modular-leg/moteus-setup/moteus-cfg/dyn_a1.cfg");
		a2_.restore_cfg("/home/pi/embir-modular-leg/moteus-setup/moteus-cfg/dyn_a2.cfg");
	}

	std::cout << "loading configs... ";

	std::ifstream grp_if("configs/grp.json");
	grp_if >> grp_j;
	grp_s = GRPSettings(grp_j);

	std::ifstream durability_if("configs/durability.json");
	durability_if >> durability_j;
	durability_s = DurabilitySettings(durability_j);

	std::ifstream efficiency_if("configs/efficiency.json");
	efficiency_if >> efficiency_j;
	efficiency_s = EfficiencySettings(efficiency_j);

	std::ifstream step_if("configs/step.json");
	step_if >> step_j;
	step_s = StepSettings(step_j);

	std::ifstream torque_step_if("configs/torque_step.json");
	torque_step_if >> torque_step_j;
	torque_step_s = TorqueStepSettings(torque_step_j);

	std::ifstream safety_if("configs/safety_limits.json");
	safety_if >> safety_j;
	safety_s = SafetySettings(safety_j);
	if(rs_set_.tqsen == RobotServer::TorqueSensor::kTRS605_5) safety_s.ts_max_torque_Nm_ = safety_s.trs605_5_max_torque_Nm_;
	if(rs_set_.tqsen == RobotServer::TorqueSensor::kTRD605_18) safety_s.ts_max_torque_Nm_ = safety_s.trd605_18_max_torque_Nm_;

	rs_set_.grp_sampling_period_us = static_cast<int64_t>(
		(1e6)/float(grp_j["random_sampling_frequency_Hz"]));

	rs_set_.status_period_us = static_cast<int64_t>(
		(1e6)/float(10.0)); // 10Hz

	std::cout << "done.\n";

	// for (float trq : durability_s.torques_Nm) {
	// 	for (float vel : durability_s.velocities_rad_s) {
	// 		// std::cout << trq << ", " << vel << std::endl;
	// 		sweep_trq.push_back(trq);
	// 		sweep_trq.push_back(trq);
	// 		sweep_trq.push_back(0);
	// 		sweep_trq.push_back(-trq);
	// 		sweep_trq.push_back(-trq);
	// 		sweep_trq.push_back(0);

	// 		sweep_vel.push_back(vel);
	// 		sweep_vel.push_back(-vel);
	// 		sweep_vel.push_back(0);
	// 		sweep_vel.push_back(vel);
	// 		sweep_vel.push_back(-vel);
	// 		sweep_vel.push_back(0);
	// 	}
	// }

	std::uniform_real_distribution<> dist(-grp_s.amplitude, grp_s.amplitude);
	realdist = dist;

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

	if (rs_set_.test_mode == RobotServer::TestMode::kEfficiency) {
		efficiency_timer.start_timer(efficiency_s.idle_duration_s);
	}
	if (rs_set_.test_mode == RobotServer::TestMode::kDurability) {
		durability_timer.start_timer(0.5);
		durability_cycle_s = 
			2*efficiency_s.efficiency_sweep_duration_s
			+ 2*durability_s.damping_total_duration_s
			+ 4*durability_s.interphase_idle_duration_s
			+ durability_s.follow_duration_s;
		std::cout << "durability cycle time = " << durability_cycle_s <<" s" << std::endl;
		if (rs_set_.durability_resume > 0.1) {
			initial_follow_delay = true;
			float time_delay = 0;
			time_delay =
				int(rs_set_.durability_resume / durability_cycle_s)
				*(durability_s.follow_duration_s)
				+ fmod(rs_set_.durability_resume, durability_cycle_s);
			playback_idx = time_delay / 0.002;
			if (playback_idx >= playback_vel.size()) { // if we got thru a complete playback
				if (playback_idx < playback_vel.size() + 30.0/0.002) { // if we resume from the 30s pause
					time_delay = 0;
				}
				else { // else if we had got past the pause
					time_delay = fmod(time_delay, playback_vel.size()*0.002 + 30);
				}
			}
			playback_idx = time_delay / 0.002;
			std::cout << "resuming from previous test at " << rs_set_.durability_resume << " s\n"
				<< "time delay into playback data = " << time_delay << " s\n"
				<< "playback_idx = " << playback_idx << std::endl;
		}
	}
}

void RobotServer::iterate_fsm() {
	cycles_++;
	sample_sensors();
	// std::cout << "in iterate_fsm(), cycle = " << cycles_ << "\n" << std::flush;
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

	// state logic for simple test:
	// if (((int)time_prog_s_)%2) next_state_ = FSMState::kIdle;
	// else next_state_ = FSMState::kRunning;

	if ((time_prog_s_ > rs_set_.test_delay) && (curr_state_ == FSMState::kIdle)) 
		next_state_ = FSMState::kRunning;
	// std::cout << "1 " << std::flush;

	switch (curr_state_) {
		case FSMState::kIdle: {
			break;}
		case FSMState::kStartup: {
			break;}
		case FSMState::kRunning: {
			// ***TODO***: Implement your operation here
			// feel free to expand into multiple states

			break;}
		case FSMState::kRecovery: {
			// if coming from non-recovery, store the state so we can go back
			if(prev_state_ != FSMState::kRecovery) {
				recovery_return_state_ = prev_state_;
				recovery_cycle = 0;
				num_recoveries++;
			}
			// std::cout << "recovery! " << int(recovery_cycle) << std::endl;
			a1_.make_stop();
			a2_.make_stop();
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
			a1_.make_stop();
			a2_.make_stop();
			quit_cycle++;
			// wait at least a few cycles, then raise flag (main will have to quit)
			if(quit_cycle > quit_cycle_thresh) {
				ready_to_quit = true;
				std::cout << "\n\nerror codes on exit: a1 = " << int(a1_.fault()) << "; a2 = " << int(a2_.fault()) << std::endl;
			}
			break;}
		
		default: {
			next_state_ = curr_state_;
			break;}
	}
	// std::cout << "3 " << std::flush;
	prev_state_ = curr_state_;
	log_data();
	// std::cout << "4 " << std::flush;
	return;
}

void RobotServer::log_data() {
	datastream_ << std::setw(10) << std::setprecision(4) << std::fixed
		<< time_prog_s_ << ",";
	// std::cout << " 3.1 " << std::flush;
	if (rs_set_.test_mode == RobotServer::TestMode::kGRP) {
		datastream_ << std::setw(10) << std::setprecision(5) << std::fixed
			<< a_driving_ptr->get_velocity_rad_s() << ",";
		datastream_ << std::setw(10) << std::setprecision(5) << std::fixed
			<< get_ts_torque() << ",";
		datastream_ << std::setw(10) << std::setprecision(5) << std::fixed
			<< a_driving_ptr->get_torque_cmd_Nm() << ",";
		datastream_ << std::setw(10) << std::setprecision(5) << std::fixed
			<< a_driving_ptr->get_torque_Nm() << ",";
		datastream_ << std::setw(10) << std::fixed
			<< (int)(a1_.fault()) << ",";
		datastream_ << std::setw(10) << std::fixed
			<< (int)(a2_.fault()) << ",";
		datastream_ << std::setw(10) << std::setprecision(5) << std::fixed
			<< a_driving_ptr->get_velocity_cmd_rad_s() << ",";
		datastream_ << std::setw(10) << std::setprecision(5) << std::fixed
			<< a_driving_ptr->get_position_cmd_rad() << ",";
		datastream_ << std::setw(10) << std::setprecision(5) << std::fixed
			<< a_driving_ptr->get_position_rad();
	}
	else {
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
		if(rs_set_.test_mode == RobotServer::TestMode::kDurability) {
			datastream_ << "," << (int)dts;
		}
		if(rs_set_.test_mode == RobotServer::TestMode::kEfficiency
				|| rs_set_.test_mode == RobotServer::TestMode::kDurability) {
			datastream_ << "," << (int)ets;
		}
	}
			
	datastream_ << "\n";
	
	return;
}

void RobotServer::log_headers() {
	datastream_ << "# driving actuator id: " << a_driving_ptr->get_id() << "\n";
	datastream_ << "time [s],";
	if(rs_set_.test_mode == RobotServer::TestMode::kGRP) {
		datastream_ << "a1 velocity [rad/s],trs605-5 torque [Nm],a1 torque cmd [Nm],a1 torque est [Nm],c1 fault,c2 fault,a1 velocity cmd [rad/s],a1 position cmd [rad],a1 position [rad]\n";
	}
	else {
		datastream_ << a1_.stringify_actuator_header() << ","
		<< a1_.stringify_moteus_reply_header() << ","
		<< a2_.stringify_actuator_header() << ","
		<< a2_.stringify_moteus_reply_header() << ","
		<< stringify_sensor_data_headers() << ","
		<< "driving id" << ","
		<< "robot server fsm state";
	}
	
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
	if (rs_set_.test_mode == RobotServer::TestMode::kDurability) {
		std::cout << "dts:"
			<< std::setw(2) << std::setprecision(2) << std::fixed << (int)dts << "|";
	}
	if (rs_set_.test_mode == RobotServer::TestMode::kEfficiency) {
		std::cout << "ets:"
			<< std::setw(2) << std::setprecision(2) << std::fixed << (int)ets << "|";
		std::cout << "v=" << std::setw(5) << std::setprecision(2) << std::fixed << cond_vel;
		std::cout << ",t=" << std::setw(5) << std::setprecision(2) << std::fixed << cond_torque << "|";
		std::cout << "nt=" << std::setw(3) << std::setprecision(1) << std::fixed
			<< int(num_tested) << "|";
	}
	if (rs_set_.test_mode == RobotServer::TestMode::kEfficiency
							|| rs_set_.test_mode == RobotServer::TestMode::kDurability) {
		std::cout << "dp=" << std::setw(4) << std::setprecision(2) << std::fixed
			<< a1_.get_position_rad() + a2_.get_position_rad() << "|";
		std::cout << "V=" << std::setw(4) << std::setprecision(2) << std::fixed << sd_.ina1_voltage_V << "|";
	}
	if (rs_set_.test_mode == RobotServer::TestMode::kStep) {
		if (step_s.step_temp_latch) bg_temp_latch = color_factory.bg_red();
		else bg_temp_latch = color_factory.bg_grn();
		std::cout << "temp latch:"
			<< std::setw(2) << std::setprecision(2) << std::fixed 
				<< bg_temp_latch << (int)step_s.step_temp_latch << color_factory.bg_def() << "|";
	}
	std::cout << "FSM:"
		<< std::setw(2) << std::setprecision(2) << std::fixed << (int)curr_state_ << "|";
	
	std::cout << "NR:"
		<< std::setw(2) << std::setprecision(2) << std::fixed << (int)num_recoveries << "|";

	if(rs_set_.test_mode == RobotServer::TestMode::kTorqueStep) {
		std::cout << "IX:"
			<< std::setw(2) << std::setprecision(2) << std::fixed << (int)torque_step_s.torque_idx << "|";
		std::cout << "IP:"
			<< std::setw(2) << std::setprecision(2) << std::fixed << (int)torque_step_s.is_paused << "|";
	}


	float temp_m = sd_.temp1_C;
	float temp_h = sd_.temp2_C;
	if (temp_m < 40) bg_temp_m = color_factory.bg_grn();
	else if (temp_m < 60) bg_temp_m = color_factory.bg_yel();
	else bg_temp_m = color_factory.bg_red();

	if (temp_h < 40) bg_temp_h = color_factory.bg_grn();
	else if (temp_h < 60) bg_temp_h = color_factory.bg_yel();
	else bg_temp_h = color_factory.bg_red();
	std::cout << "temp:" << bg_temp_m << color_factory.fg_blk() 
		<< std::setw(5) << std::setprecision(1) << std::fixed << temp_m << color_factory.fg_def() << color_factory.bg_def()
		// << "|temp_h:" << bg_temp_h << color_factory.fg_blk 
		// << std::setw(5) << std::setprecision(1) << std::fixed << temp_h << color_factory.fg_def << color_factory.bg_def
		<< "|ID: "
		<< color_factory.bg_blu() << color_factory.fg_def() << (int)a_driving_ptr->get_id() << color_factory.bg_def();

	if (!(bool)robotserver_fault()) std::cout << color_factory.bg_grn() << color_factory.fg_blk() << " **SAFE**";
	else std::cout << "|" << color_factory.bg_red() << color_factory.fg_blk() << "**FAULT**";
	std::cout << color_factory.fg_def() << color_factory.bg_def() << "\r";
	std::cout.flush();
	return;
}

cxxopts::Options rs_opts() {
	cxxopts::Options options(
		"leg", "Run 2D leg");

	options.add_options()
		("c,comment", "enter comment string to be included in output csv.", 
			cxxopts::value<std::string>())
		("p,path", "path to output csv.",
			cxxopts::value<std::string>()->default_value(
				"/home/pi/embir-modular-leg/leg-data/"))
		("main-cpu", "main CPU", cxxopts::value<uint8_t>()->default_value("1"))
		("can-cpu", "CAN CPU", cxxopts::value<uint8_t>()->default_value("2"))
		("duration", "test duration in seconds", cxxopts::value<float>())
		("frequency", "test sampling and command frequency in Hz", 
			cxxopts::value<float>()->default_value("250"))
		("skip-cal", "skip recalibration")
		("skip-zero", "skip setting actuator zero")
		("skip-cfg-load", "skip loading actuator config")
		("h,help", "Print usage")
	;

	return options;
}

RobotServer::RobotServerSettings::RobotServerSettings(cxxopts::ParseResult& rs_opts_in) {
	rs_opts = rs_opts_in;
	period_s = 1.0/rs_opts["frequency"].as<float>();
	duration_s = rs_opts["duration"].as<float>();

	main_cpu = rs_opts["main-cpu"].as<uint8_t>();
	can_cpu = rs_opts["can-cpu"].as<uint8_t>();
	
	test_delay = rs_opts["test-delay"].as<float>();

	skip_cal = rs_opts["skip-cal"].as<bool>();
	skip_zero = rs_opts["skip-zero"].as<bool>();
	skip_cfg_load = rs_opts["skip-cfg-load"].as<bool>();
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
	
	auto testmode = rs_set_.test_mode;

	// conditions for temp sensors
	if (testmode != TestMode::kGRP && testmode != TestMode::kTorqueStep) {
		// TODO: put these into a config json
		float R0 = 100000;
		float T0 = 25;
		float Rf = 100000;
		float V0 = 3.3;

		ads_.prime_i2c();
		uint16_t adc = ads_.readADC_SingleEnded(2);
		float Vt = ads_.computeVolts(adc);
		if (std::isnan(Vt)) std::cout << "\n nan temp 0" << std::endl;
		// thermistor resistance
		float Rt = Rf*Vt / (V0 - Vt);
		if (std::isnan(Rt)) std::cout << "\n nan temp 1" << std::endl;
		
		t1 = (1.0/298.15) + (1.0/3950.0)*log(Rt/R0);
		if (std::isnan(t1)) t1 = sd_.temp1_C;
		else {
			t1 = 1.0/t1 - 273.15;
			if (std::isnan(t1)) std::cout << "\n nan temp 3" << std::endl;
		}
		// exp decay lpf
		t1 = alpha*t1 + (1-alpha)*sd_.temp1_C;

		adc = ads_.readADC_SingleEnded(3);
		Vt = ads_.computeVolts(adc);
		Rt = Rf*Vt / (V0 - Vt);
		t2 = (1.0/298.15) + (1.0/3950.0)*log(Rt/R0);
		if (std::isnan(t2)) t2 = sd_.temp2_C;
		else t2 = 1.0/t2 - 273.15;

		t2 = alpha*t2 + (1-alpha)*sd_.temp2_C;
	}
	// conditions for power meter
	if (testmode != TestMode::kGRP && testmode != TestMode::kTorqueStep) {
		ina1_.prime_i2c();
		sd_.ina1_voltage_V = ina1_.readBusVoltage()/1000;
		sd_.ina1_current_A = ina1_.readCurrent()/1000;
		sd_.ina1_power_W = sd_.ina1_current_A * sd_.ina1_voltage_V;

		ina2_.prime_i2c();
		sd_.ina2_voltage_V = ina2_.readBusVoltage()/1000;
		sd_.ina2_current_A = ina2_.readCurrent()/1000;
		sd_.ina2_power_W = sd_.ina2_current_A * sd_.ina2_voltage_V;
	}
	sd_.temp1_C = t1;
	sd_.temp2_C = t2;
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
	float trq1;
	float trq2;
	float motor_temp = sd_.temp1_C;
	float housing_temp = sd_.temp2_C;

	if (robotserver_fault()) return false;
	
	// Put conditions to check here

	return safe;
}
