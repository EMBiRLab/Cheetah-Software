#include <iomanip>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <utility>

#include <cmath>
#include <math.h>

#include "dynamometer.h"
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

Dynamometer::Dynamometer(Dynamometer::DynamometerSettings& dynset, std::ostream& datastream) : 
	a1_(dynset.a1_id, dynset.a1_bus, dynset.gear_a1, 1.0),
	a2_(dynset.a2_id, dynset.a2_bus, dynset.gear_a2, 1.0),
	datastream_(datastream),
	dynset_(dynset),
	commands_(),
	lpf_a1_(dynset.lpf_order, dynset.lpf_cutoff_freq_Hz, dynset.period_s),
	lpf_a2_(dynset.lpf_order, dynset.lpf_cutoff_freq_Hz, dynset.period_s),
	lpf_grp_(dynset.lpf_order, dynset.lpf_cutoff_freq_Hz, dynset.period_s) {

	if (dynset_.a1_id == dynset_.a2_id) {
		throw std::runtime_error("The servos must have unique IDs! Exiting...");
		ready_to_quit = true;
	}
	a_driving_ptr = &a1_;
	a_loading_ptr = &a2_;
	a1_.set_pos_lower_bound(std::numeric_limits<float>::quiet_NaN()); a1_.set_pos_upper_bound(std::numeric_limits<float>::quiet_NaN());
	a2_.set_pos_lower_bound(std::numeric_limits<float>::quiet_NaN()); a2_.set_pos_upper_bound(std::numeric_limits<float>::quiet_NaN());

	if (dynset_.dyn_opts["swap-actuators"].as<bool>()) swap_actuators();
	test_mode_ = dynset_.test_mode;
	if (dynset_.test_mode == Dynamometer::TestMode::kDurability) {
		if (dynset_.playback_file == "") {
			throw std::runtime_error("No replay file specified! Exiting...");
			ready_to_quit = true;
		}
		else {
			// Load mini cheetah test file.
			load_playback_data(dynset_.playback_file);
		}
	}

	if (!dynset_.skip_cal) {
		a1_.restore_cal("/home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_1_dyn.log");
		a2_.restore_cal("/home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_2_dyn.log");
	}
	if (!dynset_.skip_zero) {
		a1_.zero_offset();
		a2_.zero_offset();
	}
	if (!dynset_.skip_cfg_load) {
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
	if(dynset_.tqsen == Dynamometer::TorqueSensor::kTRS605_5) safety_s.ts_max_torque_Nm_ = safety_s.trs605_5_max_torque_Nm_;
	if(dynset_.tqsen == Dynamometer::TorqueSensor::kTRD605_18) safety_s.ts_max_torque_Nm_ = safety_s.trd605_18_max_torque_Nm_;

	dynset_.grp_sampling_period_us = static_cast<int64_t>(
		(1e6)/float(grp_j["random_sampling_frequency_Hz"]));

	dynset_.status_period_us = static_cast<int64_t>(
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

	if (dynset_.test_mode == Dynamometer::TestMode::kEfficiency) {
		efficiency_timer.start_timer(efficiency_s.idle_duration_s);
	}
	if (dynset_.test_mode == Dynamometer::TestMode::kDurability) {
		durability_timer.start_timer(0.5);
		durability_cycle_s = 
			2*efficiency_s.efficiency_sweep_duration_s
			+ 2*durability_s.damping_total_duration_s
			+ 4*durability_s.interphase_idle_duration_s
			+ durability_s.follow_duration_s;
		std::cout << "durability cycle time = " << durability_cycle_s <<" s" << std::endl;
		if (dynset_.durability_resume > 0.1) {
			initial_follow_delay = true;
			float time_delay = 0;
			time_delay =
				int(dynset_.durability_resume / durability_cycle_s)
				*(durability_s.follow_duration_s)
				+ fmod(dynset_.durability_resume, durability_cycle_s);
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
			std::cout << "resuming from previous test at " << dynset_.durability_resume << " s\n"
				<< "time delay into playback data = " << time_delay << " s\n"
				<< "playback_idx = " << playback_idx << std::endl;
		}
	}
}

void Dynamometer::load_playback_data(std::string file) {
	rapidcsv::Document doc(file);
	playback_vel = doc.GetColumn<float>("velocity [rad/s]");
	playback_trq = doc.GetColumn<float>("torque [Nm]");

	float Ts = 0.002;
	playback_idx = dynset_.playback_delay/Ts;
}

// vestigial from leg code but has useful filtering stuff
void Dynamometer::setup_playback() {
	std::cout
		<< "\tloading playback file from " << dynset_.playback_file << std::endl;
	rapidcsv::Document doc(dynset_.playback_file,
		rapidcsv::LabelParams(),
		rapidcsv::SeparatorParams(',',
															true),
		rapidcsv::ConverterParams(true, std::numeric_limits<float>::signaling_NaN(), 0),
		rapidcsv::LineReaderParams(true /* pSkipCommentLines */,
															'#' /* pCommentPrefix */,
															true /* pSkipEmptyLines */));

	std::vector<std::string> columnNames = doc.GetColumnNames();
	// for (auto& name : columnNames) std::cout << name << "\n";
	std::cout << "\t" << doc.GetColumnCount() << " columns, ";
	std::vector<float> time = doc.GetColumn<float>("time [s]");
	std::cout << time.size() << " rows" << std::endl;
	size_t delay_idx = std::upper_bound (
		time.begin(), time.end(), dynset_.playback_delay) - time.begin();
	std::vector<float>(time.begin()+delay_idx, time.end()).swap(time);
	
	// grab columns and erase elements before delay_idx
	a1_trq = doc.GetColumn<float>("a1 torque [Nm]");
	std::vector<float>(a1_trq.begin()+delay_idx, a1_trq.end()).swap(a1_trq);
	a2_trq = doc.GetColumn<float>("a2 torque [Nm]");
	std::vector<float>(a2_trq.begin()+delay_idx, a2_trq.end()).swap(a2_trq);

	// pad values for filtering
	std::cout << "\tpadding values for filtering... " << std::flush;
	for (size_t ii = 0; ii < dynset_.lpf_order; ii++) {
		a1_trq.push_back(a1_trq.back());
		a1_trq.insert(a1_trq.begin(), a1_trq.front());
		a2_trq.push_back(a2_trq.back());
		a2_trq.insert(a2_trq.begin(), a2_trq.front());
		time.push_back(time.back());
		time.insert(time.begin(), time.front());
	}
	size_t data_length = a1_trq.size();
	// take care of nans
	std::cout << "using " << data_length << " lines, taking care of nans...\n";
	for (size_t ii = 0; ii < data_length; ii++) {
		if (std::isnan(a1_trq[ii]))
			a1_trq[ii] = (ii == 0) ? 0 : a1_trq[ii-1];
		if (std::isnan(a2_trq[ii]))
			a2_trq[ii] = (ii == 0) ? 0 : a2_trq[ii-1];
	}
	data_length = a1_trq.size();
	std::cout << "\tallocating new vectors... " << std::flush;
	a1_trq_temp.resize(data_length);
	a2_trq_temp.resize(data_length);
	std::cout << "filtering playback data..." << std::flush;

	for (size_t ii = 0; ii < data_length-1; ii++) {
		// if (!(ii%100)) std::cout << ii << " " << std::flush;
		a1_trq_temp[ii] = lpf_a1_.iterate_filter(a1_trq[ii]);
		a2_trq_temp[ii] = lpf_a2_.iterate_filter(a2_trq[ii]);
	}
	lpf_a1_.flush_buffers();
	lpf_a2_.flush_buffers();
	
	// filter backwards to eliminate phase lag
	std::cout << " backward filter on " << data_length << " elements..." << std::flush;
	for (int ii = data_length-1; ii >= 0; ii--) {
		// if (!(ii%100)) std::cout << ii << " " << std::flush;
		a1_trq_temp[ii] = lpf_a1_.iterate_filter(a1_trq_temp[ii]);
		a2_trq_temp[ii] = lpf_a2_.iterate_filter(a2_trq_temp[ii]);
	}

	std::cout << " done.\n\twriting file...";
	// testing 
	std::ofstream data_file("filtering_test.csv");
	data_file << "time [s],a1 torque [Nm] in,a2 torque [Nm] in,a1 torque [Nm],a2 torque [Nm]\n";
	for (size_t ii = 0; ii < a1_trq.size(); ii++)
		data_file
			<< time[ii] << ", "
			<< a1_trq[ii] << ", "
			<< a2_trq[ii] << ", "
			<< a1_trq_temp[ii] << ", "
			<< a2_trq_temp[ii] << "\n";
	data_file.flush();
	std::cout << " done." << std::endl;

	// cut off padded elements
	std::vector<float>(
		a1_trq_temp.begin()+dynset_.lpf_order,
		a1_trq_temp.end()-dynset_.lpf_order).swap(a1_trq_temp);
	std::vector<float>(
		a2_trq_temp.begin()+dynset_.lpf_order,
		a2_trq_temp.end()-dynset_.lpf_order).swap(a2_trq_temp);


	//swap filtered data into vectors
	std::swap(a1_trq_temp, a1_trq);
	std::swap(a2_trq_temp, a2_trq);
}

void Dynamometer::iterate_fsm() {
	cycles_++;
	sample_sensors();
	// std::cout << "in iterate_fsm(), cycle = " << cycles_ << "\n" << std::flush;
	// immediately respond to fault condition
	if (dynamometer_fault()) next_state_ = 
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

	if ((time_prog_s_ > dynset_.test_delay) && (curr_state_ == FSMState::kIdle)) 
		next_state_ = FSMState::kRunning;
	// std::cout << "1 " << std::flush;

	switch (curr_state_) {
		case FSMState::kIdle: {
			// std::cout << "2 " << std::flush;

			a1_.make_stop();
			a2_.make_stop();
			break;}
		case FSMState::kStartup: {
			// std::cout << "2 " << std::flush;
			if (!encoder_offset_set) {
				if (a1_.fault() == MoteusController::errc::kSuccess 
						&& a2_.fault() == MoteusController::errc::kSuccess
						&& cycles_ >= 5) {
					float pos1 = a1_.get_position_rad();
					float pos2 = a2_.get_position_rad();
					encoder_offset = fmod(pos1+pos2, 1.0);
					std::cout << "initial encoder offset = " << encoder_offset << std::endl;
					encoder_offset_set = true;
					next_state_ = FSMState::kIdle;
					std::cout << "encoder offset set; transitioning to idle...\n";
				}
			}

			a1_.make_stop();
			a2_.make_stop();
			break;}
		case FSMState::kRunning: {
			// ***TODO***: Implement your operation here
			// feel free to expand into multiple states

			run_test();
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
			if(!dynamometer_fault()) next_state_ = recovery_return_state_;

			// if recovery hasn't occurred, quit
			if(dynamometer_fault()) next_state_ = FSMState::kQuitting;
			
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

void Dynamometer::log_data() {
	datastream_ << std::setw(10) << std::setprecision(4) << std::fixed
		<< time_prog_s_ << ",";
	// std::cout << " 3.1 " << std::flush;
	if (dynset_.test_mode == Dynamometer::TestMode::kGRP) {
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
		if(dynset_.test_mode == Dynamometer::TestMode::kDurability) {
			datastream_ << "," << (int)dts;
		}
		if(dynset_.test_mode == Dynamometer::TestMode::kEfficiency
				|| dynset_.test_mode == Dynamometer::TestMode::kDurability) {
			datastream_ << "," << (int)ets;
		}
	}
			
	datastream_ << "\n";
	
	return;
}

void Dynamometer::log_headers() {
	datastream_ << "# driving actuator id: " << a_driving_ptr->get_id() << "\n";
	datastream_ << "time [s],";
	if(dynset_.test_mode == Dynamometer::TestMode::kGRP) {
		datastream_ << "a1 velocity [rad/s],trs605-5 torque [Nm],a1 torque cmd [Nm],a1 torque est [Nm],c1 fault,c2 fault,a1 velocity cmd [rad/s],a1 position cmd [rad],a1 position [rad]\n";
	}
	else {
		datastream_ << a1_.stringify_actuator_header() << ","
		<< a1_.stringify_moteus_reply_header() << ","
		<< a2_.stringify_actuator_header() << ","
		<< a2_.stringify_moteus_reply_header() << ","
		<< stringify_sensor_data_headers() << ","
		<< "driving id" << ","
		<< "dynamometer fsm state";
		if(dynset_.test_mode == Dynamometer::TestMode::kDurability) {
			datastream_ << ",durability test state";
		}
		if(dynset_.test_mode == Dynamometer::TestMode::kEfficiency
				|| dynset_.test_mode == Dynamometer::TestMode::kDurability) {
			datastream_ << ",efficiency test state";
		}
	}
	// if (dynset_.test_mode == TestMode::kTorquePlayback)
	// 	datastream_ << 	",a1 playback torque [Nm],a2 playback torque [Nm]";
	
	datastream_ << "\n";
	
	return;
}

void Dynamometer::print_status_update() {
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
	if (dynset_.test_mode == Dynamometer::TestMode::kDurability) {
		std::cout << "dts:"
			<< std::setw(2) << std::setprecision(2) << std::fixed << (int)dts << "|";
	}
	if (dynset_.test_mode == Dynamometer::TestMode::kEfficiency) {
		std::cout << "ets:"
			<< std::setw(2) << std::setprecision(2) << std::fixed << (int)ets << "|";
		std::cout << "v=" << std::setw(5) << std::setprecision(2) << std::fixed << cond_vel;
		std::cout << ",t=" << std::setw(5) << std::setprecision(2) << std::fixed << cond_torque << "|";
		std::cout << "nt=" << std::setw(3) << std::setprecision(1) << std::fixed
			<< int(num_tested) << "|";
	}
	if (dynset_.test_mode == Dynamometer::TestMode::kEfficiency
							|| dynset_.test_mode == Dynamometer::TestMode::kDurability) {
		std::cout << "dp=" << std::setw(4) << std::setprecision(2) << std::fixed
			<< a1_.get_position_rad() + a2_.get_position_rad() << "|";
		std::cout << "V=" << std::setw(4) << std::setprecision(2) << std::fixed << sd_.ina1_voltage_V << "|";
	}
	if (dynset_.test_mode == Dynamometer::TestMode::kStep) {
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

	if(dynset_.test_mode == Dynamometer::TestMode::kTorqueStep) {
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

	if (!(bool)dynamometer_fault()) std::cout << color_factory.bg_grn() << color_factory.fg_blk() << " **SAFE**";
	else std::cout << "|" << color_factory.bg_red() << color_factory.fg_blk() << "**FAULT**";
	std::cout << color_factory.fg_def() << color_factory.bg_def() << "\r";
	std::cout.flush();
	return;
}

cxxopts::Options dyn_opts() {
	cxxopts::Options options(
		"leg", "Run 2D leg");

	options.add_options()
		("c,comment", "enter comment string to be included in output csv.", 
			cxxopts::value<std::string>())
		("p,path", "path to output csv.",
			cxxopts::value<std::string>()->default_value(
				"/home/pi/embir-modular-leg/leg-data/"))
		("playback-file", "path to csv of torque, velocity data to replay.", 
			cxxopts::value<std::string>()->default_value(""))
		("playback-delay", "jump into playback file after <delay> seconds", 
			cxxopts::value<float>()->default_value("1.0"))
		("playback-vel-scale", "scale velocity from playback data", 
			cxxopts::value<float>()->default_value("1.0"))
		("playback-trq-scale", "scale torque from playback data",
			cxxopts::value<float>()->default_value("1.0"))
		("lowpass-order", "butterworth lowpass filter order",
			cxxopts::value<int>()->default_value("5"))
		("lowpass-cutoff-frequency", "butterworth lowpass filter cutoff",
			cxxopts::value<float>()->default_value("50"))
		("gear-a1", "gear ratio of a1 actuator, as a reduction", 
			cxxopts::value<float>()->default_value("1.0"))
		("gear-a2", "gear ratio of a2 actuator, as a reduction",	
			cxxopts::value<float>()->default_value("1.0"))
		("a1-id", "a1 actuator CAN ID",
			cxxopts::value<uint8_t>()->default_value("1"))
		("a2-id", "a2 actuator CAN ID",
			cxxopts::value<uint8_t>()->default_value("2"))
		("a1-bus", "a1 actuator CAN bus",
			cxxopts::value<uint8_t>()->default_value("3"))
		("a2-bus", "a2 actuator CAN bus",
			cxxopts::value<uint8_t>()->default_value("4"))
		("main-cpu", "main CPU", cxxopts::value<uint8_t>()->default_value("1"))
		("can-cpu", "CAN CPU", cxxopts::value<uint8_t>()->default_value("2"))
		("duration", "test duration in seconds", cxxopts::value<float>())
		("torquesensor", "which torque sensor is being used of [trd605-18|trs605-5]",
			cxxopts::value<std::string>()->default_value("trs605-5"))
		("frequency", "test sampling and command frequency in Hz", 
			cxxopts::value<float>()->default_value("250"))
		("skip-cal", "skip recalibration")
		("skip-zero", "skip setting actuator zero")
		("skip-cfg-load", "skip loading actuator config")
		("config-path", "path to output csv.",
			cxxopts::value<std::string>()->default_value(
				"/home/pi/embir-modular-leg/moteus-setup/moteus-cfg/a_gen.cfg"))
		("test-mode", "choose between [none|cyclic-crouch|torque-playback|jump]",
			cxxopts::value<std::string>()->default_value("none"))
		("test-delay", "delay start of test", 
			cxxopts::value<float>()->default_value("2"))
		("durability-resume", "jump back into a durability test", 
			cxxopts::value<float>()->default_value("0"))
		("swap-actuators", "start with actuator roles swapped (driving vs. load)")
		("h,help", "Print usage")

	;

	// test deps
	return options;
}

Dynamometer::DynamometerSettings::DynamometerSettings(cxxopts::ParseResult& dyn_opts_in) {
	dyn_opts = dyn_opts_in;
	period_s = 1.0/dyn_opts["frequency"].as<float>();
	duration_s = dyn_opts["duration"].as<float>();
	gear_a1 = dyn_opts["gear-a1"].as<float>();
	gear_a2 = dyn_opts["gear-a2"].as<float>();
	a1_id = dyn_opts["a1-id"].as<uint8_t>();
	a2_id = dyn_opts["a2-id"].as<uint8_t>();
	a1_bus = dyn_opts["a1-bus"].as<uint8_t>();
	a2_bus = dyn_opts["a2-bus"].as<uint8_t>();

	main_cpu = dyn_opts["main-cpu"].as<uint8_t>();
	can_cpu = dyn_opts["can-cpu"].as<uint8_t>();

	playback_vel_scale = dyn_opts["playback-vel-scale"].as<float>();
	playback_trq_scale = dyn_opts["playback-trq-scale"].as<float>();

	lpf_order = dyn_opts["lowpass-order"].as<int>();
	lpf_cutoff_freq_Hz = dyn_opts["lowpass-cutoff-frequency"].as<float>();

	playback_file = dyn_opts["playback-file"].as<std::string>();
	// std::cerr << playback_file << std::endl;
	playback_delay = dyn_opts["playback-delay"].as<float>();
	durability_resume = dyn_opts["durability-resume"].as<float>();

	auto test_str = dyn_opts["test-mode"].as<std::string>();
	// case invariance -- convert to user input to lowercase
	std::transform(test_str.begin(), test_str.end(), test_str.begin(),
		[](unsigned char c){ return std::tolower(c); });

	// std::cout << test_str << (test_str == std::string("TV-sweep")) << std::endl;
	if (test_str == std::string("none")) test_mode = Dynamometer::TestMode::kNone;
	else if (test_str == std::string("torque-constant")) test_mode = Dynamometer::TestMode::kTorqueConstant;
	else if (test_str == std::string("grp")) test_mode = Dynamometer::TestMode::kGRP;
	else if (test_str == std::string("direct-damping")) test_mode = Dynamometer::TestMode::kDirectDamping;
	else if (test_str == std::string("torque-vel-sweep")) test_mode = Dynamometer::TestMode::kTorqueVelSweep;
	else if (test_str == std::string("durability")) test_mode = Dynamometer::TestMode::kDurability;
	else if (test_str == std::string("efficiency")) test_mode = Dynamometer::TestMode::kEfficiency;
	else if (test_str == std::string("step")) test_mode = Dynamometer::TestMode::kStep;
	else if (test_str == std::string("torque-step")) test_mode = Dynamometer::TestMode::kTorqueStep;
	else if (test_str == std::string("manual")) test_mode = Dynamometer::TestMode::kManual;
	else {
		test_mode = Dynamometer::TestMode::kNone;
		std::cout << "no test mode selected (input was \"" << test_str << "\")." << std::endl;
	}
	std::cout << "test mode \"" << test_str << "\" selected" << std::endl;
	auto tqsen_str = dyn_opts["torquesensor"].as<std::string>();
	std::transform(tqsen_str.begin(), tqsen_str.end(), tqsen_str.begin(),
		[](unsigned char c){ return std::tolower(c); });
	if (tqsen_str == std::string("trd605-18")) tqsen = Dynamometer::TorqueSensor::kTRD605_18;
	else if (tqsen_str == std::string("trs605-5")) tqsen = Dynamometer::TorqueSensor::kTRS605_5;
	
	
	test_delay = dyn_opts["test-delay"].as<float>();

	skip_cal = dyn_opts["skip-cal"].as<bool>();
	skip_zero = dyn_opts["skip-zero"].as<bool>();
	skip_cfg_load = dyn_opts["skip-cfg-load"].as<bool>();
	cfg_path = dyn_opts["config-path"].as<std::string>();
}

void Dynamometer::run_test() {
	// std::cout << "in run_test()" << std::endl;
	
	switch (test_mode_) {
		case TestMode::kNone: {
			a1_.make_stop();
			a2_.make_stop();
			break;}
		case TestMode::kTorqueConstant: {
			break;}
		case TestMode::kGRP: {
			float rand_cmd = filtered_random();
			
			if (grp_s.grp_control_mode == GRPSettings::GRPControlMode::kGRPTorque
					|| grp_s.grp_control_mode == GRPSettings::GRPControlMode::kGRPCurrent)
				a_driving_ptr->make_act_torque(rand_cmd);
			else if (grp_s.grp_control_mode == GRPSettings::GRPControlMode::kGRPVelocity)
				a_driving_ptr->make_act_velocity(rand_cmd, 0);
			else if (grp_s.grp_control_mode == GRPSettings::GRPControlMode::kGRPPosition)
				a_driving_ptr->make_act_position(rand_cmd, 0);
			else {
				throw std::runtime_error("Control mode not recognized! Exiting...");
				ready_to_quit = true;
			}

			// a_driving_ptr->make_stop();
			// a_driving_ptr->make_act_position(rand_cmd, 0);
			// a_loading_ptr->make_stop();
			break;}
		case TestMode::kDirectDamping: {
			break;}
		case TestMode::kTorqueVelSweep: {
			break;}
		case TestMode::kEfficiency: {
			iterate_efficiency_fsm();
			break;}
		case TestMode::kDurability: {
			iterate_durability_fsm();
			break;}
		case TestMode::kStep: {
			break;}
		case TestMode::kTorqueStep: {
			// a_loading_ptr->make_stop();
			if(time_fcn_s_ > torque_step_s.next_transition_s) {
				// transition modes here
				torque_step_s.is_paused = !torque_step_s.is_paused;
				// set next transition time
				if(torque_step_s.is_paused)
					torque_step_s.next_transition_s += torque_step_s.pause_time_s;
				else {
					torque_step_s.next_transition_s += torque_step_s.step_time_s;
					// progress index
					torque_step_s.torque_idx 
						= (torque_step_s.torque_idx+1) % (torque_step_s.torques_Nm.size());
				}
			}
			float torque_cmd = 0;
			if(!torque_step_s.is_paused)
				torque_cmd = torque_step_s.torques_Nm[torque_step_s.torque_idx];
			
			a_driving_ptr->make_act_torque(torque_cmd);
			// a_loading_ptr->make_stop();
			break;}
		case TestMode::kManual: {
			break;}
		default: {
			a1_.make_stop();
			a2_.make_stop();
			break;}
	}
	return;
}

void Dynamometer::iterate_durability_fsm() {
	// double fsm_prog_now = time_prog_s_;
	// double fsm_func_now = time_fcn_s_;

	durability_timer.update_time(time_fcn_s_);
	switch (dts) {
		case DurabilityTestState::kDurabilityIdle: {
			// std::cerr << "in dts idle, dts_after_idle = " << (int)dts_after_idle << std::endl;
			if (durability_timer.timer_done()) {
				// std::cout << "in idle, timer done" << std::endl;
				dts = dts_after_idle;
				std::cout << std::endl << "transitioning from Idle to ";
				switch (dts) {
				case DurabilityTestState::kDurabilityFollow:{
					std::cout << "Follow";
					durability_timer.start_timer(durability_s.follow_duration_s);
					if (initial_follow_delay) {
						// subtract out part already completed in this cycle
						durability_timer.start_timer(
							durability_s.follow_duration_s
							- fmod(dynset_.durability_resume, durability_cycle_s)
						);
						initial_follow_delay = false;
						std::cout << "first follow will be " <<
							durability_s.follow_duration_s
							- fmod(dynset_.durability_resume, durability_cycle_s)
							<< " s long\n";
					}
				break;}
				case DurabilityTestState::kDurabilityEfficiency:{
					std::cout << "Efficiency";
					durability_timer.start_timer(0);
				break;}
				case DurabilityTestState::kDurabilityTorqueDamping:{
					std::cout << "Torque Damping";
					durability_timer.start_timer(durability_s.condition_duration_s);
				break;}
				case DurabilityTestState::kDurabilityVelocityDamping:{
					std::cout << "Velocity Damping";
					durability_timer.start_timer(durability_s.condition_duration_s);
				break;}
				default:{
					std::cout << "unknown dts!!" << std::endl;
					ready_to_quit = true;
				break;}
				}
				std::cout << std::endl;
				// std::cout << std::endl << "transitioning from Idle to Follow" << std::endl;
			}
			if (num_swaps >= 2) {
				std::cout << "ran thru one cycle for each actuator; exiting..." << std::endl;
				next_state_ = FSMState::kQuitting;
			}
			a1_.make_stop();
			a2_.make_stop();
		break;}
		case DurabilityTestState::kDurabilityFollow:{
			if (durability_timer.timer_done()) {
				durability_timer.start_timer(durability_s.interphase_idle_duration_s);
				// std::cout << "timer set for " << durability_s.interphase_idle_duration_s << std::endl;
				dts = DurabilityTestState::kDurabilityIdle;
				// dts_after_idle = DurabilityTestState::kDurabilityGRP;
				dts_after_idle = DurabilityTestState::kDurabilityEfficiency;
				std::cout << std::endl << "transitioning from Follow to Idle" << std::endl;
			}
			// step index
			// get new torque and vel values
			// assign them into commands
			float vel = dynset_.playback_vel_scale * playback_vel[playback_idx];
			float trq = dynset_.playback_trq_scale * playback_trq[playback_idx];

			// clip to max values
			vel = vel > durability_s.velocity_max_rad_s ? durability_s.velocity_max_rad_s : vel;
			vel = vel < -durability_s.velocity_max_rad_s ? -durability_s.velocity_max_rad_s : vel;
			trq = trq > durability_s.torque_max_Nm ? durability_s.torque_max_Nm : trq;
			trq = trq < -durability_s.torque_max_Nm ? -durability_s.torque_max_Nm : trq;

			++playback_idx;
			if (playback_idx == playback_trq.size()) {
				std::cout << "looping replay file and swapping actuators (30s pause)..." << std::endl;
				playback_idx = 0;
				swap_actuators();
				num_swaps++;
				ready_to_quit = true;
				durability_timer.start_timer(30);
				dts = DurabilityTestState::kDurabilityIdle;
				dts_after_idle = DurabilityTestState::kDurabilityFollow;
				std::cout << std::endl << "transitioning from Follow to Idle" << std::endl;
			}
			
			a_driving_ptr->make_act_torque(trq);
			a_loading_ptr->set_kd_scale(10.0);
			a_loading_ptr->make_act_velocity(-vel, 0.8*trq);
			
		break;}
		case DurabilityTestState::kDurabilityGRP:{
			if (durability_timer.timer_done()) {
				durability_timer.start_timer(durability_s.interphase_idle_duration_s);
				dts = DurabilityTestState::kDurabilityIdle;
				dts_after_idle = DurabilityTestState::kDurabilityEfficiency;
				std::cout << std::endl << "transitioning from DurabilityGRP to Idle" << std::endl;
			}
			// float rand_cmd = filtered_random();
			
			// a_driving_ptr->make_act_torque(rand_cmd);
			a_driving_ptr->make_stop();
			a_loading_ptr->make_stop();
		break;}
		case DurabilityTestState::kDurabilityEfficiency: {
			iterate_efficiency_fsm();

			if (efficiency_done) {
				reset_efficiency();
				swap_actuators();
				num_efficiency_sweeps++;
				durability_timer.start_timer(0.25);
				dts = DurabilityTestState::kDurabilityIdle;
				dts_after_idle = DurabilityTestState::kDurabilityEfficiency;
			}
			// transition when we're on the last index and timer has run out
			if (num_efficiency_sweeps >= 2 && durability_timer.timer_done()) {
				durability_timer.start_timer(durability_s.interphase_idle_duration_s);
				dts = DurabilityTestState::kDurabilityIdle;
				dts_after_idle = DurabilityTestState::kDurabilityTorqueDamping;
				std::cout << std::endl << "transitioning from DurabilityEfficiency to Idle" << std::endl;
				num_efficiency_sweeps = 0; // reset index for next round
			}
		break;}
		case DurabilityTestState::kDurabilityTorqueDamping: {
			float torque_cond = durability_s.torques_Nm[trq_damp_idx];
			float torque_cmd = torque_cond;
			float condition_time = durability_s.condition_duration_s - durability_timer.time_left();
			float ramp_time = 0.25;
			float slope = torque_cond/ramp_time;
			if (condition_time < ramp_time)
				torque_cmd = condition_time * slope;
			else if (durability_timer.time_left() < ramp_time)
				torque_cmd = durability_timer.time_left() * slope;
			torque_cmd = fabs(torque_cmd) > fabs(torque_cond) ? torque_cond : torque_cmd;
			torque_cmd = signbit(torque_cmd) != signbit(torque_cond) ? 0 : torque_cmd;

			a_driving_ptr->make_act_torque(torque_cmd);
			a_loading_ptr->make_stop();
			if (durability_timer.timer_done()) {
				if (trq_damp_idx < durability_s.torques_Nm.size()) {
					trq_damp_idx++;
					// durability_timer.start_timer(durability_s.condition_duration_s);
					durability_timer.start_timer(durability_s.intercondition_idle_duration_s);
					dts = DurabilityTestState::kDurabilityIdle;
					dts_after_idle = DurabilityTestState::kDurabilityTorqueDamping;
				}
				if (trq_damp_idx >= durability_s.torques_Nm.size()) { // idx past size -- finished a sweep
					trq_damp_idx = 0;
					swap_actuators();
					num_trq_sweeps++;
				}
				// transition if we've done two sweeps
				if((num_trq_sweeps >= 2)) {
					durability_timer.start_timer(durability_s.interphase_idle_duration_s);
					dts = DurabilityTestState::kDurabilityIdle;
					dts_after_idle = DurabilityTestState::kDurabilityVelocityDamping;
					num_trq_sweeps = 0;
					trq_damp_idx = 0;
				}
			}
		break;}
		case DurabilityTestState::kDurabilityVelocityDamping: {
			float velocity_cond = durability_s.velocities_rad_s[vel_damp_idx];
			float velocity_cmd = velocity_cond;
			float condition_time = durability_s.condition_duration_s - durability_timer.time_left();
			float ramp_time = 0.25;
			float slope = velocity_cond/ramp_time;
			if (condition_time < ramp_time)
				velocity_cmd = condition_time * slope;
			else if (durability_timer.time_left() < ramp_time)
				velocity_cmd = durability_timer.time_left() * slope;
			velocity_cmd = fabs(velocity_cmd) > fabs(velocity_cond) ? velocity_cond : velocity_cmd;
			velocity_cmd = signbit(velocity_cmd) != signbit(velocity_cond) ? 0 : velocity_cmd;

			a_driving_ptr->make_act_velocity(velocity_cmd,0);
			a_loading_ptr->make_stop();
			if (durability_timer.timer_done()) {
				if (vel_damp_idx < durability_s.velocities_rad_s.size()) {
					vel_damp_idx++;
					durability_timer.start_timer(durability_s.intercondition_idle_duration_s);
					dts = DurabilityTestState::kDurabilityIdle;
					dts_after_idle = DurabilityTestState::kDurabilityVelocityDamping;
				}
				if (vel_damp_idx >= durability_s.velocities_rad_s.size()) { // idx past size -- finished a sweep
					vel_damp_idx = 0;
					swap_actuators();
					num_vel_sweeps++;
				}
				// transition if we've done two sweeps
				if((num_vel_sweeps >= 2)) {
					durability_timer.start_timer(durability_s.interphase_idle_duration_s);
					dts = DurabilityTestState::kDurabilityIdle;
					dts_after_idle = DurabilityTestState::kDurabilityFollow;
					num_vel_sweeps = 0;
					vel_damp_idx = 0;
				}
			}
		break;}
		case DurabilityTestState::kDurabilityNone: {
			a_driving_ptr->make_stop();
			a_loading_ptr->make_stop();
		break;}
	}
}

void Dynamometer::iterate_efficiency_fsm() {
	// double fsm_prog_now = time_prog_s_;
	// double fsm_func_now = time_fcn_s_;
	efficiency_timer.update_time(time_fcn_s_);

	cond_vel = efficiency_s.velocities_rad_s[efficiency_vel_idx];
	cond_torque = efficiency_s.torques_Nm[efficiency_torque_idx];
	// check validity of condition
	size_t num_skips = 0;
	while (exceeds_limit(cond_torque, cond_vel)) {
		iterate_condition();
		cond_vel = efficiency_s.velocities_rad_s[efficiency_vel_idx];
		cond_torque = efficiency_s.torques_Nm[efficiency_torque_idx];
		num_skips++;
	}
	if (num_skips) std::cout << "skipped " << (int)num_skips << " conditions\n";
	if (fabs(sd_.torque_Nm) > safety_s.ts_max_torque_Nm_) {
		// immediately reset to beginning of sequence, zero out commands,
		// and move to next test condition
		ets = EfficiencyTestState::kEfficiencyIdle;
		iterate_condition();
	}
	cond_torque *= (positive_work ? 1 : -1);

	switch (ets) {
		case EfficiencyTestState::kEfficiencyIdle: {
			a_driving_ptr->make_act_velocity(0,0);
			a_loading_ptr->make_act_torque(0);
			if (efficiency_timer.timer_done()) {
				efficiency_timer.start_timer(efficiency_s.ramp_up_vel_duration_s);
				ets = EfficiencyTestState::kEfficiencyRampUpVel;
				// std::cout << std::endl << "transitioning from Idle to RampUpVelocity" << std::endl;
			}
			break;
		}
		case EfficiencyTestState::kEfficiencyRampUpVel: {
			float vel_slope = cond_vel / efficiency_s.ramp_up_vel_duration_s;
			// time = duration - time left
			float ramp_time = efficiency_s.ramp_up_vel_duration_s - (efficiency_timer.time_left());

			float ramp_vel = vel_slope * ramp_time;
			ramp_vel = fabs(ramp_vel) > fabs(cond_vel) ? cond_vel : ramp_vel;
			a_driving_ptr->make_act_velocity(ramp_vel,0);
			a_loading_ptr->make_act_torque(0);
			if (efficiency_timer.timer_done()) {
				efficiency_timer.start_timer(efficiency_s.lag_up_duration_s);
				ets = EfficiencyTestState::kEfficiencyLagUp;
				// std::cout << std::endl << "transitioning from RampUpVelocity to LagUp" << std::endl;
			}
			break;
		}
		case EfficiencyTestState::kEfficiencyLagUp: {
			a_driving_ptr->make_act_velocity(cond_vel,0);
			a_loading_ptr->make_act_torque(0);
			if (efficiency_timer.timer_done()) {
				efficiency_timer.start_timer(efficiency_s.ramp_up_torque_duration_s);
				ets = EfficiencyTestState::kEfficiencyRampUpTorque;
				// std::cout << std::endl << "transitioning from LagUp to RampUpTorque" << std::endl;
			}
			break;
		}
		case EfficiencyTestState::kEfficiencyRampUpTorque: {
			float torque_slope = cond_torque / efficiency_s.ramp_up_torque_duration_s;
			// time = duration - time left
			float ramp_time = efficiency_s.ramp_up_torque_duration_s - (efficiency_timer.time_left());

			float ramp_torque = torque_slope * ramp_time;
			ramp_torque = fabs(ramp_torque) > fabs(cond_torque) ? cond_torque : ramp_torque;

			a_driving_ptr->make_act_velocity(cond_vel,1.0*ramp_torque);
			a_loading_ptr->make_act_torque(ramp_torque);
			if (efficiency_timer.timer_done()) {
				efficiency_timer.start_timer(efficiency_s.hold_condition_duration_s);
				ets = EfficiencyTestState::kEfficiencyHoldCondition;
				// std::cout << std::endl << "transitioning from RampUpTorque to HoldCondition" << std::endl;
			}
			break;
		}
		case EfficiencyTestState::kEfficiencyHoldCondition: {
			a_driving_ptr->make_act_velocity(cond_vel,1.0*cond_torque);
			a_loading_ptr->make_act_torque(cond_torque);
			if (efficiency_timer.timer_done()) {
				efficiency_timer.start_timer(efficiency_s.ramp_down_torque_duration_s);
				ets = EfficiencyTestState::kEfficiencyRampDownTorque;
				// std::cout << std::endl << "transitioning from HoldCondition to RampDownTorque" << std::endl;
			}
			break;
		}
		case EfficiencyTestState::kEfficiencyRampDownTorque: {
			float torque_slope = cond_torque / efficiency_s.ramp_up_torque_duration_s;
			// time = duration - time left
			float ramp_time = efficiency_s.ramp_down_torque_duration_s - (efficiency_timer.time_left());

			float ramp_torque = (-torque_slope * ramp_time) + cond_torque;
			ramp_torque = signbit(ramp_torque) != signbit(cond_torque) ? 0 : ramp_torque;

			a_driving_ptr->make_act_velocity(cond_vel,0);
			a_loading_ptr->make_act_torque(ramp_torque);
			if (efficiency_timer.timer_done()) {
				efficiency_timer.start_timer(efficiency_s.lag_down_duration_s);
				ets = EfficiencyTestState::kEfficiencyLagDown;
				// std::cout << std::endl << "transitioning from RampDownTorque to LagDown" << std::endl;
			}
			break;
		}
		case EfficiencyTestState::kEfficiencyLagDown: {
			a_driving_ptr->make_act_velocity(cond_vel,0);
			a_loading_ptr->make_act_torque(0);
			if (efficiency_timer.timer_done()) {
				efficiency_timer.start_timer(efficiency_s.ramp_down_vel_duration_s);
				ets = EfficiencyTestState::kEfficiencyRampDownVelocity;
				// std::cout << std::endl << "transitioning from LagDown to RampDownVelocity" << std::endl;
			}
			break;
		}
		case EfficiencyTestState::kEfficiencyRampDownVelocity: {
			float vel_slope = cond_vel / efficiency_s.ramp_up_vel_duration_s;
			// time = duration - time left
			float ramp_time = efficiency_s.ramp_down_vel_duration_s - (efficiency_timer.time_left());

			float ramp_vel = (-vel_slope * ramp_time) + cond_vel;
			ramp_vel = signbit(ramp_vel) != signbit(cond_vel) ? 0 : ramp_vel;
			a_driving_ptr->make_act_velocity(ramp_vel,0);
			a_loading_ptr->make_act_torque(0);
			if (efficiency_timer.timer_done()) {
				efficiency_timer.start_timer(efficiency_s.idle_duration_s);
				ets = EfficiencyTestState::kEfficiencyIdle;
				// std::cout << std::endl << "transitioning from RampDownVelocity to Idle" << std::endl;
				if(!positive_work) {
					positive_work = true;
					iterate_condition();
				}
				else { // if we just did positive work, switch to negative (don't iterate conditions)
					positive_work = false;
				}
			}
			break;
		}
		case EfficiencyTestState::kEfficiencyNone: {
			a_driving_ptr->make_stop();
			a_loading_ptr->make_stop();
			break;
		}
	}
	if (test_mode_ == Dynamometer::TestMode::kEfficiency && efficiency_done)
		ready_to_quit = true;
}

void Dynamometer::sample_random() {
	std::mt19937 gen(rd_());
	random_sample = realdist(gen);
}

float Dynamometer::filtered_random() {
	float rand_cmd = lpf_grp_.iterate_filter(random_sample);

	rand_cmd = (rand_cmd > grp_s.amplitude) ? grp_s.amplitude : rand_cmd;
	rand_cmd = (rand_cmd < -grp_s.amplitude) ? -grp_s.amplitude : rand_cmd;

	return rand_cmd;
}

void Dynamometer::sample_sensors() {
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
	
	auto testmode = dynset_.test_mode;

	// conditions for reading torque sensor
	if (testmode != TestMode::kDurability && testmode != TestMode::kTorqueStep
		&& !(testmode == TestMode::kGRP 
				&& (grp_s.grp_control_mode == GRPSettings::GRPControlMode::kGRPPosition
						|| grp_s.grp_control_mode == GRPSettings::GRPControlMode::kGRPCurrent))) {
		// run faster in GRP mode
		ads_.prime_i2c();
		uint16_t adc = ads_.readADC_SingleEnded(0);
		float torque_volts = ads_.computeVolts(adc);
		//float torque_volts = 0.0;
		float tqsen_gain = (dynset_.tqsen==TorqueSensor::kTRD605_18) ? 18.0/5.0 : 5.0/5.0;
		sd_.torque_Nm = -(torque_volts - 5.0/3.0) * 3.0;
		sd_.torque_Nm *= tqsen_gain;
	}
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

	if (fabs(sd_.torque_Nm) > safety_s.ts_max_torque_Nm_) {
		std::cout << "over torque condition!! torque sensor: " << sd_.torque_Nm << " Nm \n";
		// ready_to_quit = true;
	}
}

std::string Dynamometer::stringify_sensor_data() {
	std::ostringstream result;
	sprintf(cstr_buffer, "%f", sd_.torque_Nm);
	result << cstr_buffer;
	if(dynset_.test_mode == TestMode::kGRP) return result.str();

	sprintf(cstr_buffer, ",%f,%f,", sd_.temp1_C, sd_.temp2_C);
	result << cstr_buffer;
	sprintf(cstr_buffer, "%f,%f,%f,", sd_.ina1_voltage_V, sd_.ina1_current_A, sd_.ina1_power_W);
	result << cstr_buffer;
	sprintf(cstr_buffer, "%f,%f,%f", sd_.ina2_voltage_V, sd_.ina2_current_A, sd_.ina2_power_W);
	result << cstr_buffer;
	return result.str();
}

std::string Dynamometer::stringify_sensor_data_headers() {
	std::ostringstream result;
	result << ((dynset_.tqsen == TorqueSensor::kTRD605_18) ? "trd605-18 torque [Nm]" : "trs605-5 torque [Nm]");
	if(dynset_.test_mode == TestMode::kGRP) return result.str();

	result << ",motor temp [C],housing temp [C],ina1 voltage [V],ina1 current [A],ina1 power [W],ina2 voltage [V],ina2 current [A],ina2 power [W]";
	return result.str();
}


bool Dynamometer::safety_check() {
	//
	bool safe = true;
	float trq1;
	float trq2;
	float motor_temp = sd_.temp1_C;
	float housing_temp = sd_.temp2_C;

	if (dynamometer_fault()) return false;

	trq1 = a1_.get_torque_Nm();
	trq2 = a2_.get_torque_Nm();

	float disparity = fabs(trq2-trq1)/trq1;

	std::rotate(safety_s.disparity_buffer.rbegin(),
		safety_s.disparity_buffer.rbegin()+1, safety_s.disparity_buffer.rend());
	safety_s.disparity_buffer[0] = disparity;
	disparity = 0;
	for (float disp : safety_s.disparity_buffer) disparity += disp;
	disparity /= safety_s.disparity_buffer.size();

	if ((fabs(trq1) > 0.5 || fabs(trq2) > 0.5) 
		&& (fabs(trq2-trq1)/trq1 > safety_s.actuator_torque_disparity_ratio_)) {
		
		// safe &= false;
		// std::cout << "torque disparity!! trq1 = " << trq1 << ", trq2 = " << trq2 << std::endl;
	}

	// float current_encoder_offset = fmod((reply1.result.position + reply2.result.position), 1.0);
	float current_encoder_offset = (a1_.get_position_rad() + a2_.get_position_rad());

	// latch unsafe if encoder offset changes by more than 0.05 rotations,
	// which represents a rotor slip
	// std::cout << fabs(current_encoder_offset - encoder_offset) << std::endl;
	// safe &= (fabs(current_encoder_offset - encoder_offset) < 0.05);
	// std::cout << safe << std::endl;

	std::rotate(safety_s.motor_temp_buffer.rbegin(),
		safety_s.motor_temp_buffer.rbegin()+1, safety_s.motor_temp_buffer.rend());
	safety_s.motor_temp_buffer[0] = motor_temp;
	motor_temp = 0;
	for (float temp : safety_s.motor_temp_buffer) motor_temp += temp;
	motor_temp /= safety_s.motor_temp_buffer.size();

	std::rotate(safety_s.housing_temp_buffer.rbegin(),
		safety_s.housing_temp_buffer.rbegin()+1, safety_s.housing_temp_buffer.rend());
	safety_s.housing_temp_buffer[0] = housing_temp;
	housing_temp = 0;
	for (float temp : safety_s.housing_temp_buffer) housing_temp += temp;
	housing_temp /= safety_s.housing_temp_buffer.size();

	if (overtemp_latch && motor_temp < (safety_s.max_motor_temp_C_ - 15) && housing_temp < (safety_s.max_housing_temp_C_ - 15))
		overtemp_latch = false;
	if (motor_temp > safety_s.max_motor_temp_C_ || housing_temp > safety_s.max_housing_temp_C_)
		overtemp_latch = true;

	safe &= !overtemp_latch;

	// if (dynset_.tqsen == Dynamometer::TorqueSensor::kTRS605_5)
	// 	safe &= sd_.torque_Nm < safety_s.trs605_5_max_torque_Nm_;
	// if (dynset_.tqsen == Dynamometer::TorqueSensor::kTRD605_18)
	// 	safe &= sd_.torque_Nm < safety_s.trd605_18_max_torque_Nm_;

	safe &= sd_.torque_Nm < safety_s.ts_max_torque_Nm_;
	// safe if voltages are above min and below max
	safe &= (sd_.ina1_voltage_V > safety_s.min_bus_VDC || sd_.ina1_voltage_V < safety_s.max_bus_VDC);
	safe &= (sd_.ina2_voltage_V > safety_s.min_bus_VDC || sd_.ina2_voltage_V < safety_s.max_bus_VDC);

	dynamometer_safe = safe;
	if(!safe) {
		// std::cout << "unsafe condition detected!!" << std::endl;
		// std::cout << "\tovertemp_latch = " << overtemp_latch
		//	 << ",\n\tmotor_temp = " << motor_temp
		//	 << ",\n\thousing_temp = " << housing_temp
		//	 << ",\n\ttrq1 = " <<	trq1
		//	 << ",\n\ttrq2 = " <<	trq2 << std::endl;
	}
	return safe;
}
