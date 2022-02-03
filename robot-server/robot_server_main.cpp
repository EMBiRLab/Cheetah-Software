#include <sys/mman.h>

#include <vector>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <fstream>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <cstdio>
#include <ctime>
#include <csignal>

#include "third-party/mjbots/moteus/moteus_protocol.h"
#include "third-party/mjbots/moteus/pi3hat_moteus_interface.h"

#include "robot_server.h"
#include "utils.h"

#include "third-party/iir/iir.h"
#include "third-party/nlohmann/json.hpp"
#include "third-party/cxxopts/cxxopts.hpp"

#ifndef PI
#define PI 3.1415926
#endif

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;
using json = nlohmann::json;
namespace chron = std::chrono;

char cstr_buffer[128];

volatile sig_atomic_t interrupted=false; 

void sig_handle(int s) {
	interrupted = true;
}

void setup_sigint_catch() {
	// ** CTRL+C CATCH **
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = sig_handle;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
}

void Run(RobotServer& robotserver) {
	if (robotserver.is_ready_to_quit()) return;
	// * SETUP *
	RobotServer::RobotServerSettings& rs_settings = robotserver.get_dynset();
	
	setup_sigint_catch();

	// ** CONFIGURE CPU AND MOTEUS INFRASTRUCTURE **
	std::cout << "configuring realtime and constructing moteus interface... ";
	moteus::ConfigureRealtime(rs_settings.main_cpu);
	MoteusInterface::Options moteus_options;
	moteus_options.cpu = rs_settings.can_cpu;

	moteus_options.servo_bus_map = robotserver.create_servo_bus_map();
	MoteusInterface moteus_interface{moteus_options};
	std::cout << "done.\n" << std::flush;

	// ** CONTAINER FOR COMMANDS **
	std::vector<MoteusInterface::ServoCommand> curr_commands;
	std::vector<MoteusInterface::ServoCommand> prev_commands;
	for (const auto& pair : moteus_options.servo_bus_map) {
		curr_commands.push_back({});
		curr_commands.back().id = pair.first;
		prev_commands.push_back({});
		prev_commands.back().id = pair.first;
	}

	// ** CONTAINER FOR REPLIES **
	std::vector<MoteusInterface::ServoReply> replies{curr_commands.size()};
	std::vector<MoteusInterface::ServoReply> saved_replies{curr_commands.size()};

	// ** PACKAGE COMMANDS AND REPLIES IN moteus_data **
	std::cout << "setting up moteus_data container... ";
	MoteusInterface::Data moteus_data;
	moteus_data.commands = { curr_commands.data(), curr_commands.size() };
	moteus_data.replies = { replies.data(), replies.size() };
	std::cout << "done.\n" << std::flush;

	std::future<MoteusInterface::Output> can_result;

	// ** TIMING **
	const auto period =
			chron::microseconds(static_cast<int64_t>(rs_settings.period_s * 1e6));
	auto next_cycle = chron::steady_clock::now() + period;

	const auto status_period = chron::microseconds(rs_settings.status_period_us);
	auto next_status = next_cycle + status_period;

	uint64_t cycle_count = 0; uint64_t total_skip_count = 0;
	uint64_t margin_cycles = 0; uint64_t reply_miss_count = 0;
	double total_margin = 0.0;

	// * IMMEDIATE PRE LOOP *
	auto t0 = chron::steady_clock::now();
	robotserver.set_time0(t0);
	robotserver.log_headers();
	std::cout << "beginning while loop..." << std::endl;

	// * MAIN LOOP *
	while (!interrupted
			&& robotserver.get_time_prog() < rs_settings.duration_s
			&& !robotserver.is_ready_to_quit()) {
		cycle_count++; margin_cycles++;
		// Terminal status update
		{
			const auto now = chron::steady_clock::now();
			if (now > next_status) {
				// Insert code here that you want to run at the status frequency
				next_status += status_period;
				total_margin = 0; margin_cycles = 0;

				robotserver.print_status_update();
				// std::cout << cycle_count << " " << robotserver.get_time_prog() << std::endl;
			}

			int skip_count = 0;
			while (now > next_cycle) {skip_count++; next_cycle += period;}

			total_skip_count += skip_count;
			if (skip_count)
				std::cout << "Skipped " << total_skip_count << "/" << cycle_count 
					<<" cycles in total" << std::endl;

			if (skip_count > rs_settings.skip_count_max) {
				std::cout << "too many skipped cycles, exiting..." << std::endl;
				std::exit(EXIT_FAILURE);
			}
		}
		
		// Sleep current thread until next control interval, per the period setting.
		{
			const auto pre_sleep = chron::steady_clock::now();
			std::this_thread::sleep_until(next_cycle);
			const auto post_sleep = chron::steady_clock::now();
			chron::duration<double> elapsed = post_sleep - pre_sleep;
			total_margin += elapsed.count();
		}
		next_cycle += period;

		// **** MANIPULATE COMMANDS HERE FOR OPERATION ****
		// fsm will create commands stored in the leg member actuators
		// robotserver.print_status_update();
		robotserver.iterate_fsm();
		// retrieve the commands (copy)
		for (size_t a_idx = 0; a_idx < curr_commands.size(); a_idx++) {
			curr_commands[a_idx] = robotserver.get_actuator_cmd(a_idx);
		}

		if (can_result.valid()) {
			// Now we get the result of our last query and send off our new one.
			const auto current_values = can_result.get();
			// We copy out the results we just got out.
			const auto rx_count = current_values.query_result_size;
			for (size_t ii = 0; ii < replies.size(); ii++)	{
				saved_replies[ii] = replies[ii];
			}
		}
		if(replies.size() < 2) std::cout << "main: incorrect number of replies: " << replies.size() << std::endl;

		// copy the replies over to the member actuators; they look for ID match. If
		// there's no matching ID response, fault is raised
		robotserver.retrieve_replies(saved_replies);
		// send out robot data back onto LCM network
		robotserver.publish_LCM_response();

		// Then we can immediately ask them to be used again.
		auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
		// Cycle out curr_commands to drivers
		moteus_interface.Cycle(
				moteus_data,
				[promise](const MoteusInterface::Output& output) {
					// This is called from an arbitrary thread, so we just set
					// the promise value here.
					promise->set_value(output);
				});
		can_result = promise->get_future();

		if (cycle_count > 5 && saved_replies.size() >= robotserver.num_actuators()) reply_miss_count = 0;
		else if (cycle_count > 5 && saved_replies.size() < robotserver.num_actuators()) reply_miss_count++;

		// kill loop if we miss all these replies
		if (reply_miss_count > 20) {
			std::cout << "missed too many replies in a row! ending..." << std::endl;
			break;
		}

		if (cycle_count > 1) {
			// std::copy(curr_commands.begin(), curr_commands.end(), prev_commands.begin());
			for (size_t ii = 0; ii < curr_commands.size(); ii++)	{
				prev_commands[ii] = curr_commands[ii];
			}
		}
		// robotserver->safety_check(saved_replies);
	}
	// IF INTERRUPTED
	std::cout << std::endl << "exiting..." << std::endl;

	std::exit(EXIT_SUCCESS);
}


int main(int argc, char** argv) {
	auto options = rs_opts();
	auto opts = options.parse(argc, argv);
	std::cout << "user comment: " << opts["comment"].as<std::string>() << std::endl;

	chron::time_point<chron::system_clock> now = chron::system_clock::now();
	std::time_t nowc = chron::system_clock::to_time_t(now);
	std::ostringstream filename_stream;
	filename_stream << std::put_time(std::localtime(&nowc), "robot_server_%d_%m_%Y_%H-%M-%S.csv");
	std::string filename = filename_stream.str();
	Color::Modifier color_factory(Color::Code::FG_DEFAULT);

	std::cout << "outputting to file \"" << color_factory.fg_blk() << color_factory.bg_wht() <<
		filename << "\"" << color_factory.fg_def() << color_factory.bg_def() << std::endl;

	// std::ofstream data_file("/home/pi/embir-modular-leg/robotserver-data/"+filename);
	std::ofstream data_file(opts["path"].as<std::string>()+filename);

	std::vector<std::string> arg_list(argv, argv+argc);
	data_file << "# ";
	for (auto arg_str : arg_list) data_file << arg_str << " ";
	data_file << std::endl;
	// return 0;

	// Adafruit_ADS1015 ads;
	// Adafruit_INA260 ina1;
	// Adafruit_INA260 ina2;
	// if (!bcm2835_init()) {
	// 	std::cout << "bcm2835_init failed. Are you running as root??\n" << std::endl;
	// 	return 1;
	// }
	// bcm2835_i2c_begin();

	LockMemory();

	RobotServer::RobotServerSettings rs_settings(opts);
	if (!rs_settings.load_success) {
		std::exit(EXIT_FAILURE);
	}
		
	rs_settings.status_period_us = static_cast<int64_t>(
		(1e6)/10);
	data_file << "# \n# user comment: " << opts["comment"]
		.as<std::string>() << "\n# \n";
	data_file << "# period s: " << 1.0/opts["frequency"].as<float>() << "\n";
	data_file << "# duration s: " << opts["duration"].as<float>() << "\n";

	ConfigureRealtime(rs_settings.main_cpu);
	ConfigureRealtime(rs_settings.can_cpu);

	RobotServer robotserver(
		rs_settings,
		data_file);
	// return 0;
	Run(robotserver);
	std::cout << "(returned to main)" << std::endl;
	data_file.close();
	std::exit(EXIT_SUCCESS);
}