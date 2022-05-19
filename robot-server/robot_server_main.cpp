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

// bool to store inerrupt (CTRL+C) signal
volatile sig_atomic_t interrupted=false; 


// handler to catch interrupt signal and set bool -- allows us to handle in
// our own way
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
	// rs_settings holds configuration options loaded from command line args
	// and json config file
	RobotServer::RobotServerSettings& rs_settings = robotserver.get_rs_set();
	
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
	pi3hat::Attitude attitude;

	// ** PACKAGE COMMANDS AND REPLIES IN moteus_data **
	// moteus_data has access to commands and replies via pointer
	std::cout << "setting up moteus_data container... ";
	MoteusInterface::Data moteus_data;
	moteus_data.commands = { curr_commands.data(), curr_commands.size() };
	moteus_data.replies = { replies.data(), replies.size() };
	moteus_data.attitude = &attitude;
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
	while (robotserver.get_time_prog() < rs_settings.duration_s
			&& !robotserver.is_ready_to_quit()) {
		if (interrupted) {
			// gives RobotServer a chance to send out stop commands to all actuators before closing the program
			std::cout << "\ncaught interruption... transitioning to quit";
			robotserver.transition_to_quit();
		}
		cycle_count++; margin_cycles++;

		{ // Terminal status update
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
		} // end Terminal status update
		
		{ // Sleep current thread until next control interval, per the period setting.
			const auto pre_sleep = chron::steady_clock::now();
			std::this_thread::sleep_until(next_cycle);
			const auto post_sleep = chron::steady_clock::now();
			chron::duration<double> elapsed = post_sleep - pre_sleep;
			total_margin += elapsed.count();
		} // end Sleep
		next_cycle += period;

		// **** MANIPULATE COMMANDS HERE FOR OPERATION ****
		// fsm will create commands stored in the RobotServer member actuators
		// robotserver.print_status_update();
		robotserver.iterate_fsm();
		// retrieve the commands (copy)
		for (size_t a_idx = 0; a_idx < curr_commands.size(); a_idx++) {
			curr_commands[a_idx] = robotserver.get_actuator_cmd(a_idx);
		}

		// object can_result is a stl future type -- it allows the program to know
		// that, at some point, it will hold valid data. can_result.valid() checks
		// for that validity
		if (can_result.valid()) {
			// Now we get the result of our last query and send off our new one.
			const auto current_values = can_result.get();
			// We copy out the results we just got out.
			const auto rx_count = current_values.query_result_size;
			for (size_t ii = 0; ii < replies.size(); ii++)	{
				saved_replies[ii] = replies[ii];
			}
			robotserver.set_pi3hat_attitude(moteus_data.attitude);
			if (current_values.attitude_present) { // copy out new IMU attitude data if it exists
				robotserver.set_pi3hat_attitude(moteus_data.attitude);
			}
		}

		// copy the replies over to the member actuators; they look for ID match. If
		// there's no matching ID response, we missed a reply from that actuator
		// previously, this was treated as a fault triggering a transition to the
		// recovery state. However, this created more problems where the moteus
		// controllers would oscillate between running and stop modes, which 
		// could cause more faults. 
		// It was found to more stable to allow occasional missed replies
		robotserver.retrieve_replies(saved_replies);
		// send out robot data back onto LCM network
		robotserver.publish_LCM_response();

		// Then we can immediately ask them to be used again.
		auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
		// Cycle out curr_commands to drivers
		// this is an asynchronous operation -- functions below .Cycle() mostly
		// run in other threads
		//
		// It may be useful to think of this as the start of the loop and the
		// extraction of replies above as the end of the loop
		moteus_interface.Cycle(
				moteus_data,
				[promise](const MoteusInterface::Output& output) {
					// This is called from an arbitrary thread, so we just set
					// the promise value here.
					promise->set_value(output);
				});
		// Here, we link can_result with promise -- can_result will get valid data
		// once .Cycle() finishes all its work and can then be used in the next loop
		can_result = promise->get_future();

		if (cycle_count > 5 && saved_replies.size() >= robotserver.num_actuators()) reply_miss_count = 0;
		else if (cycle_count > 5 && saved_replies.size() < robotserver.num_actuators()) reply_miss_count++;

		// kill loop if we miss all these replies
		if (reply_miss_count > 20) {
			std::cout << " RMC:" << reply_miss_count << " ";
			// std::cout << "missed too many replies in a row! ending..." << std::endl;
			// break;
		}

		if (cycle_count > 1) {
			// normally would do this with std::copy but there was a mysterious bug
			// that went away when we switched to naive for-loop copy
			for (size_t ii = 0; ii < curr_commands.size(); ii++)	{
				prev_commands[ii] = curr_commands[ii];
			}
		}
		// robotserver->safety_check(saved_replies);
	}
	// IF INTERRUPTED
	robotserver.flush_data();
	std::cout << std::endl << "exiting..." << std::endl;

	std::exit(EXIT_SUCCESS);
}


int main(int argc, char** argv) {
	auto options = rs_opts();
	auto opts = options.parse(argc, argv);
	std::cout << "user comment: " << opts["comment"].as<std::string>() << std::endl;

	// generate output log file name
	chron::time_point<chron::system_clock> now = chron::system_clock::now();
	std::time_t nowc = chron::system_clock::to_time_t(now);
	std::ostringstream filename_stream;
	filename_stream << std::put_time(std::localtime(&nowc), "robot_server_%d_%m_%Y_%H-%M-%S.csv");
	std::string filename = filename_stream.str();
	Color::Modifier color_factory(Color::Code::FG_DEFAULT);

	std::cout << "outputting to file \"" << color_factory.fg_blk() << color_factory.bg_wht() <<
		filename << "\"" << color_factory.fg_def() << color_factory.bg_def() << std::endl;

	std::ofstream data_file(opts["path"].as<std::string>()+filename);

	// place command line args into log file for reproducibility
	std::vector<std::string> arg_list(argv, argv+argc);
	data_file << "# ";
	for (auto arg_str : arg_list) data_file << arg_str << " ";
	data_file << std::endl;

	LockMemory();
	// construct settings object from command line args and JSON config file
	RobotServer::RobotServerSettings rs_settings(opts);
	if (!rs_settings.load_success) {
		std::exit(EXIT_FAILURE);
	}
		
	rs_settings.status_period_us = static_cast<int64_t>(
		(1e6)/10);
	data_file << "# \n# user comment: " << opts["comment"]
		.as<std::string>() << "\n# \n";
	data_file << "# period s: " << rs_settings.period_s << "\n";
	data_file << "# duration s: " << rs_settings.duration_s << "\n";

	ConfigureRealtime(rs_settings.main_cpu);
	ConfigureRealtime(rs_settings.can_cpu);

	// construct server
	RobotServer robotserver(
		rs_settings,
		data_file);
	Run(robotserver);
	std::cout << "(returned to main -- should not normally happen)" << std::endl;
	data_file.close();
	std::exit(EXIT_SUCCESS);
}