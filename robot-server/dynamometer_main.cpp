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

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

#include "dynamometer.h"
#include "utils.h"
// #include "color.h"

#include "sensors/Adafruit_ADS1X15.h"
#include "sensors/Adafruit_INA260.h"

#include "libFilter/filters.h"
#include "iir/iir.h"
#include "nlohmann/json.hpp"
#include "cxxopts/cxxopts.hpp"

#define PI 3.1415926

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;
using json = nlohmann::json;
namespace chron = std::chrono;

char cstr_buffer[128];

volatile sig_atomic_t interrupted=false; 

void sig_handle(int s) {
  interrupted = true;
}

void Run(Dynamometer& dynamometer) {
  if (dynamometer.is_ready_to_quit()) return;
  // * SETUP *
  // ** CTRL+C CATCH **
	Dynamometer::DynamometerSettings& dynset = dynamometer.get_dynset();
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sig_handle;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ** CONFIGURE CPU AND MOTEUS INFRASTRUCTURE **
  std::cout << "configuring realtime and constructing moteus interface... ";
  moteus::ConfigureRealtime(dynset.main_cpu);
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = dynset.can_cpu;
  moteus_options.servo_bus_map = {
    { dynset.a1_id, dynset.a1_bus },
    { dynset.a2_id, dynset.a2_bus },
  };
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
  // distributes pointers to this data into the MoteusController objects and
  // initializes the commands (resolution settings etc.)
  // dynamometer.share_commands(curr_commands, prev_commands);

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
      chron::microseconds(static_cast<int64_t>(dynset.period_s * 1e6));
  auto next_cycle = chron::steady_clock::now() + period;

  const auto status_period = chron::microseconds(dynset.status_period_us);
  auto next_status = next_cycle + status_period;

  const auto grp_sampling_period = std::chrono::microseconds(dynset.grp_sampling_period_us);
  auto next_grp = next_cycle + grp_sampling_period;

  uint64_t cycle_count = 0;
  uint64_t total_skip_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;
  uint64_t reply_miss_count = 0;

  std::string c1_str, c2_str, a1_str, a2_str, sensor_str;

  auto t0 = chron::steady_clock::now();
  dynamometer.set_time0(t0);

  dynamometer.log_headers();

  // * MAIN LOOP *
  std::cout << "beginning while loop..." << std::endl;
  while (!interrupted
      && dynamometer.get_time_prog() < dynset.duration_s
      && !dynamometer.is_ready_to_quit()) {
    cycle_count++; margin_cycles++;
    // Terminal status update
    {
      const auto now = chron::steady_clock::now();
      if (now > next_status) {
        // Insert code here that you want to run at the status frequency
        next_status += status_period;
        total_margin = 0; margin_cycles = 0;

        dynamometer.print_status_update();
        // std::cout << cycle_count << " " << dynamometer.get_time_prog() << std::endl;
      }

      int skip_count = 0;
      while (now > next_cycle) {skip_count++; next_cycle += period;}

      total_skip_count += skip_count;
      if (skip_count)
        std::cout << "Skipped " << total_skip_count << "/" << cycle_count 
          <<" cycles in total" << std::endl;

      if (skip_count > 50) {
        std::cout << "too many skipped cycles, exiting..." << std::endl;
        std::exit(EXIT_FAILURE);
      }
    }

    {
      const auto now = std::chrono::steady_clock::now();
      if (now > next_grp) {
        dynamometer.sample_random();
        next_grp += grp_sampling_period;
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
    // dynamometer.print_status_update();
    dynamometer.iterate_fsm();
    // retrieve the commands (copy)
    curr_commands[0] = dynamometer.get_a1_cmd();
    curr_commands[1] = dynamometer.get_a2_cmd();

    if (can_result.valid()) {
      // Now we get the result of our last query and send off our new one.
      const auto current_values = can_result.get();
      // We copy out the results we just got out.
      const auto rx_count = current_values.query_result_size;
      // saved_replies.resize(rx_count);
  	  // std::cout << "cycle_count: " << cycle_count
      //   << " replies.size(): " << replies.size() 
      //   << " saved_replies.size(): " << saved_replies.size() 
      //   << " rx_count: " << rx_count << std::endl;
      // std::copy(replies.begin(), replies.begin() + rx_count, 
      //   saved_replies.begin());
      for (size_t ii = 0; ii < replies.size(); ii++)  {
        saved_replies[ii] = replies[ii];
      }
    }
  	if(replies.size() < 2) std::cout << "main: incorrect number of replies: " << replies.size() << std::endl;

    // copy the replies over to the member actuators; they look for ID match. If
    // there's no matching ID response, fault is raised
    dynamometer.retrieve_replies(saved_replies);

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

    if (cycle_count > 5 && saved_replies.size() >= 2) reply_miss_count = 0;
    else if (cycle_count > 5 && saved_replies.size() < 2) reply_miss_count++;

    // kill loop if we miss all these replies
    if (reply_miss_count > 20) {
      std::cout << "missed too many replies in a row! ending..." << std::endl;
      break;
    }

    if (cycle_count > 1) {
      // std::copy(curr_commands.begin(), curr_commands.end(), prev_commands.begin());
      for (size_t ii = 0; ii < curr_commands.size(); ii++)  {
        prev_commands[ii] = curr_commands[ii];
      }
    }
    // dynamometer->safety_check(saved_replies);
  }
  // IF INTERRUPTED
  std::cout << std::endl << "exiting..." << std::endl;
  // std::cout << dynamometer.get_time_prog() << "\n" << dynset.duration_s;

  std::exit(EXIT_SUCCESS);
}


int main(int argc, char** argv) {
  auto options = dyn_opts();
  auto opts = options.parse(argc, argv);
  std::cout << "user comment: " << opts["comment"].as<std::string>() << std::endl;

  chron::time_point<chron::system_clock> now = chron::system_clock::now();
  std::time_t nowc = chron::system_clock::to_time_t(now);
  std::ostringstream filename_stream;
  filename_stream << std::put_time(std::localtime(&nowc), "dynamometer_test_%d_%m_%Y_%H-%M-%S.csv");
  std::string filename = filename_stream.str();
  Color::Modifier color_factory(Color::Code::FG_DEFAULT);

  std::cout << "outputting to file \"" << color_factory.fg_blk() << color_factory.bg_wht() <<
    filename << "\"" << color_factory.fg_def() << color_factory.bg_def() << std::endl;

  // std::ofstream data_file("/home/pi/embir-modular-leg/dynamometer-data/"+filename);
  std::ofstream data_file(opts["path"].as<std::string>()+filename);

  std::vector<std::string> arg_list(argv, argv+argc);
  data_file << "# ";
  for (auto arg_str : arg_list) data_file << arg_str << " ";
  data_file << std::endl;
  // return 0;

  Adafruit_ADS1015 ads;
  Adafruit_INA260 ina1;
  Adafruit_INA260 ina2;
  if (!bcm2835_init()) {
    std::cout << "bcm2835_init failed. Are you running as root??\n" << std::endl;
    return 1;
  }
  bcm2835_i2c_begin();

  LockMemory();

  Dynamometer::DynamometerSettings dynset(opts);
  
  dynset.status_period_us = static_cast<int64_t>(
    (1e6)/10);
  data_file << "# \n# user comment: " << opts["comment"]
    .as<std::string>() << "\n# \n";
  if (dynset.test_mode != Dynamometer::TestMode::kGRP) {
    data_file << "# period s: " << 1.0/opts["frequency"].as<float>() << "\n";
    data_file << "# duration s: " << opts["duration"].as<float>() << "\n";
    data_file << "# gear a1: " << opts["gear-a1"].as<float>() << "\n";
    data_file << "# gear a2: " << opts["gear-a2"].as<float>() << "\n";
    data_file << "# a1 actuator id: " << (int)(opts["a1-id"]
      .as<uint8_t>()) << "\n";
    data_file << "# a2 actuator id: " << (int)(opts["a2-id"]
      .as<uint8_t>()) << "\n";
    data_file << "# a1 actuator bus: " << (int)(opts["a1-bus"]
      .as<uint8_t>()) << "\n";
    data_file << "# a2 actuator bus: " << (int)(opts["a2-bus"]
      .as<uint8_t>()) << "\n";
    data_file << "# main cpu: " << (int)(opts["main-cpu"].as<uint8_t>()) << "\n";
    data_file << "# can cpu: " << (int)(opts["can-cpu"]
      .as<uint8_t>()) << "\n# \n";
  }

  ConfigureRealtime(dynset.main_cpu);
  ConfigureRealtime(dynset.can_cpu);

	Dynamometer dynamometer(
    dynset,
    data_file);
  data_file << "# a1 kt Nm/A: " << dynamometer.get_a1_kt() << "\n";
  data_file << "# a2 kt Nm/A: " << dynamometer.get_a2_kt() << "\n";
  // return 0;
  Run(dynamometer);
  std::cout << "(returned to main)" << std::endl;
  data_file.close();
  std::exit(EXIT_SUCCESS);
}