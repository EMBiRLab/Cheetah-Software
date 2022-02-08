#include <sys/mman.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <random>
#include <algorithm>

#include "moteus_controller.h"

#ifndef PI
#define PI 3.1415926
#endif

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

MoteusController::MoteusController(id_t id, uint8_t bus) : 
	id_(id), bus_(bus) {
	
	// set up resolutions
  res.position = moteus::Resolution::kFloat;
  res.velocity = moteus::Resolution::kFloat;
  res.feedforward_torque = moteus::Resolution::kFloat;
  res.kp_scale = moteus::Resolution::kInt16;
  res.kd_scale = moteus::Resolution::kInt16;
  res.maximum_torque = moteus::Resolution::kIgnore;
  res.stop_position = moteus::Resolution::kIgnore;
  res.watchdog_timeout = moteus::Resolution::kIgnore;

	curr_cmd_.resolution = res;
	curr_cmd_.query.velocity = mjbots::moteus::Resolution::kFloat;
	curr_cmd_.query.position = mjbots::moteus::Resolution::kFloat;
	curr_cmd_.query.torque = mjbots::moteus::Resolution::kFloat;

	prev_cmd_.resolution = res;
	prev_cmd_.query.velocity = mjbots::moteus::Resolution::kFloat;
	prev_cmd_.query.position = mjbots::moteus::Resolution::kFloat;
	prev_cmd_.query.torque = mjbots::moteus::Resolution::kFloat;

	retrieve_v_per_hz();
}

void MoteusController::restore_cal(std::string path) {
	std::string id_str = std::to_string(id_);
	std::string bus_str = std::to_string(bus_);
	std::string command = "python3 -m moteus.moteus_tool --target " +
		id_str + " --pi3hat-cfg '" + bus_str + "=" + id_str +
		"' --restore-cal " + path;
	std::cout << "restoring calibration to id " << id_str << " from " <<
		path << "...\n" << command;
	system(command.c_str());
	std::cout << "    done. " << std::endl;

	retrieve_v_per_hz();
}

void MoteusController::restore_cfg(std::string path) {
	std::string id_str = std::to_string(id_);
	std::string bus_str = std::to_string(bus_);
	std::string command = "python3 -m moteus.moteus_tool --target " +
		id_str + " --pi3hat-cfg '" + bus_str + "=" + id_str +
		"' --write-config " + path;
	std::cout << "restoring config to id " << id_str << " from " <<
		path << "...\n" << command;
	system(command.c_str());
	std::cout << "    done. " << std::endl;
}

void MoteusController::zero_offset() {
	std::string id_str = std::to_string(id_);
	std::string bus_str = std::to_string(bus_);
	std::string command = "python3 -m moteus.moteus_tool --target " +
		id_str + " --pi3hat-cfg '" + bus_str + "=" + id_str +
		"' --zero-offset";
	std::cout << "setting current position as zero...\n" << command;
	system(command.c_str());
	std::cout << "    done. " << std::endl;
}

void MoteusController::make_stop() {
	curr_cmd_.id = id_;
	curr_cmd_.mode = moteus::Mode::kStopped;
	curr_cmd_.position.feedforward_torque = 0;
	curr_cmd_.position.kp_scale = 0;
	curr_cmd_.position.kd_scale = 0;
}

void MoteusController::make_mot_position(float pos_rot, float kps,
	float kds, float ff_trq_Nm) {
	if (fault() != errc::kSuccess) {make_stop(); return;}
	curr_cmd_.id = id_;
	curr_cmd_.mode = moteus::Mode::kPosition;
	curr_cmd_.position.kp_scale = kps;
	curr_cmd_.position.kd_scale = kds;
	curr_cmd_.position.position = pos_rot;
	curr_cmd_.position.velocity = 0;
	curr_cmd_.position.feedforward_torque = ff_trq_Nm;
}

void MoteusController::make_mot_velocity(float vel_Hz, float kps,
	float kds, float ff_trq_Nm) {
	if (fault() != errc::kSuccess) {make_stop(); return;}
	curr_cmd_.id = id_;
	curr_cmd_.mode = moteus::Mode::kPosition;
	curr_cmd_.position.kp_scale = kps;
	curr_cmd_.position.kd_scale = kds;
	curr_cmd_.position.position = std::numeric_limits<double>::quiet_NaN();
	curr_cmd_.position.velocity = vel_Hz;
	curr_cmd_.position.feedforward_torque = ff_trq_Nm;
}

void MoteusController::make_mot_torque(float trq_Nm) {
	if (fault() != errc::kSuccess) {make_stop(); return;}
	curr_cmd_.id = id_;
	curr_cmd_.mode = moteus::Mode::kPosition;
	curr_cmd_.position.kp_scale = 0;
	curr_cmd_.position.kd_scale = 0;
	curr_cmd_.position.position = std::numeric_limits<double>::quiet_NaN();
	curr_cmd_.position.velocity = 0;
	curr_cmd_.position.feedforward_torque = trq_Nm;
}

void MoteusController::make_mot_full_pos(float pos_rot, float vel_Hz,
	float trq_Nm, float kps, float kds) {
	if (fault() != errc::kSuccess) {make_stop(); return;}
	curr_cmd_.id = id_;
	curr_cmd_.mode = moteus::Mode::kPosition;
	curr_cmd_.position.kp_scale = kps;
	curr_cmd_.position.kd_scale = kds;
	curr_cmd_.position.position = pos_rot;
	curr_cmd_.position.velocity = vel_Hz;
	curr_cmd_.position.feedforward_torque = trq_Nm;
}


void MoteusController::retrieve_reply(std::vector<MoteusInterface::ServoReply>& replies) {
	// if(replies.size() < 2) std::cout << "incorrect number of replies: " << replies.size() << std::endl;
	for (auto reply : replies) {
		if (reply.id == id_) {
			prev_reply_ = reply;
			mode_ = prev_reply_.result.mode;
			fault_code_ = (errc)prev_reply_.result.fault;
			if (fault_code_ == errc::kSuccess && outside_limit()) fault_code_ = errc::kOutsideLimit;
			return;
		}
	}
	prev_reply_ = {};
	mode_ = mjbots::moteus::Mode::kFault;
	fault_code_ = errc::kMissingReply;
	return;
}

std::string MoteusController::stringify_moteus_reply() {
  auto& data = prev_reply_.result;
  std::ostringstream result;
  sprintf(cstr_buffer, "%2d,% -f,% -f,",
    data.mode, data.position, data.velocity);
  result << cstr_buffer;
  sprintf(cstr_buffer, "% -f,% -f,%3d",
    data.torque, data.temperature, (int)fault_code_);
  result << cstr_buffer;
  return result.str();
}

std::string MoteusController::stringify_moteus_reply_header() {
  std::ostringstream result;
  std::string cN = "c"+std::to_string(id_)+" ";
  result << cN << "mode," << cN << "position [rev]," << cN << "velocity [Hz],"
    << cN << "torque [Nm]," << cN << "temp [C]," << cN << "fault";
  return result.str();
}

void MoteusController::retrieve_v_per_hz() {
	// this is a super ugly way of doing this but we only need to do it once...
	// better way would be to send a CAN frame requesting the data thru pi3hat
	std::string id_str = std::to_string(id_);
	std::string bus_str = std::to_string(bus_);
	std::string command = "python3 -m moteus.moteus_tool --target " +
		id_str + " --pi3hat-cfg '" + bus_str + "=" + id_str +
		"' --dump-config | grep motor.v_per_hz | cut -c 16-24 > moteus_kt.temp";
	int exit_code = system(command.c_str());
	std::ifstream("moteus_kt.temp") >> v_per_hz;
	torque_constant_Nm_A = 0.78*v_per_hz/PI;
	std::cout << "actuator id " << id_str << " torque constant = " 
		<< torque_constant_Nm_A << " Nm/A" << std::endl;
}