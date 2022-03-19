#include <sstream>
#include "actuator.h"

#include <fmt/core.h>
#include <fmt/printf.h>

//using id_t = uint8_t;

Actuator::Actuator(id_t id, uint8_t bus, float gear_ratio, float trq_efficiency)
  : MoteusController(id, bus), gear_ratio_(gear_ratio),
	trq_efficiency_(trq_efficiency) {
	return;
}

std::string Actuator::stringify_actuator() {
  // std::ostringstream result;
  auto& cmd_data = curr_cmd_.position;
  auto& reply_data = prev_reply_.result;
  
  return fmt::sprintf("% -f,% -f,% -f,% -f,% -f,% -f,% -f,%3d",
    cmd_data.position*2*PI,
    cmd_data.velocity*2*PI,
    cmd_data.feedforward_torque,
    reply_data.position*2*PI,
    reply_data.velocity*2*PI,
    reply_data.torque,
    reply_data.temperature,
    (int)fault_code_);
  // result << cstr_buffer;
  // return result.str();
}

std::string Actuator::stringify_actuator_header() {
  // std::ostringstream result;
  // std::string aN = "a"+std::to_string(id_)+" ";
  int id_int = (int)id_;
  // result << aN << "position cmd [rad]," << aN << "velocity cmd [rad/s]," << aN << "ff torque cmd [Nm],"
  //   << aN << "position [rad]," << aN << "velocity [rad/s]," << aN << "torque [Nm],"
  //   << aN << "temp [C]," << aN << "fault";
  return fmt::sprintf("a%d position cmd [rad], a%d velocity cmd [rad/s], a%d ff torque cmd [Nm], a%d position [rad], a%d velocity [rad/s], a%d torque [Nm], a%d temp [C], a%d fault",
    id_int, id_int, id_int, id_int, id_int, id_int, id_int, id_int);
  // return result.str();
}
