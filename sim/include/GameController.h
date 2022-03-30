/*! @file GameController.h
 *  @brief Code to read the Logitech F310 Game Controller
 *  Creates a DriverCommand object to be sent to the robot controller
 *  Used in the development simulator and in the robot control mode
 */

#ifndef PROJECT_GAMECONTROLLER_H
#define PROJECT_GAMECONTROLLER_H

#include "SimUtilities/GamepadCommand.h"
#include "gamepad_lcmt.hpp"
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <lcm/lcm-cpp.hpp>

#define INTERFACE_LCM_NAME "interface"

class QGamepad;  // for an unknown reason, #including <QtGamepad/QGamepad> here
                 // makes compilation *very* slow
class QTimer;

class GameController : public QObject {
  Q_OBJECT
 public:
  explicit GameController(QObject *parent = 0, int sim = 1);
  // explicit GameController(QObject *parent = 0);
  void updateGamepadCommand(GamepadCommand &gamepadCommand);
  void findNewController();
  ~GameController();

 public slots:
  void updateGamepadCommandLCM();

 private:
  QGamepad *_qGamepad = nullptr;
  QTimer *timer = nullptr;
  gamepad_lcmt _gamepad_lcmt;
  GamepadCommand gamepadCmd;
  lcm::LCM _lcm;
};

#endif  // PROJECT_GAMECONTROLLER_H
