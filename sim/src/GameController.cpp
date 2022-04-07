/*! @file GameController.cpp
 *  @brief Code to read the Logitech F310 Game Controller
 *  Creates a DriverCommand object to be sent to the robot controller
 *  Used in the development simulator and in the robot control mode
 *
 *  NOTE: Because QT is weird, the updateDriverCommand has to be called from a
 * QT event. Running it in another thread will cause it to not work. As a
 * result, this only works if called in the update method of a QTObject
 */

#include "GameController.h"

#include <QtCore/QObject>
#include <QtGamepad/QGamepad>
#include <lcm/lcm-cpp.hpp>
#include "Utilities/utilities.h"
#include <iostream>
// #include <unistd.h>

/*!
 * By default, the game controller selects the "first" joystick, printing a
 * warning if there are multiple joysticks On Linux, this is /dev/input/js0 If
 * no joystick is found, it will print an error message, and will return zero.
 * It is possible to change/add a joystick later with findNewController
 */
GameController::GameController(QObject *parent, int sim) : QObject(parent), _lcm(getLcmUrl(255)) {
// GameController::GameController(QObject *parent) : QObject(parent) {
  findNewController();
  std::cout << "<sim> value is " << sim << "***********************************" << std::endl;
  if (!sim){
  // if (1){
    // We are not calling this object from the simulator, so we want to periodically
    // send LCMs for the hardwarebridge to see
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateGamepadCommandLCM()));
    // connect(timer, SIGNAL(timeout()), this, &GameController::updateGamepadCommandLCM());
    timer->start(2);
    std::cout << "<sim> value is " << sim << "***********************************" << std::endl;
  }
}

/*!
 * Re-run the joystick finding code to select the "first" joystick. This can be
 * used to set up the joystick if the simulator is started without a joystick
 * plugged in
 */
void GameController::findNewController() {
  delete _qGamepad;
  _qGamepad = nullptr;  // in case this doesn't work!

  printf("[Gamepad] Searching for gamepads, please ignore \"Device discovery cannot open device\" errors\n");
  auto gamepadList = QGamepadManager::instance()->connectedGamepads();
  printf("[Gamepad] Done searching for gamepads.\n");
  if (gamepadList.empty()) {
    printf(
        "[ERROR: GameController] No controller was connected! All joystick "
        "commands will be zero!\n");
  } else {
    if (gamepadList.size() > 1) {
      printf(
          "[ERROR: GameController] There are %d joysticks connected.  Using "
          "the first one.\n",
          gamepadList.size());
    } else {
      printf("[GameController] Found 1 joystick\n");
    }
    _qGamepad = new QGamepad(*gamepadList.begin());
  }
}

/*!
 * Overwrite a driverCommand with the current joystick state.  If there's no
 * joystick, sends zeros
 * TODO: what happens if the joystick is unplugged?
 */
void GameController::updateGamepadCommand(GamepadCommand &gamepadCommand) {
  if (_qGamepad) {
    gamepadCommand.leftBumper = _qGamepad->buttonL1();
    gamepadCommand.rightBumper = _qGamepad->buttonR1();
    gamepadCommand.leftTriggerButton = _qGamepad->buttonL2() != 0.;
    gamepadCommand.rightTriggerButton = _qGamepad->buttonR2() != 0.;
    gamepadCommand.back = _qGamepad->buttonSelect();
    gamepadCommand.start = _qGamepad->buttonStart();
    gamepadCommand.a = _qGamepad->buttonA();
    gamepadCommand.b = _qGamepad->buttonB();
    gamepadCommand.x = _qGamepad->buttonX();
    gamepadCommand.y = _qGamepad->buttonY();
    gamepadCommand.leftStickButton = _qGamepad->buttonL3();
    gamepadCommand.rightStickButton = _qGamepad->buttonR3();
    gamepadCommand.leftTriggerAnalog = (float)_qGamepad->buttonL2();
    gamepadCommand.rightTriggerAnalog = (float)_qGamepad->buttonR2();
    gamepadCommand.leftStickAnalog =
        Vec2<float>(_qGamepad->axisLeftX(), -_qGamepad->axisLeftY());
    gamepadCommand.rightStickAnalog =
        Vec2<float>(_qGamepad->axisRightX(), -_qGamepad->axisRightY());
  } else {
    // gamepadCommand.zero();  // no joystick, return all zeros
    printf("setting left analog and right analog stick values\n");
    gamepadCommand.leftBumper = 1;
    gamepadCommand.rightBumper = 1;
    gamepadCommand.leftTriggerButton = 1;
    gamepadCommand.rightTriggerButton = 1;
    gamepadCommand.back = 1;
    gamepadCommand.start = 1;
    gamepadCommand.a = 1;
    gamepadCommand.b = 1;
    gamepadCommand.x = 1;
    gamepadCommand.y = 1;
    gamepadCommand.leftStickButton = 1;
    gamepadCommand.rightStickButton = 1;
    gamepadCommand.leftTriggerAnalog = 1.0;
    gamepadCommand.rightTriggerAnalog = 1.0;
    gamepadCommand.leftStickAnalog =
        Vec2<float>(0.0, 0.5);
    gamepadCommand.rightStickAnalog =
        Vec2<float>(0.0, 0.0);
  }

  // printf("%s\n", gamepadCommand.toString().c_str());
}

void GameController::updateGamepadCommandLCM(){
  std::cout << "*new gamepad cmd*" << std::endl;
  if (_qGamepad) {
    gamepadCmd.leftBumper = _qGamepad->buttonL1();
    gamepadCmd.rightBumper = _qGamepad->buttonR1();
    gamepadCmd.leftTriggerButton = _qGamepad->buttonL2() != 0.;
    gamepadCmd.rightTriggerButton = _qGamepad->buttonR2() != 0.;
    gamepadCmd.back = _qGamepad->buttonSelect();
    gamepadCmd.start = _qGamepad->buttonStart();
    gamepadCmd.a = _qGamepad->buttonA();
    gamepadCmd.b = _qGamepad->buttonB();
    gamepadCmd.x = _qGamepad->buttonX();
    gamepadCmd.y = _qGamepad->buttonY();
    gamepadCmd.leftStickButton = _qGamepad->buttonL3();
    gamepadCmd.rightStickButton = _qGamepad->buttonR3();
    gamepadCmd.leftTriggerAnalog = (float)_qGamepad->buttonL2();
    gamepadCmd.rightTriggerAnalog = (float)_qGamepad->buttonR2();
    gamepadCmd.leftStickAnalog =
        Vec2<float>(_qGamepad->axisLeftX(), -_qGamepad->axisLeftY());
    gamepadCmd.rightStickAnalog =
        Vec2<float>(_qGamepad->axisRightX(), -_qGamepad->axisRightY());
  } else {
    // gamepadCmd.zero();  // no joystick, return all zeros
    printf("setting left analog and right analog stick values\n");
    gamepadCmd.leftBumper = 1;
    gamepadCmd.rightBumper = 1;
    gamepadCmd.leftTriggerButton = 1;
    gamepadCmd.rightTriggerButton = 1;
    gamepadCmd.back = 1;
    gamepadCmd.start = 1;
    gamepadCmd.a = 1;
    gamepadCmd.b = 1;
    gamepadCmd.x = 1;
    gamepadCmd.y = 1;
    gamepadCmd.leftStickButton = 1;
    gamepadCmd.rightStickButton = 1;
    gamepadCmd.leftTriggerAnalog = 1.0;
    gamepadCmd.rightTriggerAnalog = 1.0;
    gamepadCmd.leftStickAnalog =
        Vec2<float>(0.0, 0.5);
    gamepadCmd.rightStickAnalog =
        Vec2<float>(0.0, 0.0);
  }
  gamepadCmd.get(&_gamepad_lcmt);
  _lcm.publish(INTERFACE_LCM_NAME, &_gamepad_lcmt);
}

GameController::~GameController() { 
  delete _qGamepad;
  delete timer;
}