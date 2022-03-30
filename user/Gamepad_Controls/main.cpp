/*!
 * @file main.cpp
 * @brief Main Function for the Torso Position Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

// #include <QtCore/QCoreApplication>
#include <QApplication>
// #include "sim/include/GameController.h"
#include "../../sim/include/GameController.h"

int main(int argc, char **argv)
{
    QApplication application(argc, argv);

    GameController gamepad_controls(0,0);


    // return application.exec();
    application.exec();

    return 0;
}