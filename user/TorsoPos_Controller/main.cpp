/*!
 * @file main.cpp
 * @brief Main Function for the Torso Position Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "TorsoPos_Controller.hpp"

int main(int argc, char** argv) {
  main_helper(argc, argv, new TorsoPos_Controller());
  return 0;
}
