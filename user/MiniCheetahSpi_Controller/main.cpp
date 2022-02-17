
#include <main_helper.h>
#include "MiniCheetahSpi_Controller.h"

int main(int argc, char** argv) {
  std::cout<<"argc "<<argc<<std::endl;
  std::cout<<"argv "<<argv<<std::endl;
  main_helper(argc, argv, new MiniCheetahSpi_Controller());
  return 0;
}