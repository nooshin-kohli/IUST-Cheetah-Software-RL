
#include <main_helper.h>
#include "IUST_Spi_Controller.h"

int main(int argc, char** argv) {
  main_helper(argc, argv, new IUST_Spi_Controller());
  return 0;
}