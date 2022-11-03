#include <robotnik_eipscanner/EIPScanner.h>
// #include "my_cpp_library/my_cpp_library.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eipscanner_node");
  ros::NodeHandle n;

  robotnik_eipscanner::EIPScanner publisher(n);
  publisher.start();
}
