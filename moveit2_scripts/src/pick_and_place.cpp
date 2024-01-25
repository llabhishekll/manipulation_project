#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);

  // shutdown
  rclcpp::shutdown();
  return 0;
}