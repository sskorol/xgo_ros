#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <xgo2_ros/xgo.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto xgo = std::make_shared<XGO>();
  std::cout << "Initialization done." << std::endl;

  rclcpp::Rate loop_rate(500);

  while (rclcpp::ok()) {
      xgo->readState();
      rclcpp::spin_some(xgo);
      loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
