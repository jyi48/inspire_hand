#include <rclcpp/rclcpp.hpp>
#include "inspire_hand_driver/inspire_hand_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<inspire_hand_driver::InspireHandDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
