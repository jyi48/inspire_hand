#pragma once

#include <rclcpp/rclcpp.hpp>
#include <inspire_hand_msgs/msg/inspire_hand_ctrl.hpp>
#include <inspire_hand_msgs/msg/inspire_hand_state.hpp>
#include "inspire_hand_driver/modbus_client.hpp"

#include <mutex>
#include <memory>

namespace inspire_hand_driver
{

// Modbus 레지스터 주소 (inspire hand 스펙)
namespace reg
{
  constexpr int POS_SET   = 1474;
  constexpr int ANGLE_SET = 1486;
  constexpr int FORCE_SET = 1498;
  constexpr int SPEED_SET = 1522;

  constexpr int POS_ACT   = 1534;
  constexpr int ANGLE_ACT = 1546;
  constexpr int FORCE_ACT = 1582;
  constexpr int CURRENT   = 1594;
  // err/status/temp: byte 레지스터
  constexpr int ERR       = 1606;  // 3 레지스터 → 6 bytes
  constexpr int STATUS    = 1612;
  constexpr int TEMP      = 1618;

  constexpr int RESET_ERR = 1004;
}

struct HandConfig {
  std::string ip;
  std::string side;   // "l" or "r"
  int port;
  int device_id;
};

class InspireHandDriver : public rclcpp::Node
{
public:
  explicit InspireHandDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~InspireHandDriver() override;

private:
  // 각 손별 상태
  struct Hand {
    HandConfig config;
    std::unique_ptr<ModbusClient> modbus;
    std::mutex write_mutex;

    rclcpp::Time last_ctrl_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<inspire_hand_msgs::msg::InspireHandCtrl>::SharedPtr sub;
    rclcpp::Publisher<inspire_hand_msgs::msg::InspireHandState>::SharedPtr state_pub;
  };

  Hand left_;
  Hand right_;

  double ctrl_min_interval_s_{0.02};  // 1/ctrl_hz

  rclcpp::TimerBase::SharedPtr state_timer_;

  void init_hand(Hand & hand);

  void ctrl_callback(
    Hand & hand,
    const inspire_hand_msgs::msg::InspireHandCtrl::SharedPtr msg);

  void read_and_publish_state(Hand & hand);
};

}  // namespace inspire_hand_driver
