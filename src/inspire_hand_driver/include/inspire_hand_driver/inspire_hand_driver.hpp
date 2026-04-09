#pragma once

#include <rclcpp/rclcpp.hpp>
#include <inspire_hand_msgs/msg/inspire_hand_ctrl.hpp>
#include <inspire_hand_msgs/msg/inspire_hand_state.hpp>
#include "inspire_hand_driver/modbus_client.hpp"

#include <mutex>
#include <memory>
#include <optional>

namespace inspire_hand_driver
{

namespace reg
{
  // write
  constexpr int POS_SET   = 1474;
  constexpr int ANGLE_SET = 1486;
  constexpr int FORCE_SET = 1498;
  constexpr int SPEED_SET = 1522;

  // read — 2번 블록으로 묶기
  // block1 (short): 1534~1599, 66개
  constexpr int BLOCK1_START = 1534;
  constexpr int BLOCK1_LEN   = 66;
  constexpr int OFF_POS_ACT   = 0;   // 1534
  constexpr int OFF_ANGLE_ACT = 12;  // 1546
  constexpr int OFF_FORCE_ACT = 48;  // 1582
  constexpr int OFF_CURRENT   = 60;  // 1594

  // block2 (byte): 1606~1620, 15개 레지스터 → 30 bytes
  constexpr int BLOCK2_START  = 1606;
  constexpr int BLOCK2_LEN    = 15;
  constexpr int OFF_ERR        = 0;   // 1606 → bytes 0-5
  constexpr int OFF_STATUS     = 12;  // 1612 → bytes 12-17
  constexpr int OFF_TEMP       = 24;  // 1618 → bytes 24-29

  constexpr int RESET_ERR = 1004;
}

struct HandConfig {
  std::string ip;
  std::string side;  // "l" or "r"
  int port;
  int device_id;
};

class InspireHandDriver : public rclcpp::Node
{
public:
  explicit InspireHandDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~InspireHandDriver() override;

private:
  using CtrlMsg = inspire_hand_msgs::msg::InspireHandCtrl;
  using StateMsg = inspire_hand_msgs::msg::InspireHandState;

  struct Hand {
    HandConfig config;
    std::unique_ptr<ModbusClient> modbus;

    // ctrl 최신 명령 캐시 (callback → timer 전달)
    std::mutex cache_mutex;
    std::optional<CtrlMsg> cached_ctrl;

    rclcpp::Subscription<CtrlMsg>::SharedPtr sub;
    rclcpp::Publisher<StateMsg>::SharedPtr state_pub;
  };

  Hand left_;
  Hand right_;

  rclcpp::TimerBase::SharedPtr timer_;

  void init_hand(Hand & hand);
  void tick(Hand & hand);  // 타이머에서 호출: write + read + publish
  void do_write(Hand & hand, const CtrlMsg & msg);
  void do_read_publish(Hand & hand);
  bool ensure_connected(Hand & hand);
};

}  // namespace inspire_hand_driver
