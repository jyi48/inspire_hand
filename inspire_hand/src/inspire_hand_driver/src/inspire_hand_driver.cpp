#include "inspire_hand_driver/inspire_hand_driver.hpp"
#include <chrono>

using namespace std::chrono_literals;
using CtrlMsg  = inspire_hand_msgs::msg::InspireHandCtrl;
using StateMsg = inspire_hand_msgs::msg::InspireHandState;
// using TouchMsg = inspire_hand_msgs::msg::InspireHandTouch;  // touch 활성화 시 해제

namespace inspire_hand_driver
{

InspireHandDriver::InspireHandDriver(const rclcpp::NodeOptions & options)
: Node("inspire_hand_driver", options)
{
  this->declare_parameter("right_ip",    "192.168.123.210");
  this->declare_parameter("left_ip",     "192.168.123.211");
  this->declare_parameter("modbus_port", 6000);
  this->declare_parameter("device_id",   1);
  this->declare_parameter("timer_hz",    100.0);
  // this->declare_parameter("touch_hz", 20.0);  // touch 활성화 시 해제

  right_.config = {
    this->get_parameter("right_ip").as_string(), "r",
    static_cast<int>(this->get_parameter("modbus_port").as_int()),
    static_cast<int>(this->get_parameter("device_id").as_int())
  };
  left_.config = {
    this->get_parameter("left_ip").as_string(), "l",
    static_cast<int>(this->get_parameter("modbus_port").as_int()),
    static_cast<int>(this->get_parameter("device_id").as_int())
  };

  init_hand(right_);
  init_hand(left_);

  double hz = this->get_parameter("timer_hz").as_double();
  // double touch_hz = this->get_parameter("touch_hz").as_double();  // touch 활성화 시 해제
  // touch_interval_ = std::max(1, static_cast<int>(hz / touch_hz));

  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / hz));

  timer_ = this->create_wall_timer(period, [this]() {
    tick(right_);
    tick(left_);
  });

  RCLCPP_INFO(get_logger(),
    "InspireHandDriver started — right:%s left:%s %.0fHz",
    right_.config.ip.c_str(), left_.config.ip.c_str(), hz);
}

InspireHandDriver::~InspireHandDriver() {}

void InspireHandDriver::init_hand(Hand & hand)
{
  hand.modbus = std::make_unique<ModbusClient>(
    hand.config.ip, hand.config.port, hand.config.device_id);

  if (ensure_connected(hand)) {
    // 에러 초기화
    try {
      hand.modbus->write_registers(reg::RESET_ERR, {1});
    } catch (...) {}
  }

  std::string ctrl_topic  = "/rt/inspire_hand/ctrl/"  + hand.config.side;
  std::string state_topic = "/rt/inspire_hand/state/" + hand.config.side;

  // ctrl_callback: 캐시에만 저장 — Modbus 건드리지 않음
  hand.sub = this->create_subscription<CtrlMsg>(
    ctrl_topic, rclcpp::QoS(1),
    [&hand](const CtrlMsg::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(hand.cache_mutex);
      hand.cached_ctrl = *msg;
    });

  hand.state_pub = this->create_publisher<StateMsg>(state_topic, rclcpp::QoS(1));

  // touch publisher — 활성화 시 아래 주석 해제 + 헤더/using/do_touch_read_publish 함께 해제
  // std::string touch_topic = "/rt/inspire_hand/touch/" + hand.config.side;
  // hand.touch_pub = this->create_publisher<TouchMsg>(touch_topic, rclcpp::QoS(1));

  RCLCPP_INFO(get_logger(), "[%s] sub:%s", hand.config.side.c_str(), ctrl_topic.c_str());
}

// ── 타이머에서 100Hz 호출 ──────────────────────────
void InspireHandDriver::tick(Hand & hand)
{
  if (!ensure_connected(hand)) return;

  // 1. 최신 캐시된 명령 write
  {
    std::optional<CtrlMsg> ctrl;
    {
      std::lock_guard<std::mutex> lock(hand.cache_mutex);
      ctrl = hand.cached_ctrl;
      hand.cached_ctrl.reset();  // 소비
    }
    if (ctrl) {
      do_write(hand, *ctrl);
    }
  }

  // 2. state read + publish
  do_read_publish(hand);

  // 3. touch read + publish — 활성화 시 아래 주석 해제
  // if (++hand.touch_tick >= touch_interval_) {
  //   hand.touch_tick = 0;
  //   do_touch_read_publish(hand);
  // }
}

bool InspireHandDriver::ensure_connected(Hand & hand)
{
  if (hand.modbus->is_connected()) return true;

  try {
    hand.modbus->connect();
    RCLCPP_INFO(get_logger(), "[%s] reconnected", hand.config.side.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[%s] reconnect failed: %s", hand.config.side.c_str(), e.what());
    return false;
  }
}

void InspireHandDriver::do_write(Hand & hand, const CtrlMsg & msg)
{
  try {
    if (msg.mode & 0b0001) {
      hand.modbus->write_registers(reg::ANGLE_SET,
        std::vector<int16_t>(msg.angle_set.begin(), msg.angle_set.end()));
    }
    if (msg.mode & 0b0010) {
      hand.modbus->write_registers(reg::POS_SET,
        std::vector<int16_t>(msg.pos_set.begin(), msg.pos_set.end()));
    }
    if (msg.mode & 0b0100) {
      hand.modbus->write_registers(reg::FORCE_SET,
        std::vector<int16_t>(msg.force_set.begin(), msg.force_set.end()));
    }
    if (msg.mode & 0b1000) {
      hand.modbus->write_registers(reg::SPEED_SET,
        std::vector<int16_t>(msg.speed_set.begin(), msg.speed_set.end()));
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[%s] write failed: %s", hand.config.side.c_str(), e.what());
  }
}

void InspireHandDriver::do_read_publish(Hand & hand)
{
  try {
    auto pos_act   = hand.modbus->read_registers_short(reg::POS_ACT,   6);
    auto angle_act = hand.modbus->read_registers_short(reg::ANGLE_ACT, 6);
    auto force_act = hand.modbus->read_registers_short(reg::FORCE_ACT, 6);
    auto current   = hand.modbus->read_registers_short(reg::CURRENT,   6);
    auto err       = hand.modbus->read_registers_byte(reg::ERR,     3);
    auto status    = hand.modbus->read_registers_byte(reg::STATUS,  3);
    auto temp      = hand.modbus->read_registers_byte(reg::TEMP,    3);

    StateMsg msg;
    for (int i = 0; i < 6; ++i) {
      msg.pos_act[i]   = pos_act[i];
      msg.angle_act[i] = angle_act[i];
      msg.force_act[i] = force_act[i];
      msg.current[i]   = current[i];

      msg.err[i]         = err[i];
      msg.status[i]      = status[i];
      msg.temperature[i] = temp[i];
    }
    hand.state_pub->publish(msg);

  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[%s] read failed: %s", hand.config.side.c_str(), e.what());
  }
}

// ── touch read + publish — 활성화 시 주석 전체 해제 ──────────
// read count = data_sheet 레지스터 수 / 2 (각 센서값 = 1 레지스터 int16)
// void InspireHandDriver::do_touch_read_publish(Hand & hand)
// {
//   try {
//     TouchMsg msg;
//     auto read_s = [&](int addr, int n) {
//       return hand.modbus->read_registers_short(addr, n);
//     };
//     auto r = read_s(reg::TOUCH_FINGERONE_TIP,     9); std::copy(r.begin(), r.end(), msg.fingerone_tip_touch.begin());
//     r = read_s(reg::TOUCH_FINGERONE_TOP,    96); std::copy(r.begin(), r.end(), msg.fingerone_top_touch.begin());
//     r = read_s(reg::TOUCH_FINGERONE_PALM,   80); std::copy(r.begin(), r.end(), msg.fingerone_palm_touch.begin());
//     r = read_s(reg::TOUCH_FINGERTWO_TIP,     9); std::copy(r.begin(), r.end(), msg.fingertwo_tip_touch.begin());
//     r = read_s(reg::TOUCH_FINGERTWO_TOP,    96); std::copy(r.begin(), r.end(), msg.fingertwo_top_touch.begin());
//     r = read_s(reg::TOUCH_FINGERTWO_PALM,   80); std::copy(r.begin(), r.end(), msg.fingertwo_palm_touch.begin());
//     r = read_s(reg::TOUCH_FINGERTHREE_TIP,   9); std::copy(r.begin(), r.end(), msg.fingerthree_tip_touch.begin());
//     r = read_s(reg::TOUCH_FINGERTHREE_TOP,  96); std::copy(r.begin(), r.end(), msg.fingerthree_top_touch.begin());
//     r = read_s(reg::TOUCH_FINGERTHREE_PALM, 80); std::copy(r.begin(), r.end(), msg.fingerthree_palm_touch.begin());
//     r = read_s(reg::TOUCH_FINGERFOUR_TIP,    9); std::copy(r.begin(), r.end(), msg.fingerfour_tip_touch.begin());
//     r = read_s(reg::TOUCH_FINGERFOUR_TOP,   96); std::copy(r.begin(), r.end(), msg.fingerfour_top_touch.begin());
//     r = read_s(reg::TOUCH_FINGERFOUR_PALM,  80); std::copy(r.begin(), r.end(), msg.fingerfour_palm_touch.begin());
//     r = read_s(reg::TOUCH_FINGERFIVE_TIP,    9); std::copy(r.begin(), r.end(), msg.fingerfive_tip_touch.begin());
//     r = read_s(reg::TOUCH_FINGERFIVE_TOP,   96); std::copy(r.begin(), r.end(), msg.fingerfive_top_touch.begin());
//     r = read_s(reg::TOUCH_FINGERFIVE_MIDDLE, 9); std::copy(r.begin(), r.end(), msg.fingerfive_middle_touch.begin());
//     r = read_s(reg::TOUCH_FINGERFIVE_PALM,  96); std::copy(r.begin(), r.end(), msg.fingerfive_palm_touch.begin());
//     r = read_s(reg::TOUCH_PALM,            112); std::copy(r.begin(), r.end(), msg.palm_touch.begin());
//     hand.touch_pub->publish(msg);
//   } catch (const std::exception & e) {
//     RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
//       "[%s] touch read failed: %s", hand.config.side.c_str(), e.what());
//   }
// }

}  // namespace inspire_hand_driver
