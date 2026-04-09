#include "inspire_hand_driver/inspire_hand_driver.hpp"

#include <functional>
#include <chrono>

using namespace std::chrono_literals;
using InspireHandCtrl  = inspire_hand_msgs::msg::InspireHandCtrl;
using InspireHandState = inspire_hand_msgs::msg::InspireHandState;

namespace inspire_hand_driver
{

InspireHandDriver::InspireHandDriver(const rclcpp::NodeOptions & options)
: Node("inspire_hand_driver", options)
{
  this->declare_parameter("right_ip",    "192.168.123.210");
  this->declare_parameter("left_ip",     "192.168.123.211");
  this->declare_parameter("modbus_port", 6000);
  this->declare_parameter("device_id",   1);
  this->declare_parameter("state_hz",    50.0);
  this->declare_parameter("ctrl_hz",     50.0);

  ctrl_min_interval_s_ = 1.0 / this->get_parameter("ctrl_hz").as_double();

  right_.config = {
    this->get_parameter("right_ip").as_string(),
    "r",
    static_cast<int>(this->get_parameter("modbus_port").as_int()),
    static_cast<int>(this->get_parameter("device_id").as_int())
  };
  left_.config = {
    this->get_parameter("left_ip").as_string(),
    "l",
    static_cast<int>(this->get_parameter("modbus_port").as_int()),
    static_cast<int>(this->get_parameter("device_id").as_int())
  };

  init_hand(right_);
  init_hand(left_);

  double state_hz = this->get_parameter("state_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / state_hz);
  state_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {
      read_and_publish_state(right_);
      read_and_publish_state(left_);
    });

  RCLCPP_INFO(get_logger(),
    "InspireHandDriver started — right:%s left:%s state:%.0fHz ctrl:%.0fHz",
    right_.config.ip.c_str(), left_.config.ip.c_str(),
    state_hz, 1.0 / ctrl_min_interval_s_);
}

InspireHandDriver::~InspireHandDriver() {}

// ──────────────────────────────────────────────
// 내부 헬퍼: Modbus 연결 생성 + connect
// ──────────────────────────────────────────────
static std::unique_ptr<ModbusClient> make_modbus(const HandConfig & cfg, const std::string & label,
  rclcpp::Logger logger)
{
  auto mb = std::make_unique<ModbusClient>(cfg.ip, cfg.port, cfg.device_id);
  try {
    mb->connect();
    RCLCPP_INFO(logger, "[%s][%s] Modbus connected: %s:%d",
      cfg.side.c_str(), label.c_str(), cfg.ip.c_str(), cfg.port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "[%s][%s] connect failed: %s",
      cfg.side.c_str(), label.c_str(), e.what());
  }
  return mb;
}

void InspireHandDriver::init_hand(Hand & hand)
{
  hand.modbus_write = make_modbus(hand.config, "write", get_logger());
  hand.modbus_read  = make_modbus(hand.config, "read",  get_logger());

  // 에러 초기화 (write 연결로)
  if (hand.modbus_write->is_connected()) {
    try {
      std::vector<int16_t> reset_val = {1};
      hand.modbus_write->write_registers(reg::RESET_ERR, reset_val);
    } catch (...) {}
  }

  // ctrl subscriber
  std::string ctrl_topic = "/rt/inspire_hand/ctrl/" + hand.config.side;
  hand.sub = this->create_subscription<InspireHandCtrl>(
    ctrl_topic, rclcpp::QoS(1),
    [this, &hand](const InspireHandCtrl::SharedPtr msg) {
      ctrl_callback(hand, msg);
    });

  // state publisher
  std::string state_topic = "/rt/inspire_hand/state/" + hand.config.side;
  hand.state_pub = this->create_publisher<InspireHandState>(state_topic, rclcpp::QoS(1));

  RCLCPP_INFO(get_logger(), "[%s] sub: %s", hand.config.side.c_str(), ctrl_topic.c_str());
}

// ──────────────────────────────────────────────
// ctrl callback — write 전용 연결 사용
// ──────────────────────────────────────────────
void InspireHandDriver::ctrl_callback(
  Hand & hand,
  const InspireHandCtrl::SharedPtr msg)
{
  // rate limiting
  auto now = this->now();
  if ((now - hand.last_ctrl_time_).seconds() < ctrl_min_interval_s_) {
    return;
  }
  hand.last_ctrl_time_ = now;

  // reconnect if needed
  if (!hand.modbus_write->is_connected()) {
    try {
      hand.modbus_write->disconnect();
      hand.modbus_write->connect();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] write reconnect failed: %s", hand.config.side.c_str(), e.what());
      return;
    }
  }

  std::vector<int16_t> angle_vals(msg->angle_set.begin(), msg->angle_set.end());
  std::vector<int16_t> pos_vals(msg->pos_set.begin(), msg->pos_set.end());
  std::vector<int16_t> force_vals(msg->force_set.begin(), msg->force_set.end());
  std::vector<int16_t> speed_vals(msg->speed_set.begin(), msg->speed_set.end());

  try {
    if (msg->mode & 0b0001) {
      hand.modbus_write->write_registers(reg::ANGLE_SET, angle_vals);
    }
    if (msg->mode & 0b0010) {
      hand.modbus_write->write_registers(reg::POS_SET, pos_vals);
    }
    if (msg->mode & 0b0100) {
      hand.modbus_write->write_registers(reg::FORCE_SET, force_vals);
    }
    if (msg->mode & 0b1000) {
      hand.modbus_write->write_registers(reg::SPEED_SET, speed_vals);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[%s] write failed: %s", hand.config.side.c_str(), e.what());
  }
}

// ──────────────────────────────────────────────
// state read timer — read 전용 연결 사용
// ──────────────────────────────────────────────
void InspireHandDriver::read_and_publish_state(Hand & hand)
{
  // reconnect if needed
  if (!hand.modbus_read->is_connected()) {
    try {
      hand.modbus_read->disconnect();
      hand.modbus_read->connect();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] read reconnect failed: %s", hand.config.side.c_str(), e.what());
      return;
    }
  }

  InspireHandState state_msg;
  try {
    auto pos_act   = hand.modbus_read->read_registers_short(reg::POS_ACT,   6);
    auto angle_act = hand.modbus_read->read_registers_short(reg::ANGLE_ACT, 6);
    auto force_act = hand.modbus_read->read_registers_short(reg::FORCE_ACT, 6);
    auto current   = hand.modbus_read->read_registers_short(reg::CURRENT,   6);
    auto err       = hand.modbus_read->read_registers_byte(reg::ERR,    3);
    auto status    = hand.modbus_read->read_registers_byte(reg::STATUS, 3);
    auto temp      = hand.modbus_read->read_registers_byte(reg::TEMP,   3);

    for (int i = 0; i < 6; ++i) {
      state_msg.pos_act[i]   = pos_act[i];
      state_msg.angle_act[i] = angle_act[i];
      state_msg.force_act[i] = force_act[i];
      state_msg.current[i]   = current[i];
    }
    for (int i = 0; i < 6; ++i) {
      state_msg.err[i]         = err[i];
      state_msg.status[i]      = status[i];
      state_msg.temperature[i] = temp[i];
    }

    hand.state_pub->publish(state_msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[%s] state read failed: %s", hand.config.side.c_str(), e.what());
  }
}

}  // namespace inspire_hand_driver
