/**
 * rby1_rt_node.cpp
 *
 * C++ ROS2 node replacing new_core_main.py with improved real-time performance.
 *
 * Improvements over Python:
 *   1. StartStateUpdate callback — robot state fetch decoupled from control loop
 *   2. clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) — µs-precision loop timing
 *   3. pthread_setschedparam(SCHED_FIFO, 80) — RT thread priority (needs CAP_SYS_NICE)
 *   4. No GC / GIL pauses
 *
 * Compatible with small_main.py:
 *   - All /rby1_command action strings handled
 *   - /rby1_status JSON matches small_main parse keys
 *   - /rby1_status_joint, /rby1_teleop_command, /rby1_impedance_teleop_command
 */

#include <pthread.h>
#include <time.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "rby1_core_msgs/action/rby1_command.hpp"

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace rb::y1_model;

// Model selection: define USE_MODEL_M to build for mecanum (robot_m, 4-wheel)
#ifdef USE_MODEL_M
  using ModelType = M;
  static constexpr int kNumWheel = 4;
#else
  using ModelType = A;
  static constexpr int kNumWheel = 2;
#endif

// SDK dynamics types: dyn::Robot<DOF> / dyn::State<DOF>
using DynRobot = dyn::Robot<ModelType::kRobotDOF>;
using DynState = dyn::State<ModelType::kRobotDOF>;

static constexpr double kStreamDt   = 0.02;   // 50 Hz
static constexpr int    kNumBody    = 20;      // 6 torso + 7 rarm + 7 larm
static constexpr double kStopWheelT = 0.5;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static void set_rt_priority(int priority = 80) {
  struct sched_param param{};
  param.sched_priority = priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0)
    RCLCPP_WARN(rclcpp::get_logger("rby1_rt"), "RT priority failed (need CAP_SYS_NICE)");
}

static void sleep_until_abs(struct timespec& next, long dt_ns) {
  next.tv_nsec += dt_ns;
  while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
}

static std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, delim)) out.push_back(tok);
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Joint Position Teleop Controller
// ─────────────────────────────────────────────────────────────────────────────

struct JointTeleopController {
  bool enabled{false};
  bool new_cmd{false};
  int  traj_dt_cnt{0};

  std::array<double, kNumBody> q{}, q_filtered{};
  std::array<double, kNumBody> max_q{}, min_q{}, max_dq{}, max_ddq{};
  std::array<double, 16> cmd_array{};
  double gripper_p_ref[2]{0.5, 0.5};
  double gripper_ref[2]{0., 0.};
  double gripper_pos[2]{0., 0.};
  std::mutex mtx;

  void on_msg(const interbotix_xs_msgs::msg::JointGroupCommand::SharedPtr m) {
    if (!enabled) { new_cmd = false; return; }
    std::lock_guard<std::mutex> lk(mtx);
    for (size_t i = 0; i < std::min(m->cmd.size(), cmd_array.size()); ++i)
      cmd_array[i] = m->cmd[i];
    new_cmd = true;
  }

  // Fills `out` and returns true when a new command is ready.
  // Returns false → caller sends hold command.
  bool compute(const RobotState<ModelType>& rs,
               DynRobot& dyn,
               const std::shared_ptr<DynState>& ds,
               double dt,
               JointPositionCommandBuilder& out) {
    if (traj_dt_cnt == 0) {
      auto ub  = dyn.GetLimitQUpper(ds);
      auto lb  = dyn.GetLimitQLower(ds);
      auto dq  = dyn.GetLimitQdotUpper(ds);
      auto ddq = dyn.GetLimitQddotUpper(ds);
      for (int i = 0; i < kNumBody; ++i) {
        max_q[i]   = ub[kNumWheel+i];  min_q[i]   = lb[kNumWheel+i];
        max_dq[i]  = dq[kNumWheel+i];  max_ddq[i] = ddq[kNumWheel+i];
      }
      for (int i = 6; i < kNumBody; ++i) { max_dq[i] *= 10.; max_ddq[i] *= 30.; }
      for (int i = 0; i < kNumBody; ++i) {
        q[i] = rs.position[kNumWheel+i];
        q_filtered[i] = q[i];
      }
    }
    if (!new_cmd) return false;
    new_cmd = false;
    {
      std::lock_guard<std::mutex> lk(mtx);
      for (int i = 0; i < 7; ++i) q[6 +i] = cmd_array[i];
      for (int i = 0; i < 7; ++i) q[13+i] = cmd_array[7+i];
      gripper_p_ref[0] = cmd_array[14];
      gripper_p_ref[1] = cmd_array[15];
    }
    constexpr double dz = 0.1, gain = 2.;
    for (int i = 0; i < 2; ++i) {
      double pos = gripper_pos[i] * 0.8, ref = gripper_p_ref[i];
      if      (pos > ref + dz) gripper_ref[i] = std::max(-0.5, gain*(ref+dz-pos));
      else if (pos < ref - dz) gripper_ref[i] = std::min( 0.5, gain*(ref-dz-pos));
      else                     gripper_ref[i] = 0.;
    }
    constexpr double alpha = 0.95;
    for (int i = 0; i < kNumBody; ++i)
      q_filtered[i] = alpha*q_filtered[i] + (1.-alpha)*q[i];
    std::array<double, kNumBody> qc;
    for (int i = 0; i < kNumBody; ++i)
      qc[i] = std::clamp(q_filtered[i], min_q[i], max_q[i]);
    Eigen::Map<Eigen::VectorXd> qv(qc.data(), kNumBody);
    Eigen::Map<Eigen::VectorXd> dqv(max_dq.data(), kNumBody);
    Eigen::Map<Eigen::VectorXd> ddqv(max_ddq.data(), kNumBody);
    out.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(dt*30))
       .SetPosition(qv).SetVelocityLimit(dqv).SetAccelerationLimit(ddqv).SetMinimumTime(dt*2);
    traj_dt_cnt++;
    return true;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Joint Impedance Teleop Controller
// ─────────────────────────────────────────────────────────────────────────────

struct JointImpedanceTeleopController {
  bool enabled{false};
  bool new_cmd{false};
  int  traj_dt_cnt{0};

  std::array<double, kNumBody> q{}, q_filtered{};
  std::array<double, kNumBody> max_q{}, min_q{};
  std::array<double, 16> cmd_array{};
  double gripper_p_ref[2]{0., 0.};
  double gripper_ref[2]{0., 0.};
  double gripper_pos[2]{0., 0.};

  bool imp_ref_set{false};
  Eigen::VectorXd q_ref_imp{Eigen::VectorXd::Zero(kNumBody)};

  std::mutex mtx;

  void on_msg(const interbotix_xs_msgs::msg::JointGroupCommand::SharedPtr m) {
    if (!enabled) { new_cmd = false; return; }
    std::lock_guard<std::mutex> lk(mtx);
    for (size_t i = 0; i < std::min(m->cmd.size(), cmd_array.size()); ++i)
      cmd_array[i] = m->cmd[i];
    new_cmd = true;
  }

  // Fills out_active (new cmd) or out_hold (impedance hold at cached ref).
  // Returns true if out_active was filled.
  bool compute(const RobotState<ModelType>& rs,
               DynRobot& dyn,
               const std::shared_ptr<DynState>& ds,
               double dt,
               BodyComponentBasedCommandBuilder& out_active,
               BodyComponentBasedCommandBuilder& out_hold) {
    if (traj_dt_cnt == 0) {
      auto ub = dyn.GetLimitQUpper(ds);
      auto lb = dyn.GetLimitQLower(ds);
      for (int i = 0; i < kNumBody; ++i) {
        max_q[i] = ub[kNumWheel+i];
        min_q[i] = lb[kNumWheel+i];
      }
      for (int i = 0; i < kNumBody; ++i) {
        q[i] = rs.position[kNumWheel+i];
        q_filtered[i] = q[i];
      }
      gripper_p_ref[0] = gripper_p_ref[1] = 0.;
      imp_ref_set = false;
    }

    const Eigen::Vector<double,7> K  = (Eigen::Vector<double,7>() << 80,80,80,80,80,80,40).finished();
    const Eigen::Vector<double,7> tq = (Eigen::Vector<double,7>() << 35,35,35,20,20,20,15).finished();

    auto fill_bc = [&](BodyComponentBasedCommandBuilder& bc, const Eigen::VectorXd& qc) {
      Eigen::Map<const Eigen::VectorXd> ra(qc.data()+6,  7);
      Eigen::Map<const Eigen::VectorXd> la(qc.data()+13, 7);
      Eigen::Map<const Eigen::VectorXd> to(qc.data(),    6);
      JointImpedanceControlCommandBuilder ra_cmd, la_cmd;
      ra_cmd.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(dt*30))
            .SetPosition(ra).SetMinimumTime(dt*2).SetStiffness(K).SetDampingRatio(1.0).SetTorqueLimit(tq);
      la_cmd.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(dt*30))
            .SetPosition(la).SetMinimumTime(dt*2).SetStiffness(K).SetDampingRatio(1.0).SetTorqueLimit(tq);
      JointPositionCommandBuilder to_cmd;
      to_cmd.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(dt*30))
            .SetPosition(to).SetMinimumTime(dt*2);
      bc.SetRightArmCommand(ra_cmd).SetLeftArmCommand(la_cmd).SetTorsoCommand(to_cmd);
    };

    if (!new_cmd) {
      if (!imp_ref_set) {
        q_ref_imp.resize(kNumBody);
        for (int i = 0; i < kNumBody; ++i)
          q_ref_imp[i] = rs.target_position[kNumWheel+i];
        imp_ref_set = true;
      }
      fill_bc(out_hold, q_ref_imp);
      return false;
    }
    new_cmd     = false;
    imp_ref_set = false;
    {
      std::lock_guard<std::mutex> lk(mtx);
      for (int i = 0; i < 7; ++i) q[6 +i] = cmd_array[i];
      for (int i = 0; i < 7; ++i) q[13+i] = cmd_array[7+i];
      gripper_p_ref[0] = cmd_array[14];
      gripper_p_ref[1] = cmd_array[15];
    }
    constexpr double dz = 0.1, gain = 2.;
    for (int i = 0; i < 2; ++i) {
      double pos = gripper_pos[i]*0.8, ref = gripper_p_ref[i];
      if      (pos > ref+dz) gripper_ref[i] = std::max(-0.5, gain*(ref+dz-pos));
      else if (pos < ref-dz) gripper_ref[i] = std::min( 0.5, gain*(ref-dz-pos));
      else                   gripper_ref[i] = 0.;
    }
    constexpr double alpha = 0.95;
    for (int i = 0; i < kNumBody; ++i)
      q_filtered[i] = alpha*q_filtered[i] + (1.-alpha)*q[i];
    Eigen::VectorXd qc(kNumBody);
    for (int i = 0; i < kNumBody; ++i)
      qc[i] = std::clamp(q_filtered[i], min_q[i], max_q[i]);
    fill_bc(out_active, qc);
    traj_dt_cnt++;
    return true;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Main Node
// ─────────────────────────────────────────────────────────────────────────────

class Rby1RtNode : public rclcpp::Node {
 public:
  using Rby1Command   = rby1_core_msgs::action::Rby1Command;
  using GoalHandleCmd = rclcpp_action::ServerGoalHandle<Rby1Command>;

  Rby1RtNode() : Node("rby1_core_node") {
    sub_base_vel_ = create_subscription<geometry_msgs::msg::Twist>(
        "/rby1_base_velocity", 2,
        [this](const geometry_msgs::msg::Twist::SharedPtr m) {
          x_speed_ = m->linear.x; y_speed_ = m->linear.y; yaw_speed_ = m->angular.z; base_cnt_ = 0;
        });
    sub_teleop_ = create_subscription<interbotix_xs_msgs::msg::JointGroupCommand>(
        "/rby1_teleop_command", 2,
        [this](const interbotix_xs_msgs::msg::JointGroupCommand::SharedPtr m){ ctrl_jp_.on_msg(m); });
    sub_imp_teleop_ = create_subscription<interbotix_xs_msgs::msg::JointGroupCommand>(
        "/rby1_impedance_teleop_command", 2,
        [this](const interbotix_xs_msgs::msg::JointGroupCommand::SharedPtr m){ ctrl_jip_.on_msg(m); });
    sub_pink_teleop_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "/rby1_pink_teleop_command", 2,
        [this](const geometry_msgs::msg::PoseArray::SharedPtr){ /* TODO: pink IK */ });

    pub_status_ = create_publisher<std_msgs::msg::String>("/rby1_status", 2);
    pub_joints_ = create_publisher<sensor_msgs::msg::JointState>("/rby1_status_joint", 2);

    action_server_ = rclcpp_action::create_server<Rby1Command>(
        this, "/rby1_command",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Rby1Command::Goal>){
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const std::shared_ptr<GoalHandleCmd>){ stop_move(); return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<GoalHandleCmd> gh){
          std::thread([this, gh](){ execute_command(gh); }).detach(); });

    stream_thread_ = std::thread(&Rby1RtNode::stream_loop, this);
    RCLCPP_INFO(get_logger(), "Rby1RtNode ready");
  }

  ~Rby1RtNode() {
    stream_enabled_ = false;
    if (stream_thread_.joinable()) stream_thread_.join();
  }

 private:
  std::shared_ptr<Robot<ModelType>>                     robot_;
  std::unique_ptr<RobotCommandStreamHandler<ModelType>> stream_;
  std::shared_ptr<DynRobot>                             dyn_;
  std::shared_ptr<DynState>                             dyn_state_;

  std::shared_ptr<const RobotState<ModelType>> cached_state_;
  std::mutex state_mtx_;

  std::atomic<bool> stream_enabled_{false};
  std::atomic<bool> robot_ok_{false};
  std::atomic<bool> no_gripper_{true};
  std::atomic<bool> power_on_{false};
  std::atomic<bool> servo_on_{false};

  std::string ctr_type_{"JointPosition"};
  std::mutex  ctr_type_mtx_;

  JointTeleopController          ctrl_jp_;
  JointImpedanceTeleopController ctrl_jip_;

  double x_speed_{0.}, y_speed_{0.}, yaw_speed_{0.};
  int    base_cnt_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_base_vel_;
  rclcpp::Subscription<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr sub_teleop_, sub_imp_teleop_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pink_teleop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp_action::Server<Rby1Command>::SharedPtr action_server_;
  std::thread stream_thread_;

  // ── Stream loop (RT thread, 50 Hz) ──────────────────────────────────────
  void stream_loop() {
    set_rt_priority(80);
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long dt_ns = static_cast<long>(kStreamDt * 1e9);

    while (rclcpp::ok()) {
      if (!stream_enabled_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        clock_gettime(CLOCK_MONOTONIC, &next);
        continue;
      }

      std::shared_ptr<const RobotState<ModelType>> rs;
      { std::lock_guard<std::mutex> lk(state_mtx_); rs = cached_state_; }
      if (!rs) { sleep_until_abs(next, dt_ns); continue; }

      // Mobility
      base_cnt_++;
      if (base_cnt_ * kStreamDt > kStopWheelT) {
        x_speed_ = y_speed_ = yaw_speed_ = 0.;
        base_cnt_ = static_cast<int>(kStopWheelT / kStreamDt * 2);
      }
      Eigen::Vector2d lv(x_speed_, y_speed_);
      MobilityCommandBuilder mc;
      mc.SetCommand(SE2VelocityCommandBuilder()
          .SetVelocity(lv, yaw_speed_)
          .SetMinimumTime(kStreamDt * 5)
          .SetAccelerationLimit(Eigen::Vector2d::Constant(10.), 10.));

      std::string ctr;
      { std::lock_guard<std::mutex> lk(ctr_type_mtx_); ctr = ctr_type_; }

      RobotCommandBuilder rc;

      if (ctr == "JointPosition") {
        JointPositionCommandBuilder bc;
        if (ctrl_jp_.compute(*rs, *dyn_, dyn_state_, kStreamDt, bc)) {
          ComponentBasedCommandBuilder cbc;
          cbc.SetBodyCommand(bc).SetMobilityCommand(mc);
          rc.SetCommand(cbc);
        } else {
          // Hold: use current encoder position (target_position may be 0 before first cmd)
          Eigen::VectorXd qh(kNumBody);
          for (int i = 0; i < kNumBody; ++i) qh[i] = rs->position[kNumWheel+i];
          JointPositionCommandBuilder hold;
          hold.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(kStreamDt*30))
              .SetPosition(qh).SetMinimumTime(kStreamDt*10);
          ComponentBasedCommandBuilder cbc;
          cbc.SetBodyCommand(hold).SetMobilityCommand(mc);
          rc.SetCommand(cbc);
        }
      } else if (ctr == "JointImpedance") {
        BodyComponentBasedCommandBuilder bc_active, bc_hold;
        bool has_new = ctrl_jip_.compute(*rs, *dyn_, dyn_state_, kStreamDt, bc_active, bc_hold);
        ComponentBasedCommandBuilder cbc;
        cbc.SetBodyCommand(has_new ? bc_active : bc_hold).SetMobilityCommand(mc);
        rc.SetCommand(cbc);
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unknown ctr_type: %s", ctr.c_str());
        sleep_until_abs(next, dt_ns);
        continue;
      }

      if (stream_ && stream_->IsDone()) {
        RCLCPP_ERROR(get_logger(), "stream died before SendCommand (OnDone fired late)");
        stream_enabled_ = false;
        stream_->Cancel(); stream_.reset();
        ctrl_jp_.enabled = ctrl_jip_.enabled = false;
        if (robot_) {
          auto cms2 = robot_->GetControlManagerState();
          RCLCPP_ERROR(get_logger(), "cms after stream death: %s",
                       rb::to_string(cms2.state).c_str());
        }
        sleep_until_abs(next, dt_ns);
        continue;
      }
      try {
        if (stream_) stream_->SendCommand(rc);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "stream expired: %s", e.what());
        stream_enabled_ = false;
        if (stream_) { stream_->Cancel(); stream_.reset(); }
        ctrl_jp_.enabled = ctrl_jip_.enabled = false;
        if (robot_) {
          auto cms2 = robot_->GetControlManagerState();
          RCLCPP_ERROR(get_logger(), "cms after stream expired: %s",
                       rb::to_string(cms2.state).c_str());
        }
      }
      sleep_until_abs(next, dt_ns);
    }
  }

  // ── State update callback (SDK thread, ~50 Hz) ───────────────────────────
  void on_state(const RobotState<ModelType>& rs, const ControlManagerState& cms) {
    { std::lock_guard<std::mutex> lk(state_mtx_); cached_state_ = std::make_shared<RobotState<ModelType>>(rs); }

    sensor_msgs::msg::JointState jmsg;
    jmsg.header.stamp = get_clock()->now();
    for (double v : rs.position) jmsg.position.push_back(v);
    for (double v : rs.velocity) jmsg.velocity.push_back(v);
    for (double v : rs.torque)   jmsg.effort.push_back(v);
    pub_joints_->publish(jmsg);

    std::string ctrl_str;
    switch (cms.state) {
      case ControlManagerState::State::kEnabled:    ctrl_str = "State.Enabled";    break;
      case ControlManagerState::State::kMajorFault: ctrl_str = "State.MajorFault"; break;
      case ControlManagerState::State::kMinorFault: ctrl_str = "State.MinorFault"; break;
      default:                                      ctrl_str = "State.Idle";       break;
    }
    std::string ctr_snap;
    { std::lock_guard<std::mutex> lk(ctr_type_mtx_); ctr_snap = ctr_type_; }

    std::ostringstream json;
    json << "{"
         << "\"control_state\":\"" << ctrl_str                              << "\","
         << "\"power_state\":\""   << (power_on_.load()    ? "True":"False") << "\","
         << "\"servo_state\":\""   << (servo_on_.load()    ? "True":"False") << "\","
         << "\"stream_state\":\""  << (stream_enabled_.load() ? "True":"False") << "\","
         << "\"gripper_state\":\"" << (no_gripper_.load()  ? "True":"False") << "\","
         << "\"ctr_type\":\""      << ctr_snap                             << "\""
         << "}";
    std_msgs::msg::String smsg;
    smsg.data = json.str();
    pub_status_->publish(smsg);
  }

  // ── Command dispatch ─────────────────────────────────────────────────────
  void execute_command(const std::shared_ptr<GoalHandleCmd>& gh) {
    auto result = std::make_shared<Rby1Command::Result>();
    const std::string& cmd = gh->get_goal()->command;
    RCLCPP_INFO(get_logger(), "Command: %s", cmd.c_str());
    auto parts = split(cmd, '\n');
    bool ok = false;

    if      (parts[0] == "connect")                ok = cmd_connect(parts);
    else if (parts[0] == "power_on")               ok = cmd_power_on();
    else if (parts[0] == "power_off")              ok = cmd_power_off();
    else if (parts[0] == "servo_on")               ok = cmd_servo_on();
    else if (parts[0] == "servo_on_no_wheel")      ok = cmd_servo_on_no_wheel();
    else if (parts[0] == "gripper_init")           ok = cmd_gripper_init();
    else if (parts[0] == "control_enable")         ok = cmd_control_enable();
    else if (parts[0] == "error_reset")            ok = cmd_error_reset();
    else if (parts[0] == "stop_move")              ok = stop_move();
    else if (parts[0] == "stream_start")           ok = cmd_stream_start(parts.size() > 1 ? parts[1] : "JointPosition");
    else if (parts[0] == "stream_stop")            ok = cmd_stream_stop();
    else if (parts[0] == "teleop_start")           ok = cmd_teleop_start();
    else if (parts[0] == "impedance_teleop_start") ok = cmd_impedance_teleop_start();
    else if (parts[0] == "teleop_pink_start")      ok = cmd_teleop_start();
    else if (parts[0] == "teleop_stop")            ok = cmd_teleop_stop();
    else if (parts[0] == "zero_pose")              ok = cmd_pose(Eigen::VectorXd::Zero(kNumBody));
    else if (parts[0] == "ready_pose")             ok = cmd_pose(build_ready_q());
    else if (parts[0] == "vla_pose")               ok = cmd_pose(build_vla_q());
    else if (parts[0] == "vla_pose2")              ok = cmd_pose(build_vla2_q());
    else if (parts[0] == "clean_test")             ok = cmd_clean_test();
    else RCLCPP_WARN(get_logger(), "Unknown command: %s", cmd.c_str());

    result->response = ok ? "command success" : ("command failed: " + cmd);
    if (ok) gh->succeed(result); else gh->abort(result);
  }

  // ── SDK helpers ──────────────────────────────────────────────────────────
  bool check() const { return robot_ && robot_->IsConnected(); }

  bool cmd_connect(const std::vector<std::string>& parts) {
    if (parts.size() < 2) return false;
    no_gripper_ = (parts.size() > 2 && parts[2] == "no_gripper");
    robot_ = Robot<ModelType>::Create(parts[1]);
    if (!robot_->Connect()) return false;
    dyn_       = robot_->GetDynamics();
    dyn_state_ = dyn_->MakeState({"base","ee_right","ee_left","link_torso_5"}, ModelType::kRobotJointNames);
    robot_->StartStateUpdate(
        [this](const RobotState<ModelType>& rs, const ControlManagerState& cms){ on_state(rs, cms); },
        50.0);
    RCLCPP_INFO(get_logger(), "Connected to %s%s", parts[1].c_str(), no_gripper_.load() ? " (no_gripper)" : "");
    return true;
  }

  bool cmd_power_on() {
    if (!check()) return false;
    if (!robot_->IsPowerOn(".*") && !robot_->PowerOn(".*")) return false;
    power_on_ = true;
    return true;
  }
  bool cmd_power_off() {
    if (!check() || !robot_->PowerOff(".*")) return false;
    power_on_ = false; servo_on_ = false;
    return true;
  }
  bool cmd_servo_on() {
    if (!check()) return false;
    if (!robot_->IsServoOn("^((?!head).)*") && !robot_->ServoOn("^((?!head).)*")) return false;
    servo_on_ = true;
    return true;
  }
  bool cmd_servo_on_no_wheel() {
    if (!check()) return false;
    if (!robot_->IsServoOn("torso_.*|right_arm_.*|left_arm_.*") &&
        !robot_->ServoOn("torso_.*|right_arm_.*|left_arm_.*")) return false;
    servo_on_ = true;
    return true;
  }
  bool cmd_gripper_init() {
    if (no_gripper_.load()) return true;
    RCLCPP_WARN(get_logger(), "gripper_init: not implemented in C++ node");
    return false;
  }
  bool cmd_control_enable() {
    if (!check() || !robot_->EnableControlManager()) return false;
    robot_ok_ = true;
    return true;
  }
  bool cmd_error_reset() {
    if (!check()) return false;
    auto s = robot_->GetControlManagerState();
    if (s.state == ControlManagerState::State::kMajorFault ||
        s.state == ControlManagerState::State::kMinorFault)
      return robot_->ResetFaultControlManager();
    return true;
  }
  bool stop_move() {
    if (!check()) return false;
    cmd_stream_stop();
    RobotCommandBuilder rc;
    rc.SetCommand(WholeBodyCommandBuilder().SetCommand(StopCommandBuilder()));
    return robot_->SendCommand(rc, 99)->Get().finish_code() == RobotCommandFeedback::FinishCode::kOk;
  }
  bool cmd_stream_start(const std::string& type) {
    if (!check() || !robot_ok_ || stream_enabled_) return false;
    auto cms = robot_->GetControlManagerState();
    RCLCPP_INFO(get_logger(), "cmd_stream_start: cms=%s", rb::to_string(cms.state).c_str());
    if (cms.state != ControlManagerState::State::kEnabled) return false;
    stream_ = robot_->CreateCommandStream();
    // 현재 상태로 hold command를 즉시 전송 — Python example 17 방식
    // stream 생성 직후 첫 command를 즉시 보내야 서버가 stream을 유지함
    try {
      auto rs = robot_->GetState();
      Eigen::VectorXd qh(kNumBody);
      for (int i = 0; i < kNumBody; ++i) qh[i] = rs.position[kNumWheel + i];
      RCLCPP_INFO(get_logger(), "hold q torso: %.3f %.3f %.3f %.3f %.3f %.3f",
                  qh[0], qh[1], qh[2], qh[3], qh[4], qh[5]);
      RCLCPP_INFO(get_logger(), "hold q rarm:  %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                  qh[6], qh[7], qh[8], qh[9], qh[10], qh[11], qh[12]);
      RCLCPP_INFO(get_logger(), "hold q larm:  %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                  qh[13], qh[14], qh[15], qh[16], qh[17], qh[18], qh[19]);
      MobilityCommandBuilder mc;
      mc.SetCommand(SE2VelocityCommandBuilder()
          .SetVelocity(Eigen::Vector2d::Zero(), 0.)
          .SetMinimumTime(kStreamDt * 5)
          .SetAccelerationLimit(Eigen::Vector2d::Constant(10.), 10.));
      JointPositionCommandBuilder hold;
      hold.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(kStreamDt * 30))
          .SetPosition(qh).SetMinimumTime(kStreamDt * 10);
      ComponentBasedCommandBuilder cbc;
      cbc.SetBodyCommand(hold).SetMobilityCommand(mc);
      RobotCommandBuilder rc;
      rc.SetCommand(cbc);
      stream_->SendCommand(rc);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "cmd_stream_start: initial command failed: %s", e.what());
      stream_.reset();
      return false;
    }
    { std::lock_guard<std::mutex> lk(ctr_type_mtx_); ctr_type_ = type; }
    stream_enabled_ = true;
    return true;
  }
  bool cmd_stream_stop() {
    stream_enabled_ = false;
    if (stream_) {
      std::this_thread::sleep_for(std::chrono::duration<double>(kStreamDt * 2));
      stream_->Cancel();
      stream_.reset();
    }
    ctrl_jp_.enabled = ctrl_jip_.enabled = false;
    return true;
  }
  bool cmd_teleop_start() {
    if (!cmd_stream_start("JointPosition")) return false;
    ctrl_jp_.traj_dt_cnt = 0; ctrl_jp_.new_cmd = false; ctrl_jp_.enabled = true;
    return true;
  }
  bool cmd_impedance_teleop_start() {
    if (!cmd_stream_start("JointImpedance")) return false;
    ctrl_jip_.traj_dt_cnt = 0; ctrl_jip_.new_cmd = false;
    ctrl_jip_.imp_ref_set = false; ctrl_jip_.enabled = true;
    return true;
  }
  bool cmd_teleop_stop() {
    ctrl_jp_.enabled = ctrl_jip_.enabled = false;
    return cmd_stream_stop();
  }
  bool cmd_pose(const Eigen::VectorXd& q, double t = 5.) {
    if (!check() || !robot_ok_) return false;
    cmd_stream_stop();
    JointPositionCommandBuilder bc;
    bc.SetPosition(q).SetMinimumTime(t);
    ComponentBasedCommandBuilder cbc;
    cbc.SetBodyCommand(bc);
    RobotCommandBuilder rc;
    rc.SetCommand(cbc);
    return robot_->SendCommand(rc, 20)->Get().finish_code() == RobotCommandFeedback::FinishCode::kOk;
  }
  bool cmd_clean_test() {
    if (!check() || !robot_ok_) return false;
    cmd_pose(Eigen::VectorXd::Zero(kNumBody));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    cmd_pose(build_vla_q());
    std::this_thread::sleep_for(std::chrono::seconds(2));
    cmd_pose(build_ready_q());
    std::this_thread::sleep_for(std::chrono::seconds(2));
    cmd_pose(Eigen::VectorXd::Zero(kNumBody));
    return true;
  }

  // ── Joint angle presets ───────────────────────────────────────────────────
  static Eigen::VectorXd build_ready_q() {
    Eigen::Vector<double,6> torso; torso << 0, 30, -60, 30, 0, 0;
    Eigen::Vector<double,7> ra;    ra    << -8.68, -9.86,  1.89, -103.95,  0.37, 22.07, -10.35;
    Eigen::Vector<double,7> la;    la    << -8.68,  9.86, -1.89, -103.95, -0.37, 22.07,  10.35;
    Eigen::VectorXd q(kNumBody);
    q << torso*(M_PI/180.), ra*(M_PI/180.), la*(M_PI/180.);
    return q;
  }
  static Eigen::VectorXd build_vla_q() {
    Eigen::Vector<double,6> torso; torso << 0.0, -0.223,  0.497, 0.0263, 0., 0.;
    Eigen::Vector<double,7> ra;    ra    << -1.80, -1.16,  1.30, -1.32, -1.47, 1.08,  2.3;
    Eigen::Vector<double,7> la;    la    << -1.80,  1.16, -1.30, -1.32,  1.47, 1.08, -2.3;
    Eigen::VectorXd q(kNumBody); q << torso, ra, la;
    return q;
  }
  static Eigen::VectorXd build_vla2_q() {
    constexpr double adj = 0.3, wrist = M_PI/2.;
    Eigen::Vector<double,6> torso; torso << 0.0, -0.223+adj,  0.497, 0.0263+adj, 0., 0.;
    Eigen::Vector<double,7> ra;    ra    << -1.80-adj, -1.16-adj*0.5,  1.30, -1.32, -1.47, 1.08,  2.3-wrist;
    Eigen::Vector<double,7> la;    la    << -1.80-adj,  1.16+adj*0.5, -1.30, -1.32,  1.47, 1.08, -2.3+wrist;
    Eigen::VectorXd q(kNumBody); q << torso, ra, la;
    return q;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Rby1RtNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
