#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <chrono>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

class SpeedControlNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  SpeedControlNode()
  : Node("speed_control_node_simple"),
    speed_fraction_(1.0),
    has_active_traj_(false)
  {
    override_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/override_trajectory", 10,
      std::bind(&SpeedControlNode::on_override, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      std::bind(&SpeedControlNode::on_joint_state, this, std::placeholders::_1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/speed_command", 10,
      std::bind(&SpeedControlNode::on_speed, this, std::placeholders::_1));

    ack_pub_ = this->create_publisher<std_msgs::msg::String>("/override_ack", 10);
    result_pub_ = this->create_publisher<std_msgs::msg::String>("/override_result", 10);

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    if (!action_client_->wait_for_action_server(10s)) {
      RCLCPP_WARN(this->get_logger(), "Action server not available (waited 10s). Node still running and will try when sending.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Action server available.");
    }
  }

private:
  // ROS interfaces
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr override_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  // internal state (single-trajectory)
  trajectory_msgs::msg::JointTrajectory current_traj_;
  bool has_active_traj_;
  rclcpp::Time active_goal_start_time_;
  rclcpp::Time active_goal_sent_time_; // same as start_time but explicit
  std::shared_ptr<GoalHandleFollowJointTrajectory> active_goal_handle_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  double speed_fraction_;

  // -------------------------
  // Helpers
  // -------------------------
  static double normalize_angle(double a) {
    const double two_pi = 2.0 * M_PI;
    a = std::fmod(a + M_PI, two_pi);
    if (a < 0) a += two_pi;
    return a - M_PI;
  }

  double joint_pos_from_last_state(const std::string &name) {
    if (!last_joint_state_) return 0.0;
    for (size_t i=0; i<last_joint_state_->name.size(); ++i) {
      if (last_joint_state_->name[i] == name) {
        return normalize_angle(last_joint_state_->position[i]); // already radians per ROS spec
      }
    }
    // not found -> return 0 and warn once
    static std::unordered_set<std::string> warned;
    if (warned.find(name) == warned.end()) {
      RCLCPP_WARN(this->get_logger(), "Joint '%s' not found in /joint_states; using 0.0", name.c_str());
      warned.insert(name);
    }
    return 0.0;
  }

  // Build and send a FollowJointTrajectory goal using `traj` as-is.
  // This sets active goal tracking state.
  void send_trajectory_goal(const trajectory_msgs::msg::JointTrajectory & traj) {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Action server not available when trying to send trajectory");
      // still store as current so we can try later
      current_traj_ = traj;
      has_active_traj_ = true;
      return;
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory = traj;
    goal.goal_time_tolerance.sec = 0;
    goal.goal_time_tolerance.nanosec = 500000000;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle) {
        std_msgs::msg::String ack;
        if (!goal_handle) {
          ack.data = "REJECTED";
          RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        } else {
          ack.data = "ACCEPTED";
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
        ack_pub_->publish(ack);
        active_goal_handle_ = goal_handle;
      };

    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result) {
        std_msgs::msg::String out;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          out.data = "SUCCEEDED";
          RCLCPP_INFO(this->get_logger(), "Active goal SUCCEEDED");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          out.data = "CANCELED";
          RCLCPP_WARN(this->get_logger(), "Active goal CANCELED");
        } else {
          out.data = "ABORTED";
          RCLCPP_ERROR(this->get_logger(), "Active goal ABORTED/FAILED");
        }
        // publish result and clear active state
        result_pub_->publish(out);
        has_active_traj_ = false;
        active_goal_handle_.reset();
      };

    // send and record start time
    auto fut = action_client_->async_send_goal(goal, send_goal_options);
    // we don't block waiting for acceptance; but record the time we sent it
    active_goal_sent_time_ = this->now();
    active_goal_start_time_ = active_goal_sent_time_;
    has_active_traj_ = true;
    current_traj_ = traj;
  }

  // compute elapsed seconds since active_goal_start_time_
  double active_elapsed_seconds() const {
    if (!has_active_traj_) return 0.0;
    rclcpp::Time now = this->now();
    return (now - active_goal_start_time_).seconds();
  }

  // Build a new trajectory that starts at current joint positions (ordered to traj.joint_names),
  // and includes remaining points from `traj` after `elapsed` seconds, with times scaled by 1/speed_factor.
  trajectory_msgs::msg::JointTrajectory build_rescaled_remaining(const trajectory_msgs::msg::JointTrajectory & traj,
                                                                 double elapsed,
                                                                 double speed_factor)
  {
    trajectory_msgs::msg::JointTrajectory out;
    out.joint_names = traj.joint_names;

    // Point 0: current actual positions, at t=0
    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.positions.reserve(out.joint_names.size());
    for (const auto &jn : out.joint_names) {
      p0.positions.push_back(joint_pos_from_last_state(jn));
    }
    p0.time_from_start.sec = 0;
    p0.time_from_start.nanosec = 0;
    out.points.push_back(p0);

    if (speed_factor <= 0.0) {
      // paused — no remaining points appended
      return out;
    }

    double factor = 1.0 / std::max(1e-6, speed_factor);

    // For each original point, if its orig_time > elapsed + tiny_eps, include with new time = (orig_time - elapsed) * factor
    const double tiny_eps = 1e-4;
    for (const auto &pt : traj.points) {
      double orig_t = double(pt.time_from_start.sec) + double(pt.time_from_start.nanosec)/1e9;
      double remaining = orig_t - elapsed;
      if (remaining <= tiny_eps) continue;
      double new_t = remaining * factor;
      trajectory_msgs::msg::JointTrajectoryPoint np;
      np.positions = pt.positions;
      np.velocities = pt.velocities;
      np.accelerations = pt.accelerations;
      // set time_from_start relative to goal start (now)
      int64_t sec = static_cast<int64_t>(std::floor(new_t));
      int64_t nsec = static_cast<int64_t>((new_t - sec) * 1e9);
      if (nsec < 0) nsec = 0;
      np.time_from_start.sec = static_cast<int32_t>(sec);
      np.time_from_start.nanosec = static_cast<uint32_t>(nsec);
      out.points.push_back(np);
    }
    return out;
  }

  // -------------------------
  // Callbacks
  // -------------------------
  void on_override(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    // Replace any active trajectory with this one and send immediately.
    RCLCPP_INFO(this->get_logger(), "Received override trajectory with %zu points", msg->points.size());

    // cancel any active goal (best-effort)
    if (action_client_->action_server_is_ready()) {
      auto fut = action_client_->async_cancel_all_goals();
      std::this_thread::sleep_for(40ms); // allow cancel to propagate
    }

    // store and send new trajectory as-is (no queue)
    current_traj_ = *msg;
    has_active_traj_ = false; // will be set by send_trajectory_goal
    current_traj_ = build_rescaled_remaining(current_traj_, 0.0, speed_fraction_); // carry over existing speed
    send_trajectory_goal(current_traj_);
    // publish ack
    std_msgs::msg::String s; s.data = "ACCEPTED";
    ack_pub_->publish(s);
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // just store the latest joint state for building re-issued trajectories
    last_joint_state_ = msg;
  }

  void on_speed(const std_msgs::msg::Float32::SharedPtr msg) {
    double new_speed = std::clamp(static_cast<double>(msg->data), -1e6, 1.0); // accept <=0 as pause
    RCLCPP_INFO(this->get_logger(), "Received speed_command = %f", new_speed);

    // if no active traj, just update speed_fraction_ and return
    if (!has_active_traj_) {
      speed_fraction_ = new_speed;
      return;
    }

    // if speed <= 0 -> pause: cancel current goal and keep current_traj_ intact for resume
    if (new_speed <= 0.0) {
      RCLCPP_INFO(this->get_logger(), "Pausing execution (speed <= 0). Cancelling active goal.");
      speed_fraction_ = new_speed;
      if (action_client_->action_server_is_ready()) {
        auto fut = action_client_->async_cancel_all_goals();
        std::this_thread::sleep_for(40ms);
      }
      // leave current_traj_ intact so we can resume later
      has_active_traj_ = true;  // mark that we have a traj but not executing
      return;
    }

    // positive speed -> rescale remaining and reissue
    speed_fraction_ = new_speed;
    RCLCPP_INFO(this->get_logger(), "Rescaling remaining trajectory with speed %f", speed_fraction_);

    // compute elapsed time on the currently active goal
    double elapsed = active_elapsed_seconds();

    // cancel current goal first
    if (action_client_->action_server_is_ready()) {
      auto fut = action_client_->async_cancel_all_goals();
      std::this_thread::sleep_for(40ms);
    }

    // Build new trajectory: start at actual pose, append remaining points scaled
    if (!last_joint_state_) {
      RCLCPP_WARN(this->get_logger(), "No /joint_states received yet; cannot reissue remaining trajectory reliably.");
      // simply re-send original trajectory at new speed by constructing scaled version from start
      auto rescaled = build_rescaled_remaining(current_traj_, 0.0, speed_fraction_);
      send_trajectory_goal(rescaled);
      return;
    }

    // build and send
    auto new_traj = build_rescaled_remaining(current_traj_, elapsed, speed_fraction_);
    send_trajectory_goal(new_traj);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpeedControlNode>();
  // use single-threaded spinning
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
