// speed_control_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

class SpeedControlNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  SpeedControlNode()
  : Node("speed_control_node"),
    speed_fraction_(1.0),
    has_target_(false),
    has_active_goal_(false),
    min_time_between_points_(0.02),   // avoid tiny dt that causes jumps
    base_time_between_points_(1.0),   // tunable: base tempo (seconds) per segment at speed=1.0 (bigger is slower)
    points_per_radian_(4),            // tunable: how many interpolation points per radian of joint-space motion (bigger is slower)
    max_points_(100)                  // safety cap to avoid huge trajectories
  {
    // joints used by UR5 typical ordering used earlier
    joint_names_ = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};

    override_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/override_trajectory", 10,
      std::bind(&SpeedControlNode::on_override, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 100,
      std::bind(&SpeedControlNode::on_joint_state, this, std::placeholders::_1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/speed_command", 10,
      std::bind(&SpeedControlNode::on_speed, this, std::placeholders::_1));

    ack_pub_ = this->create_publisher<std_msgs::msg::String>("/override_ack", 10);
    result_pub_ = this->create_publisher<std_msgs::msg::String>("/override_result", 10);

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "SpeedControlNode ready");
  }

private:
  // ROS interfaces
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr override_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  // simple state
  std::vector<std::string> joint_names_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  double speed_fraction_;                // 0..1 (0 => pause)
  bool has_target_;
  std::vector<double> target_positions_; // final destination (last point of override)
  bool has_active_goal_;
  std::shared_ptr<GoalHandle> active_goal_handle_;
  trajectory_msgs::msg::JointTrajectory original_traj_;

  // timing params
  double base_time_between_points_;
  double min_time_between_points_;
  double points_per_radian_;
  int max_points_;

  // ----------------------------
  // Utilities
  // ----------------------------
  static double normalize_angle(double a) {
    const double two_pi = 2.0 * M_PI;
    a = std::fmod(a + M_PI, two_pi);
    if (a < 0) a += two_pi;
    return a - M_PI;
  }

  // return current positions in the order of joint_names_; if joint_states missing, return zeros
  std::vector<double> get_current_positions_ordered() {
    std::vector<double> out(joint_names_.size(), 0.0);
    if (!last_joint_state_) return out;
    for (size_t i=0; i<joint_names_.size(); ++i) {
      const auto &jn = joint_names_[i];
      for (size_t k=0; k<last_joint_state_->name.size(); ++k) {
        if (last_joint_state_->name[k] == jn) {
          out[i] = normalize_angle(last_joint_state_->position[k]);
          break;
        }
      }
    }
    return out;
  }

  size_t dist_to_num_pts(std::vector<double>& end_positions) {
    std::vector<double> start = this->get_current_positions_ordered();

    // compute joint-space L2 distance between start and target
    double dist = 0.0;
    size_t n = std::min(start.size(), end_positions.size());
    for (size_t k = 0; k < n; ++k) {
      double d = end_positions[k] - start[k];
      dist += d * d;
    }
    dist = std::sqrt(dist);

    // compute points proportional to distance (at least 1), clamp to max_points_
    int num_pts = std::max(1, static_cast<int>(std::ceil(points_per_radian_ * dist)));
    num_pts = std::min(num_pts, max_points_);

    return num_pts;
  }

  void cancel_or_resend(bool resend) {
  if (action_client_->action_server_is_ready()) {
    action_client_->async_cancel_all_goals(
      [this, resend](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
        if (!response) {
          RCLCPP_ERROR(this->get_logger(), "Cancellation response was null.");
          return;
        }

        if (response->return_code != action_msgs::srv::CancelGoal_Response::ERROR_NONE) {
          RCLCPP_WARN(this->get_logger(), "Cancellation request failed. Will need new speed request.");
          return;
        }

        RCLCPP_INFO(this->get_logger(), "All goals accepted for cancellation.");
        // ack
        std_msgs::msg::String s; s.data = "ACCEPTED";
        ack_pub_->publish(s);
        if (!resend) {
          return;
        }
        // build a fresh trajectory: current -> target, using base_time_between_points_ scaled by speed_fraction_
        std::vector<double> start = this->get_current_positions_ordered();
        size_t num_pts = dist_to_num_pts(target_positions_);
        double tbp = base_time_between_points_ / std::max(1e-6, speed_fraction_);
        trajectory_msgs::msg::JointTrajectory traj = this->generate_trajectory_segment(start, target_positions_, num_pts, tbp);

        // send it
        this->send_trajectory_goal(traj);
      }
    );
  } else {
    RCLCPP_WARN(this->get_logger(), "Action server not ready, cannot send cancel request.");
  }
}

  // cancel active goal (best-effort) and wait briefly for cancellation to propagate
  /*
  void cancel_or_resend() {
    if (!action_client_) return;
    if (active_goal_handle_) {
      // wait short time for cancel to process
      auto fut = action_client_->async_cancel_all_goals();
      std::this_thread::sleep_for(500ms);
      active_goal_handle_.reset();
      has_active_goal_ = false;
      std::this_thread::sleep_for(20ms);
      return;
    }
    // fallback
    auto fut2 = action_client_->async_cancel_all_goals();
    std::this_thread::sleep_for(500ms);
    has_active_goal_ = false;
    std::this_thread::sleep_for(20ms);
  }
  */

  // adapted from publish_trajectory_node.cpp
  trajectory_msgs::msg::JointTrajectory generate_trajectory_segment(
        const std::vector<double>& start_config,
        const std::vector<double>& end_config,
        size_t num_points, double tbp)
    {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = joint_names_;

        // Total interpolation time (N points * tbp)
        double total_time = (num_points * tbp) == 0 ? 1 : num_points * tbp;

        for (int i = 0; i <= num_points; i++)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            double t = i * tbp;

            for (size_t j = 0; j < start_config.size(); j++)
            {
                double interpolated_position = start_config[j] + (t / total_time) * (end_config[j] - start_config[j]);
                point.positions.push_back(interpolated_position);
            }
            RCLCPP_INFO(this->get_logger(), "Trajectory point %d: %f %f %f %f %f %f", i, point.positions[0], point.positions[1], point.positions[2], point.positions[3], point.positions[4], point.positions[5]);

            point.time_from_start = rclcpp::Duration::from_seconds(t);
            traj_msg.points.push_back(point);
        }

        return traj_msg;
    }

  // send a trajectory as an action goal (simple)
  void send_trajectory_goal(const trajectory_msgs::msg::JointTrajectory & traj) {
    if (!action_client_) return;

    // ensure server available (best-effort)
    if (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Action server not available right now; storing target and will try later.");
      // store so on future speed/override we'll resend
      return;
    }

    // Build goal
    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory = traj;
    goal.goal_time_tolerance.sec = 0;
    goal.goal_time_tolerance.nanosec = 500000000;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    // when server accepts, store handle
    send_goal_options.goal_response_callback =
      [this](const std::shared_ptr<GoalHandle> & goal_handle) {
        std_msgs::msg::String ack;
        if (!goal_handle) {
          ack.data = "REJECTED";
          RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        } else {
          ack.data = "ACCEPTED";
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
          active_goal_handle_ = goal_handle;
          has_active_goal_ = true;
        }
        ack_pub_->publish(ack);
      };

    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result) {
        std_msgs::msg::String out;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          out.data = "SUCCEEDED";
          RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          out.data = "CANCELED";
          RCLCPP_WARN(this->get_logger(), "Goal CANCELED");
        } else {
          out.data = "ABORTED";
          RCLCPP_ERROR(this->get_logger(), "Goal ABORTED");
        }
        result_pub_->publish(out);
        active_goal_handle_.reset();
        has_active_goal_ = false;
      };

    action_client_->async_send_goal(goal, send_goal_options);
  }

  // ----------------------------
  // Callbacks
  // ----------------------------
  void on_override(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    // new final target = last point of msg
    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received override with no points - ignoring");
      return;
    }
    // store authoritative original trajectory for simple remaining-point counting
    original_traj_ = *msg;

    // Grab final positions (last point)
    target_positions_.clear();
    target_positions_ = msg->points.back().positions;

    // Store that we have a target
    has_target_ = true;

    // Cancel any running goal and then compute a new traj from current pose -> target
    cancel_or_resend(true);

    // If paused (speed <= 0), do not send; just store target
    if (speed_fraction_ <= 0.0) {
      RCLCPP_INFO(this->get_logger(), "Override received but node is paused (speed <= 0). Target stored.");
      // ack anyway
      std_msgs::msg::String s; s.data = "ACCEPTED";
      ack_pub_->publish(s);
      return;
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
    last_joint_state_ = msg;
  }

  void on_speed(const std_msgs::msg::Float32::SharedPtr msg) {
    double new_speed = std::clamp(static_cast<double>(msg->data), 0.0, 1.0); // <=0 => pause
    RCLCPP_INFO(this->get_logger(), "Received speed_command = %f", new_speed);

    // If no active target, just update speed and return
    // If no update needed, do same
    if (!has_target_ || std::abs(speed_fraction_ - new_speed) <= 1e-6) { // arbitrary small margin for float comps
      speed_fraction_ = new_speed;
      return;
    }

    // Pause case: cancel running goal and do not send another
    if (new_speed <= 0.0) {
      RCLCPP_INFO(this->get_logger(), "Pausing execution (speed <= 0). Cancelling active goal.");
      speed_fraction_ = new_speed;
      cancel_or_resend(false);
      return;
    }

    // Positive speed: cancel active goal and re-synthesize trajectory from current pose -> same target
    speed_fraction_ = new_speed;
    RCLCPP_INFO(this->get_logger(), "Re-issuing trajectory to same target at new speed %f", speed_fraction_);

    // cancel and wait shortly to ensure old goal stops before sending new
    cancel_or_resend(true);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpeedControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
