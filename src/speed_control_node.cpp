#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <controller_manager_msgs/srv/switch_controller.hpp>

using namespace std::chrono_literals;

class SpeedControlNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  SpeedControlNode()
  : Node("speed_control_node"), speed_fraction_(1.0)
  {
    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/override_trajectory", 10, std::bind(&SpeedControlNode::traj_cb, this, std::placeholders::_1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/speed_command", 10, std::bind(&SpeedControlNode::speed_cb, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    while (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_WARN(this->get_logger(), "FollowJointTrajectory action server not available yet.");
    }
  }

private:
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  double speed_fraction_;
  std::mutex mutex_;

  void speed_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_fraction_ = std::max(0.0f, std::min(1.0f, msg->data));
    RCLCPP_INFO(this->get_logger(), "Speed fraction set to: %f", speed_fraction_);
  }

  void traj_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr traj_msg)
  {
    // Cancel existing goals
    auto f = action_client_->async_cancel_all_goals(
      [this](std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>::CancelResponse> cancel_response) {
        if (cancel_response && cancel_response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE) {
          RCLCPP_INFO(this->get_logger(), "Successfully canceled all goals");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to cancel goals");
        }
      }
    );

    double local_speed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_speed = speed_fraction_;
    }

    if (local_speed <= 0.0) {
      // treat as hard stop: stop controller
      RCLCPP_WARN(this->get_logger(), "speed_fraction==0 -> stopping controller instead of sending traj");
      stop_controller();
      return;
    }

    // build scaled trajectory
    auto scaled_traj = scale_trajectory(*traj_msg, local_speed);

    // wait for server
    if (!action_client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available to send scaled trajectory");
      return;
    }

    // create goal
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory = scaled_traj;
    goal_msg.goal_time_tolerance.nanosec = 500000000;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle) {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result");
            }
        };

    send_goal_options.result_callback =
        [this](const GoalHandleFollowJointTrajectory::WrappedResult &result) {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
            }
        };

    RCLCPP_INFO(this->get_logger(), "Sending scaled trajectory (speed_fraction=%f)", local_speed);
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  trajectory_msgs::msg::JointTrajectory scale_trajectory(const trajectory_msgs::msg::JointTrajectory & in, double speed_fraction)
  {
    trajectory_msgs::msg::JointTrajectory out;
    out.joint_names = in.joint_names;
    double factor = 1.0 / std::max(1e-6, speed_fraction); // speed_fraction 0.5 -> factor 2.0
    for (const auto & p : in.points) {
      trajectory_msgs::msg::JointTrajectoryPoint np;
      np.positions = p.positions;
      np.velocities = p.velocities;
      np.accelerations = p.accelerations;
      // scale time_from_start
      double secs = p.time_from_start.sec + p.time_from_start.nanosec / 1e9;
      double scaled = secs * factor;
      np.time_from_start.sec = static_cast<int32_t>(floor(scaled));
      np.time_from_start.nanosec = static_cast<uint32_t>((scaled - floor(scaled)) * 1e9);
      out.points.push_back(np);
    }
    return out;
  }

  void stop_controller()
  {
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->deactivate_controllers = {"scaled_joint_trajectory_controller"};
    req->activate_controllers = {};
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    auto f = switch_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), f, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Stopped scaled_joint_trajectory_controller");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to stop controller");
    }
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
