// speed_control_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <queue>
#include <mutex>
#include <cmath>

using namespace std::chrono_literals;

class SpeedControlNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  SpeedControlNode()
  : Node("speed_control_node"),
    speed_fraction_(1.0),
    max_queue_size_(10),
    sending_goal_(false)
  {
    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/override_trajectory", 10, std::bind(&SpeedControlNode::traj_cb, this, std::placeholders::_1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/speed_command", 10, std::bind(&SpeedControlNode::speed_cb, this, std::placeholders::_1));

    ack_pub_ = this->create_publisher<std_msgs::msg::String>("/override_ack", 10);
    result_pub_ = this->create_publisher<std_msgs::msg::String>("/override_result", 10);

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    while (!action_client_->wait_for_action_server(3s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
    }
    
    RCLCPP_INFO(this->get_logger(), "SpeedControlNode ready");
  }

private:
  // Subscriptions / publishers / clients
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;

  // Internal queue + state
  std::mutex mutex_;
  std::queue<trajectory_msgs::msg::JointTrajectory> queue_;
  double speed_fraction_;
  const size_t max_queue_size_;
  bool sending_goal_;

  // speed handler
  void speed_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_fraction_ = std::max(0.0f, std::min(1.0f, msg->data));
    RCLCPP_INFO(this->get_logger(), "Speed fraction set to: %f", speed_fraction_);

    if (speed_fraction_ <= 0.0) {
      // hard stop: cancel and stop controller
      RCLCPP_WARN(this->get_logger(), "speed_fraction == 0 -> cancelling goals and stopping controller");
      cancel_all_goals();
      stop_controller();
    } else {
      // ensure controller running
      start_controller();
      // if nothing is sending but we have queued items, start sending
      if (!sending_goal_ && !queue_.empty()) {
        // launch send_next_from_queue outside of lock
        lock.~lock_guard();
        send_next_from_queue();
      }
    }
  }

  // trajectory subscription
  void traj_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr traj_msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= max_queue_size_) {
        std_msgs::msg::String nack;
        nack.data = "NACK: queue full";
        ack_pub_->publish(nack);
        RCLCPP_WARN(this->get_logger(), "Incoming trajectory rejected: queue full");
        return;
      }
      queue_.push(*traj_msg);
    }

    // ack that we accepted it into the queue
    std_msgs::msg::String ack;
    ack.data = "ACCEPTED";
    ack_pub_->publish(ack);
    RCLCPP_INFO(this->get_logger(), "Trajectory queued (current queue size unknown to publisher)");

    // If no goal currently running, start sending now
    bool should_start = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!sending_goal_) should_start = true;
    }
    if (should_start) {
      send_next_from_queue();
    }
  }

  // cancel all goals helper
  void cancel_all_goals()
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_INFO(this->get_logger(), "Action server not available");
      return;
    }
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
  }

  // start controller helper
  void start_controller()
  {
    if (!switch_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "switch_controller not available to start controller");
      return;
    }
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = {"scaled_joint_trajectory_controller"};
    req->deactivate_controllers = {};
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    auto fut = switch_client_->async_send_request(req, 
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        auto res = future.get();
        if (res->ok)
        {
            RCLCPP_INFO(this->get_logger(), "Requested start of scaled_joint_trajectory_controller");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to request start of controller");
        }
      }
    );
  }

  // stop controller helper
  void stop_controller()
  {
    if (!switch_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "switch_controller not available to stop controller");
      return;
    }
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->deactivate_controllers = {"scaled_joint_trajectory_controller"};
    req->activate_controllers = {};
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    auto fut = switch_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Requested stop of scaled_joint_trajectory_controller");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to request stop of controller");
    }
  }

  // take next queued trajectory, scale it and send as an action goal
  void send_next_from_queue()
  {
    trajectory_msgs::msg::JointTrajectory traj;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.empty() || sending_goal_) {
        return;
      }
      traj = queue_.front();
      queue_.pop();
      sending_goal_ = true;
    }

    // check speed fraction
    double local_speed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_speed = speed_fraction_;
    }
    if (local_speed <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Not sending queued trajectory because speed_fraction == 0");
      std::lock_guard<std::mutex> lock(mutex_);
      sending_goal_ = false;
      return;
    }

    auto scaled = scale_trajectory(traj, local_speed);

    // wait for server
    if (!action_client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      std::lock_guard<std::mutex> lock(mutex_);
      // push back so we don't lose it
      queue_.push(traj);
      sending_goal_ = false;
      return;
    }

    // build goal
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory = scaled;
    goal_msg.goal_time_tolerance.nanosec = 500000000;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    // optional: publish ack when accepted by action server
    send_goal_options.goal_response_callback =
      [this](const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle) {
        std_msgs::msg::String ack;
        if (!goal_handle) 
        {
                ack.data = "REJECTED_BY_ACTION_SERVER";
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
        }
        else
        {
                ack.data = "SENT_TO_ACTION_SERVER";
                RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result");
        }
        ack_pub_->publish(ack);
      };


    send_goal_options.result_callback =
        [this](const GoalHandleFollowJointTrajectory::WrappedResult &result) {
            std_msgs::msg::String out;
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                out.data = "SUCCEEDED";
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                out.data = "ABORTED";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                out.data = "CANCELED";
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                out.data = "UNKNOWN";
                break;
            }

        // publish result to origin
        result_pub_->publish(out);

        // mark done and trigger next item
        {
          std::lock_guard<std::mutex> lock(mutex_);
          sending_goal_ = false;
        }
        // dispatch next after a tiny delay to avoid recursion inside action callback
        auto timer = this->create_wall_timer(10ms, [this]() {
          this->send_next_from_queue();
        });
        // timer will be destroyed after firing (it is captured by node until callback runs)
      };

    RCLCPP_INFO(this->get_logger(), "Sending scaled trajectory (speed_fraction=%f)", local_speed);
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // scaling helper
  trajectory_msgs::msg::JointTrajectory scale_trajectory(const trajectory_msgs::msg::JointTrajectory & in, double speed_fraction)
  {
    trajectory_msgs::msg::JointTrajectory out;
    out.joint_names = in.joint_names;
    double factor = 1.0 / std::max(1e-6, speed_fraction);
    for (const auto & p : in.points) {
      trajectory_msgs::msg::JointTrajectoryPoint np;
      np.positions = p.positions;
      np.velocities = p.velocities;
      np.accelerations = p.accelerations;
      double secs = p.time_from_start.sec + p.time_from_start.nanosec / 1e9;
      double scaled = secs * factor;
      np.time_from_start.sec = static_cast<int32_t>(std::floor(scaled));
      np.time_from_start.nanosec = static_cast<uint32_t>((scaled - std::floor(scaled)) * 1e9);
      out.points.push_back(np);
    }
    return out;
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
