#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <ros_gz_interfaces/srv/attach_detach.hpp> 


class GripperBridge : public rclcpp::Node
{
public:

  GripperBridge() :
    Node("gripper_bridge")
  {
    RCLCPP_INFO(get_logger(), "Initializing GripperBridge node...");
    
    /* ---- parameters ---- */
    declare_parameter<std::string>("dummy_topic",
        "/dummy_gripper_controller/gripper_cmd");
    declare_parameter<std::string>("real_topic",
        "/robotiq_2f_85_gripper_controller/gripper_cmd");
    dummy_topic_ = get_parameter("dummy_topic").as_string();
    real_topic_  = get_parameter("real_topic").as_string();
    
    RCLCPP_INFO(get_logger(), "GripperBridge parameters - Dummy topic: %s, Real topic: %s", 
                dummy_topic_.c_str(), real_topic_.c_str());

    /* ---- real action client ---- */
    RCLCPP_INFO(get_logger(), "Connecting to real gripper action server: %s", real_topic_.c_str());
    real_client_ = rclcpp_action::create_client<control_msgs::action::ParallelGripperCommand>(this, real_topic_);
    if (!real_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_FATAL(get_logger(),
                   "Real gripper action server %s not available",
                   real_topic_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Successfully connected to real gripper action server");
    }

    /* ---- dummy action server ---- */
    RCLCPP_INFO(get_logger(), "Creating dummy gripper action server: %s", dummy_topic_.c_str());
    using namespace std::placeholders;
    dummy_server_ = rclcpp_action::create_server<control_msgs::action::ParallelGripperCommand>(
        this, dummy_topic_,
        std::bind(&GripperBridge::handle_goal,    this, _1, _2),
        std::bind(&GripperBridge::handle_cancel,  this, _1),
        std::bind(&GripperBridge::handle_accepted,this, _1));
    RCLCPP_INFO(get_logger(), "Dummy gripper action server created successfully");
    /* gz simulation plugin */
    RCLCPP_INFO(get_logger(), "Connecting to Gazebo AttachDetach service: /payload/attach_detach");
    attach_detach_client_ = create_client<ros_gz_interfaces::srv::AttachDetach>(
        "/payload/attach_detach");

    if (!attach_detach_client_->wait_for_service(std::chrono::seconds(3))) {
          RCLCPP_ERROR(get_logger(),
                      "AttachDetach service not available, gazebo-attach will fail");
        } else {
          RCLCPP_INFO(get_logger(), "Successfully connected to Gazebo AttachDetach service");
        }
 
    RCLCPP_INFO(get_logger(),
                "Bridge ready: MoveIt ⇢ %s ⇢ %s",
                dummy_topic_.c_str(), real_topic_.c_str());
  }

private:
  /* ---------- callbacks ---------- */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const control_msgs::action::ParallelGripperCommand::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received gripper goal - Position: %f", 
                goal->command.position.empty() ? 0.0 : goal->command.position[0]);
    
    if (goal->command.position.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal - empty position command");
      return rclcpp_action::GoalResponse::REJECT;
    } else {
      RCLCPP_INFO(get_logger(), "Accepting gripper goal");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>>)
  {
    RCLCPP_INFO(get_logger(), "Gripper goal cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> gh)
  {
    RCLCPP_INFO(get_logger(), "Gripper goal accepted, starting execution in background thread");
    std::thread{&GripperBridge::relay_goal, this, gh}.detach();
  }

  /* ---------- relay logic ---------- */
  void relay_goal(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>> gh)
  {
    auto goal_msg = *(gh->get_goal());        // copy as‑is
    RCLCPP_INFO(get_logger(), "Relaying gripper goal - Position: %f", goal_msg.command.position[0]);

    // send to real controller
    RCLCPP_INFO(get_logger(), "Sending goal to real gripper controller...");
    auto send_fut   = real_client_->async_send_goal(goal_msg);
    if (send_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready || !send_fut.get()) {
      RCLCPP_ERROR(get_logger(), "Failed to send goal to real gripper controller");
      gh->abort(std::make_shared<control_msgs::action::ParallelGripperCommand::Result>());
      return;
    }
    RCLCPP_INFO(get_logger(), "Goal sent successfully, waiting for result...");
    auto result_fut = real_client_->async_get_result(send_fut.get());
    if (result_fut.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for gripper result");
      gh->abort(std::make_shared<control_msgs::action::ParallelGripperCommand::Result>());
      return;
    }
auto wrapped = result_fut.get();
RCLCPP_INFO(get_logger(), "Gripper result received - Reached goal: %s", 
            wrapped.result->reached_goal ? "true" : "false");

if (wrapped.result->reached_goal) {
    const double CLOSE_THRESHOLD = 0.15;
    bool closed = goal_msg.command.position[0] > CLOSE_THRESHOLD;
    RCLCPP_INFO(get_logger(), "Gripper reached goal - Position: %f, Threshold: %f, Closed: %s", 
                goal_msg.command.position[0], CLOSE_THRESHOLD, closed ? "true" : "false");

    auto req = std::make_shared<ros_gz_interfaces::srv::AttachDetach::Request>();
    req->child_model_name = "can_1";
    req->child_link_name  = "body";
    req->command          = closed ? "attach" : "detach";
    
    RCLCPP_INFO(get_logger(), "Preparing Gazebo %s request - Model: %s, Link: %s", 
                req->command.c_str(), req->child_model_name.c_str(), req->child_link_name.c_str());

    // **ensure** the service is up *right now* before we call
    RCLCPP_INFO(get_logger(), "Checking AttachDetach service availability...");
    if (!attach_detach_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(),
                  "AttachDetach service not available at execution time!");
    } else {
      RCLCPP_INFO(get_logger(), "AttachDetach service available, sending %s request", req->command.c_str());
      auto attach_fut = attach_detach_client_->async_send_request(req);

      // guard the “no state” error
      if (!attach_fut.valid()) {
        RCLCPP_WARN(get_logger(),
                    "Failed to send AttachDetach request (invalid future)");
      } else {
        // timeout if it takes too long
        if (attach_fut.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
          auto res = attach_fut.get();
          if (!res->success) {
            RCLCPP_WARN(get_logger(),
                        "Gazebo %s failed: %s",
                        closed ? "attach" : "detach",
                        res->message.c_str());
          } else {
            RCLCPP_INFO(get_logger(), "Gazebo %s successful", closed ? "attach" : "detach");
          }
        } else {
          RCLCPP_WARN(get_logger(),
                      "Gazebo %s request timed out",
                      closed ? "attach" : "detach");
        }
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Gripper goal completed successfully");
  gh->succeed(wrapped.result);
}

  /* ---------- members ---------- */
  std::string dummy_topic_, real_topic_;
  rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SharedPtr  real_client_;
  rclcpp_action::Server<control_msgs::action::ParallelGripperCommand>::SharedPtr  dummy_server_;
  rclcpp::Client<ros_gz_interfaces::srv::AttachDetach>::SharedPtr attach_detach_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
