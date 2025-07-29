#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <ros_gz_interfaces/srv/attach_detach.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <map>
#include <mutex>
#include <atomic>

namespace gripper_bridge {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gripper_bridge");

/* ========================================================================== */
/*                           CONFIGURATION MANAGER                           */
/* ========================================================================== */
class ConfigurationManager {
public:
    struct ObjectConfig {
        std::string id;
        std::string gz_link_name{"body"}; // Default fallback
    };

    struct TaskConfig {
        std::string object_id;
        std::string gz_link_name{"body"}; // Default fallback
    };

    explicit ConfigurationManager(const std::string& config_file_path) {
        loadConfiguration(config_file_path);
    }

    const std::vector<TaskConfig>& getTasks() const { return tasks_; }
    
    const ObjectConfig* getObject(const std::string& id) const {
        auto it = objects_.find(id);
        return (it != objects_.end()) ? &it->second : nullptr;
    }

    std::string getGzLinkName(const std::string& object_id) const {
        // First check if object exists in tasks (takes priority)
        for (const auto& task : tasks_) {
            if (task.object_id == object_id) {
                return task.gz_link_name;
            }
        }
        
        // Fallback to object configuration
        const auto* obj = getObject(object_id);
        return obj ? obj->gz_link_name : "body";
    }

private:
    void loadConfiguration(const std::string& config_file_path) {
        if (config_file_path.empty()) {
            RCLCPP_WARN(LOGGER, "No config file specified, using defaults");
            return;
        }

        try {
            YAML::Node config = YAML::LoadFile(config_file_path);
            loadObjects(config);
            loadTasks(config);
            
            RCLCPP_INFO(LOGGER, "Loaded configuration: %zu objects, %zu tasks", 
                       objects_.size(), tasks_.size());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(LOGGER, "Failed to load config file '%s': %s", 
                        config_file_path.c_str(), e.what());
        }
    }

    void loadObjects(const YAML::Node& config) {
        if (!config["objects"] || !config["objects"].IsMap()) {
            return;
        }

        for (auto it = config["objects"].begin(); it != config["objects"].end(); ++it) {
            ObjectConfig obj;
            obj.id = it->first.as<std::string>();
            
            // gz_link_name is optional at object level
            if (it->second["gz_link_name"]) {
                obj.gz_link_name = it->second["gz_link_name"].as<std::string>();
            }
            
            objects_[obj.id] = obj;
        }
    }

    void loadTasks(const YAML::Node& config) {
        if (!config["tasks"] || !config["tasks"].IsSequence()) {
            return;
        }

        for (const auto& task_node : config["tasks"]) {
            TaskConfig task;
            task.object_id = task_node["object"].as<std::string>();
            
            // gz_link_name can be specified at task level (overrides object level)
            if (task_node["gz_link_name"]) {
                task.gz_link_name = task_node["gz_link_name"].as<std::string>();
            } else {
                // Use object's gz_link_name if available
                const auto* obj = getObject(task.object_id);
                if (obj) {
                    task.gz_link_name = obj->gz_link_name;
                }
            }
            
            tasks_.push_back(task);
        }
    }

    std::map<std::string, ObjectConfig> objects_;
    std::vector<TaskConfig> tasks_;
};

/* ========================================================================== */
/*                           GAZEBO ATTACHMENT MANAGER                       */
/* ========================================================================== */
class GazeboAttachmentManager {
public:
    explicit GazeboAttachmentManager(rclcpp::Node::SharedPtr node) 
        : node_(node), service_timeout_(std::chrono::seconds(5)) {
        initializeService();
    }

    bool attach(const std::string& model_name, const std::string& link_name) {
        return performAttachDetach(model_name, link_name, "attach");
    }

    bool detach(const std::string& model_name, const std::string& link_name) {
        return performAttachDetach(model_name, link_name, "detach");
    }

    void setServiceTimeout(std::chrono::seconds timeout) {
        service_timeout_ = timeout;
    }

private:
    void initializeService() {
        const std::string service_name = "/payload/attach_detach";
        
        RCLCPP_INFO(LOGGER, "Connecting to Gazebo AttachDetach service: %s", service_name.c_str());
        
        attach_detach_client_ = node_->create_client<ros_gz_interfaces::srv::AttachDetach>(service_name);

        if (!attach_detach_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(LOGGER, "AttachDetach service not available, gazebo operations will fail");
        } else {
            RCLCPP_INFO(LOGGER, "Successfully connected to Gazebo AttachDetach service");
        }
    }

    bool performAttachDetach(const std::string& model_name, const std::string& link_name, 
                           const std::string& command) {
        auto request = std::make_shared<ros_gz_interfaces::srv::AttachDetach::Request>();
        request->child_model_name = model_name;
        request->child_link_name = link_name;
        request->command = command;

        RCLCPP_INFO(LOGGER, "Preparing Gazebo %s request - Model: %s, Link: %s", 
                   command.c_str(), model_name.c_str(), link_name.c_str());

        // Ensure service is available
        if (!attach_detach_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(LOGGER, "AttachDetach service not available at execution time");
            return false;
        }

        RCLCPP_INFO(LOGGER, "AttachDetach service available, sending %s request", command.c_str());
        auto future = attach_detach_client_->async_send_request(request);

        if (!future.valid()) {
            RCLCPP_WARN(LOGGER, "Failed to send AttachDetach request (invalid future)");
            return false;
        }

        if (future.wait_for(service_timeout_) == std::future_status::ready) {
            auto response = future.get();
            if (!response->success) {
                RCLCPP_WARN(LOGGER, "Gazebo %s failed: %s", command.c_str(), response->message.c_str());
                return false;
            } else {
                RCLCPP_INFO(LOGGER, "Gazebo %s successful", command.c_str());
                return true;
            }
        } else {
            RCLCPP_WARN(LOGGER, "Gazebo %s request timed out", command.c_str());
            return false;
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<ros_gz_interfaces::srv::AttachDetach>::SharedPtr attach_detach_client_;
    std::chrono::seconds service_timeout_;
};

/* ========================================================================== */
/*                           GRIPPER ACTION RELAY                            */
/* ========================================================================== */
class GripperActionRelay {
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<control_msgs::action::ParallelGripperCommand>;
    using GoalResult = control_msgs::action::ParallelGripperCommand::Result;

    GripperActionRelay(rclcpp::Node::SharedPtr node, 
                      const std::string& real_topic,
                      std::shared_ptr<GazeboAttachmentManager> gazebo_manager,
                      std::shared_ptr<ConfigurationManager> config_manager)
        : node_(node), 
          gazebo_manager_(gazebo_manager),
          config_manager_(config_manager),
          close_threshold_(0.15),
          current_target_index_(0) {
        
        initializeRealClient(real_topic);
    }

    void setCloseThreshold(double threshold) {
        close_threshold_ = threshold;
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const control_msgs::action::ParallelGripperCommand::Goal> goal) {
        
        RCLCPP_INFO(LOGGER, "Received gripper goal - Position: %f", 
                   goal->command.position.empty() ? 0.0 : goal->command.position[0]);
        
        if (goal->command.position.empty()) {
            RCLCPP_WARN(LOGGER, "Rejecting goal - empty position command");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_INFO(LOGGER, "Accepting gripper goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>) {
        RCLCPP_INFO(LOGGER, "Gripper goal cancellation requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(LOGGER, "Gripper goal accepted, starting execution in background thread");
        std::thread{&GripperActionRelay::relayGoal, this, goal_handle}.detach();
    }

private:
    void initializeRealClient(const std::string& real_topic) {
        RCLCPP_INFO(LOGGER, "Connecting to real gripper action server: %s", real_topic.c_str());
        
        real_client_ = rclcpp_action::create_client<control_msgs::action::ParallelGripperCommand>(
            node_, real_topic);

        if (!real_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_FATAL(LOGGER, "Real gripper action server %s not available", real_topic.c_str());
        } else {
            RCLCPP_INFO(LOGGER, "Successfully connected to real gripper action server");
        }
    }

    void relayGoal(std::shared_ptr<GoalHandle> goal_handle) {
        auto goal_msg = *(goal_handle->get_goal());
        RCLCPP_INFO(LOGGER, "Relaying gripper goal - Position: %f", goal_msg.command.position[0]);

        // Send to real controller
        RCLCPP_INFO(LOGGER, "Sending goal to real gripper controller...");
        auto send_future = real_client_->async_send_goal(goal_msg);
        
        if (send_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready || !send_future.get()) {
            RCLCPP_ERROR(LOGGER, "Failed to send goal to real gripper controller");
            goal_handle->abort(std::make_shared<GoalResult>());
            return;
        }

        RCLCPP_INFO(LOGGER, "Goal sent successfully, waiting for result...");
        auto result_future = real_client_->async_get_result(send_future.get());
        
        if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
            RCLCPP_ERROR(LOGGER, "Timeout waiting for gripper result");
            goal_handle->abort(std::make_shared<GoalResult>());
            return;
        }

        auto wrapped_result = result_future.get();
        RCLCPP_INFO(LOGGER, "Gripper result received - Reached goal: %s", 
                   wrapped_result.result->reached_goal ? "true" : "false");

        if (wrapped_result.result->reached_goal) {
            processGazeboAttachment(goal_msg.command.position[0]);
        }

        RCLCPP_INFO(LOGGER, "Gripper goal completed successfully");
        goal_handle->succeed(wrapped_result.result);
    }

    void processGazeboAttachment(double gripper_position) {
        bool is_closing = gripper_position > close_threshold_;
        
        RCLCPP_INFO(LOGGER, "Gripper reached goal - Position: %f, Threshold: %f, Closing: %s", 
                   gripper_position, close_threshold_, is_closing ? "true" : "false");

        std::string current_object;
        std::string current_link;
        
        if (getCurrentTarget(current_object, current_link)) {
            if (is_closing) {
                if (gazebo_manager_->attach(current_object, current_link)) {
                    // Successfully attached, move to next target for future operations
                    advanceToNextTarget();
                }
            } else {
                gazebo_manager_->detach(current_object, current_link);
                // Note: We don't advance target on detach as the object might be picked again
            }
        } else {
            RCLCPP_WARN(LOGGER, "No valid target object found for attachment/detachment");
        }
    }

    bool getCurrentTarget(std::string& object_id, std::string& link_name) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        
        const auto& tasks = config_manager_->getTasks();
        if (tasks.empty() || current_target_index_ >= tasks.size()) {
            return false;
        }

        const auto& current_task = tasks[current_target_index_];
        object_id = current_task.object_id;
        link_name = config_manager_->getGzLinkName(object_id);
        
        RCLCPP_INFO(LOGGER, "Current target: Object='%s', Link='%s', Index=%zu", 
                   object_id.c_str(), link_name.c_str(), current_target_index_);
        
        return true;
    }

    void advanceToNextTarget() {
        std::lock_guard<std::mutex> lock(target_mutex_);
        
        const auto& tasks = config_manager_->getTasks();
        if (current_target_index_ < tasks.size() - 1) {
            current_target_index_++;
            RCLCPP_INFO(LOGGER, "Advanced to next target index: %zu", current_target_index_);
        } else {
            RCLCPP_INFO(LOGGER, "Reached end of task list, staying at final target");
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SharedPtr real_client_;
    std::shared_ptr<GazeboAttachmentManager> gazebo_manager_;
    std::shared_ptr<ConfigurationManager> config_manager_;
    
    double close_threshold_;
    std::size_t current_target_index_;
    std::mutex target_mutex_;
};

/* ========================================================================== */
/*                           MAIN GRIPPER BRIDGE CLASS                       */
/* ========================================================================== */
class GripperBridge : public rclcpp::Node {
public:
    explicit GripperBridge() : Node("gripper_bridge") {
        RCLCPP_INFO(LOGGER, "Initializing GripperBridge node...");
        
        declareParameters();
        loadParameters();
    }

    void initialize() {
        initializeComponents();
        createActionServer();
        
        RCLCPP_INFO(LOGGER, "Bridge ready: MoveIt ⇢ %s ⇢ %s", 
                   dummy_topic_.c_str(), real_topic_.c_str());
    }

private:
    void declareParameters() {
        declare_parameter<std::string>("dummy_topic", "/dummy_gripper_controller/gripper_cmd");
        declare_parameter<std::string>("real_topic", "/robotiq_2f_85_gripper_controller/gripper_cmd");
        declare_parameter<std::string>("config_file", "");
        declare_parameter<double>("close_threshold", 0.15);
    }

    void loadParameters() {
        dummy_topic_ = get_parameter("dummy_topic").as_string();
        real_topic_ = get_parameter("real_topic").as_string();
        config_file_ = get_parameter("config_file").as_string();
        close_threshold_ = get_parameter("close_threshold").as_double();
        
        RCLCPP_INFO(LOGGER, "Parameters loaded - Dummy: %s, Real: %s, Config: %s, Threshold: %f", 
                   dummy_topic_.c_str(), real_topic_.c_str(), 
                   config_file_.c_str(), close_threshold_);
    }

    void initializeComponents() {
        // Initialize configuration manager
        config_manager_ = std::make_shared<ConfigurationManager>(config_file_);
        
        // Initialize Gazebo attachment manager
        gazebo_manager_ = std::make_shared<GazeboAttachmentManager>(shared_from_this());
        
        // Initialize gripper action relay
        action_relay_ = std::make_unique<GripperActionRelay>(
            shared_from_this(), real_topic_, gazebo_manager_, config_manager_);
        action_relay_->setCloseThreshold(close_threshold_);
    }

    void createActionServer() {
        RCLCPP_INFO(LOGGER, "Creating dummy gripper action server: %s", dummy_topic_.c_str());
        
        using namespace std::placeholders;
        dummy_server_ = rclcpp_action::create_server<control_msgs::action::ParallelGripperCommand>(
            this, dummy_topic_,
            std::bind(&GripperActionRelay::handleGoal, action_relay_.get(), _1, _2),
            std::bind(&GripperActionRelay::handleCancel, action_relay_.get(), _1),
            std::bind(&GripperActionRelay::handleAccepted, action_relay_.get(), _1));
        
        RCLCPP_INFO(LOGGER, "Dummy gripper action server created successfully");
    }

    // Parameters
    std::string dummy_topic_;
    std::string real_topic_;
    std::string config_file_;
    double close_threshold_;
    
    // Components
    std::shared_ptr<ConfigurationManager> config_manager_;
    std::shared_ptr<GazeboAttachmentManager> gazebo_manager_;
    std::unique_ptr<GripperActionRelay> action_relay_;
    
    // ROS interfaces
    rclcpp_action::Server<control_msgs::action::ParallelGripperCommand>::SharedPtr dummy_server_;
};

} // namespace gripper_bridge

/* ========================================================================== */
/*                                   MAIN                                    */
/* ========================================================================== */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gripper_bridge::GripperBridge>();
    
    // Initialize components after the shared_ptr is created
    node->initialize();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}