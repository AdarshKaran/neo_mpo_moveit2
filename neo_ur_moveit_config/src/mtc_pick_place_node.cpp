#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>

namespace mtc = moveit::task_constructor;
using GripperCommand = control_msgs::action::GripperCommand;

class MTCPickPlaceNode : public rclcpp::Node
{
public:
  MTCPickPlaceNode() : Node("mtc_pick_place_node")
  {
    declare_parameter("arm_group_name",   "ur_manipulator");
    declare_parameter("hand_group_name",  "gripper");
    declare_parameter("eef_name",         "endeffector");
    declare_parameter("hand_frame",       "robotiq_85_base_link");
    declare_parameter("target_object",    "small_cube");
    declare_parameter("table_reference_frame", "base_link");
    declare_parameter("place_pose_x", 0.5);
    declare_parameter("place_pose_y", 0.0);
    declare_parameter("place_pose_z", 0.05);
    declare_parameter("ready_pose",  "up");
    declare_parameter("open_pose",   "open");
    declare_parameter("close_pose",  "close");

    arm_group_name_        = get_parameter("arm_group_name").as_string();
    hand_group_name_       = get_parameter("hand_group_name").as_string();
    eef_name_              = get_parameter("eef_name").as_string();
    hand_frame_            = get_parameter("hand_frame").as_string();
    target_object_         = get_parameter("target_object").as_string();
    table_reference_frame_ = get_parameter("table_reference_frame").as_string();
    place_pose_x_          = get_parameter("place_pose_x").as_double();
    place_pose_y_          = get_parameter("place_pose_y").as_double();
    place_pose_z_          = get_parameter("place_pose_z").as_double();
    ready_pose_            = get_parameter("ready_pose").as_string();
    open_pose_             = get_parameter("open_pose").as_string();
    close_pose_            = get_parameter("close_pose").as_string();

    RCLCPP_INFO(get_logger(), "=== MTC Pick Place Node Starting ===");
  }

  void initializeTask()
  {
    // Create action server here where shared_from_this() is available
    gripper_action_server_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
        shared_from_this(),
        "gripper_action",
        std::bind(&MTCPickPlaceNode::handle_gripper_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MTCPickPlaceNode::handle_gripper_cancel, this, std::placeholders::_1),
        std::bind(&MTCPickPlaceNode::handle_gripper_accepted, this, std::placeholders::_1)
    );

    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/gripper_position_command", 10);

    setupPlanningScene();
    createTask();

    RCLCPP_INFO(get_logger(), "Custom gripper action server started on /gripper_action");
    RCLCPP_INFO(get_logger(), "Parameters loaded:");
    RCLCPP_INFO(get_logger(), "  target_object: %s", target_object_.c_str());
    RCLCPP_INFO(get_logger(), "  hand_group_name: %s", hand_group_name_.c_str());
    RCLCPP_INFO(get_logger(), "  hand_frame: %s", hand_frame_.c_str());

    try {
      task->init();
    } catch (const mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(get_logger(), e);
      return;
    }

    task->enableIntrospection();
    if (!task->plan(5)) {
      RCLCPP_ERROR(get_logger(), "Planning failed");
      return;
    }

    if (!task->solutions().empty())
    {
        is_executing_ = true;  // Set execution flag
        task->execute(*task->solutions().front());
        is_executing_ = false; // Reset flag
    }
    }

private:
  /* return every link that belongs to a joint-model-group */
  std::vector<std::string> allLinks(const std::string& group) const
  {
    const auto* jmg = task->getRobotModel()->getJointModelGroup(group);
    return jmg ? jmg->getLinkModelNamesWithCollisionGeometry()
               : std::vector<std::string>{};
  }

  /* TCP-to-object offset*/
  /* Vertical*/
  Eigen::Isometry3d graspOffset() const
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    
    // Simple 180° rotation around Y-axis to point gripper downward
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(0, 0, 0.15); //grasp height 15 cm above object
    return T;
  }

  /* Horizontal*/
//   Eigen::Isometry3d graspOffset() const
//   {
//     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//     Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()) *
//                            Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) *
//                            Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
//     T.linear()      = q.toRotationMatrix();
//     T.translation() = Eigen::Vector3d(0, 0, 0.10);   // 10 cm above object
//     return T;
//   }

  /* ---- planning-scene ---- */
  void setupPlanningScene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    rclcpp::sleep_for(std::chrono::seconds(1));

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = table_reference_frame_;
    table.id = "simple_table";

    shape_msgs::msg::SolidPrimitive tbl;
    tbl.type = tbl.BOX;
    tbl.dimensions = {0.8, 0.6, 0.7};

    geometry_msgs::msg::Pose tbl_pose;
    tbl_pose.orientation.w = 1.0;
    tbl_pose.position.x = 0.8;
    tbl_pose.position.y = 0.0;
    tbl_pose.position.z = 0.35;

    table.primitives.push_back(tbl);
    table.primitive_poses.push_back(tbl_pose);
    table.operation = table.ADD;

    moveit_msgs::msg::CollisionObject cube;
    cube.header.frame_id = table_reference_frame_;
    cube.id = target_object_;

    shape_msgs::msg::SolidPrimitive c;
    c.type = c.BOX;
    c.dimensions = {0.05, 0.05, 0.05};

    geometry_msgs::msg::Pose c_pose;
    c_pose.orientation.w = 1.0;
    c_pose.position.x = 0.8;
    c_pose.position.y = 0.0;
    c_pose.position.z = 0.725;

    cube.primitives.push_back(c);
    cube.primitive_poses.push_back(c_pose);
    cube.operation = cube.ADD;

    psi.applyCollisionObjects({table, cube});
    RCLCPP_INFO(get_logger(), "Added collision objects to planning scene");
  }

  /* ---- full MTC pipeline ---- */
  void createTask()
  {
    task = std::make_unique<mtc::Task>("pick_place_task");
    task->loadRobotModel(shared_from_this());

    task->setProperty("group",    arm_group_name_);
    task->setProperty("eef",      eef_name_);
    task->setProperty("ik_frame", hand_frame_);

    auto sampling_planner      =
        std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto interpolation_planner =
        std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner     =
        std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.1);
    cartesian_planner->setMaxAccelerationScalingFactor(0.1);
    cartesian_planner->setStepSize(0.01);

    /* ---------------- current state ---------------- */
    {
      auto current = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr_ = current.get();
      task->add(std::move(current));
    }

    /* ---------------- open hand ---------------- */
    {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("open hand");
        s->setCallback([this](const std::shared_ptr<planning_scene::PlanningScene>& scene, 
                              const mtc::PropertyMap& properties) {
            (void)scene; (void)properties;
            this->executeGripperCommand(0.0, "open hand");
        });
        task->add(std::move(s));
    }

    /* ---------------- move to pick ---------------- */
    {
      auto s = std::make_unique<mtc::stages::Connect>(
          "move to pick",
          mtc::stages::Connect::GroupPlannerVector{
              {arm_group_name_, sampling_planner}});
      s->setTimeout(5.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task->add(std::move(s));
    }

    mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator
    /* ========== PICK SERIAL CONTAINER ========== */
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task->properties().exposeTo(grasp->properties(),
                                  {"eef","group","ik_frame"});
      grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef","group","ik_frame"});

      /* approach */
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
            "approach object", cartesian_planner);
        s->properties().set("marker_ns","approach_object");
        s->properties().set("link", hand_frame_);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.01, 0.15);

        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = hand_frame_;
        v.vector.z = -1.0;
        s->setDirection(v);
        grasp->insert(std::move(s));
      }

      /* sample grasp pose + IK */
      {
        auto gen = std::make_unique<mtc::stages::GenerateGraspPose>(
            "generate grasp pose");
        gen->properties().configureInitFrom(mtc::Stage::PARENT);
        gen->properties().set("marker_ns","grasp_pose");
        gen->setPreGraspPose(open_pose_);
        gen->setObject(target_object_);
        gen->setAngleDelta(M_PI/12);
        gen->setMonitoredStage(current_state_ptr_);

        auto ik = std::make_unique<mtc::stages::ComputeIK>(
            "grasp pose IK", std::move(gen));
        ik->setMaxIKSolutions(8);
        ik->setMinSolutionDistance(1.0);
        ik->setIKFrame(graspOffset(), hand_frame_);
        ik->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef","group"});
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                            {"target_pose"});
        grasp->insert(std::move(ik));
      }

      /* close hand */
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("close hand");
        s->setCallback([this](const std::shared_ptr<planning_scene::PlanningScene>& scene, 
                              const mtc::PropertyMap& properties) {
            (void)scene; (void)properties;
            this->executeGripperCommand(0.5, "close hand");
        });
        grasp->insert(std::move(s));
      }

      /* attach */
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "attach object");
        s->attachObject(target_object_, hand_frame_);
        attach_object_stage = s.get();
        grasp->insert(std::move(s));
      }

      /* lift */
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
            "lift object", cartesian_planner);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.10, 0.30);
        s->setIKFrame(hand_frame_);
        s->properties().set("marker_ns","lift_object");

        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = table_reference_frame_;
        v.vector.z = 1.0;
        s->setDirection(v);
        grasp->insert(std::move(s));
      }
        {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "forbid collision (hand,support)");
        s->allowCollisions("simple_table", allLinks(hand_group_name_), false);
        grasp->insert(std::move(s));
        }
      task->add(std::move(grasp));
    }

    /* ---------------- move to place ---------------- */
    {
      auto s = std::make_unique<mtc::stages::Connect>(
          "move to place",
          mtc::stages::Connect::GroupPlannerVector{
              {arm_group_name_, sampling_planner}});
      s->setTimeout(5.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task->add(std::move(s));
    }

    /* ========== PLACE SERIAL CONTAINER ========== */
    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task->properties().exposeTo(place->properties(),
                                  {"eef","group","ik_frame"});
      place->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef","group","ik_frame"});

      /* allow collision (hand,object) */
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
        s->allowCollisions(target_object_, allLinks(hand_group_name_), true);
        place->insert(std::move(s));
      }

      /*lower object (approach place location) */
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
            "lower object", cartesian_planner);
        s->properties().set("marker_ns", "lower_object");
        s->properties().set("link", hand_frame_);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.03, 0.13);
        s->setIKFrame(hand_frame_);

        // Lower in the Z direction
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = table_reference_frame_;
        v.vector.z = -1.0;
        s->setDirection(v);
        place->insert(std::move(s));
      }

      /* sample place pose + IK */
      {
        auto gen = std::make_unique<mtc::stages::GeneratePlacePose>(
            "generate place pose");
        gen->properties().configureInitFrom(mtc::Stage::PARENT);
        gen->properties().set("marker_ns","place_pose");
        gen->setObject(target_object_);

        geometry_msgs::msg::PoseStamped tgt;
        tgt.header.frame_id = table_reference_frame_;
        tgt.pose.position.x = place_pose_x_;
        tgt.pose.position.y = place_pose_y_;
        tgt.pose.position.z = place_pose_z_ + 0.025;
        tgt.pose.orientation.w = 1.0;
        gen->setPose(tgt);
        
        gen->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

        auto ik = std::make_unique<mtc::stages::ComputeIK>(
            "place pose IK", std::move(gen));
        ik->setMaxIKSolutions(2);
        ik->setIKFrame(graspOffset(), hand_frame_);
        ik->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef","group"});
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                            {"target_pose"});
        place->insert(std::move(ik));
      }

      /* open hand */
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("open hand");
        s->setCallback([this](const std::shared_ptr<planning_scene::PlanningScene>& scene, 
                              const mtc::PropertyMap& properties) {
            (void)scene; (void)properties;
            this->executeGripperCommand(0.0, "open hand");
        });
        place->insert(std::move(s));
      }

      /* forbid collision hand↔object */
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "forbid collision (hand,object)");
        s->allowCollisions(target_object_, allLinks(hand_group_name_), false);
        place->insert(std::move(s));
      }

      /* detach */
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "detach object");
        s->detachObject(target_object_, hand_frame_);
        place->insert(std::move(s));
      }

      /* retreat */
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
            "retreat", cartesian_planner);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.10, 0.30);
        s->setIKFrame(hand_frame_);
        s->properties().set("marker_ns","retreat");

        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = hand_frame_;
        v.vector.z = -1.0;
        s->setDirection(v);
        place->insert(std::move(s));
      }

      task->add(std::move(place));
    }

    /* ---------------- return home ---------------- */
    {
      auto s = std::make_unique<mtc::stages::MoveTo>(
          "return home", interpolation_planner);
      s->setGroup(arm_group_name_);
      s->setGoal(ready_pose_);
      task->add(std::move(s));
    }
  }

  /* ----------------------- members ----------------------- */
  std::unique_ptr<mtc::Task> task;
  mtc::Stage*                current_state_ptr_{nullptr};

  // Gripper action server
  rclcpp_action::Server<control_msgs::action::GripperCommand>::SharedPtr gripper_action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;

  std::string arm_group_name_, hand_group_name_, eef_name_, hand_frame_;
  std::string target_object_, table_reference_frame_;
  double       place_pose_x_, place_pose_y_, place_pose_z_;
  std::string ready_pose_, open_pose_, close_pose_;

  bool is_executing_ = false;

  rclcpp_action::GoalResponse handle_gripper_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received gripper goal: position=%.3f", goal->command.position);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_gripper_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received gripper cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_gripper_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing gripper command (custom controller)");
    
    auto gripper_msg = std_msgs::msg::Float64();
    gripper_msg.data = goal_handle->get_goal()->command.position;
    gripper_pub_->publish(gripper_msg);
    
    // Simulate execution time
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Complete the action
    auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
    result->position = goal_handle->get_goal()->command.position;
    result->reached_goal = true;
    
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Gripper command completed");
  }

  void executeGripperCommand(double position, const std::string& stage_name) {
    if (!is_executing_) {
        return; // Skip during planning
    }
    
    RCLCPP_INFO(get_logger(), "EXECUTING gripper command: %.3f (stage: %s)", 
                position, stage_name.c_str());
    
    auto gripper_msg = std_msgs::msg::Float64();
    gripper_msg.data = position;
    gripper_pub_->publish(gripper_msg);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

};

/* ----------------------------- main ----------------------------- */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MTCPickPlaceNode>();
  node->initializeTask();                            

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}