#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<moveit/task_constructor/solvers/pipeline_planner.h>)
  #include <moveit/task_constructor/solvers/pipeline_planner.h>
#else
  #include <moveit/task_constructor/solvers/planner_interface.h>
#endif
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>


#include <Eigen/Geometry>
#include <thread>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_node");
namespace mtc = moveit::task_constructor;

/* ========================================================================== */
/*                           CLASS DECLARATION                                */
/* ========================================================================== */
class MTCPickPlaceNode
{
public:
  explicit MTCPickPlaceNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void doTask();

private:
  /* ----- helpers & members ------------------------------------------------ */
  mtc::Task createTask();
  Eigen::Isometry3d graspOffset() const;
  std::vector<std::string> allLinks(const std::string& group,
                                    const mtc::Task& task) const;

  /* ---------------- data -------------------------------------------------- */
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;

  std::string arm_group_name_, hand_group_name_, eef_name_, hand_frame_;
  std::string target_object_, table_reference_frame_;
  double place_pose_x_{}, place_pose_y_{}, place_pose_z_{};
  std::string ready_pose_, open_pose_, close_pose_;

};

/* ========================================================================== */
/*                           CONSTRUCTOR                                      */
/* ========================================================================== */
MTCPickPlaceNode::MTCPickPlaceNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_pick_place_node", options) }
{
  /* ---------------- parameters ---------------- */
  node_->declare_parameter("arm_group_name",   "ur_manipulator");
  node_->declare_parameter("hand_group_name",  "gripper");
  node_->declare_parameter("eef_name",         "endeffector");
  node_->declare_parameter("hand_frame",       "robotiq_85_base_link");
  node_->declare_parameter("target_object",    "small_cube");
  node_->declare_parameter("table_reference_frame", "base_link");
  node_->declare_parameter("place_pose_x", 0.5);
  node_->declare_parameter("place_pose_y", 0.0);
  node_->declare_parameter("place_pose_z", 0.05);
  node_->declare_parameter("ready_pose",  "up");
  node_->declare_parameter("open_pose",   "open");
  node_->declare_parameter("close_pose",  "close");

  arm_group_name_        = node_->get_parameter("arm_group_name").as_string();
  hand_group_name_       = node_->get_parameter("hand_group_name").as_string();
  eef_name_              = node_->get_parameter("eef_name").as_string();
  hand_frame_            = node_->get_parameter("hand_frame").as_string();
  target_object_         = node_->get_parameter("target_object").as_string();
  table_reference_frame_ = node_->get_parameter("table_reference_frame").as_string();
  place_pose_x_          = node_->get_parameter("place_pose_x").as_double();
  place_pose_y_          = node_->get_parameter("place_pose_y").as_double();
  place_pose_z_          = node_->get_parameter("place_pose_z").as_double();
  ready_pose_            = node_->get_parameter("ready_pose").as_string();
  open_pose_             = node_->get_parameter("open_pose").as_string();
  close_pose_            = node_->get_parameter("close_pose").as_string();

  RCLCPP_INFO(LOGGER, "MTC Pick Place Node Starting");

}

/* ========================================================================== */
/*                   MTC PIPELINE CODE                                        */
/* ========================================================================== */

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCPickPlaceNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

/* ---- planning-scene ---------------------------------- */
void MTCPickPlaceNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  rclcpp::sleep_for(std::chrono::seconds(1));

  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = table_reference_frame_;
  table.id = "simple_table";
  shape_msgs::msg::SolidPrimitive tbl;
  tbl.type = tbl.BOX; tbl.dimensions = {0.8, 0.6, 0.7};
  geometry_msgs::msg::Pose tbl_pose;
  tbl_pose.orientation.w = 1.0;
  tbl_pose.position.x = 0.8; tbl_pose.position.z = 0.35;
  table.primitives.push_back(tbl);
  table.primitive_poses.push_back(tbl_pose);
  table.operation = table.ADD;

  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = table_reference_frame_;
  cube.id = target_object_;
  shape_msgs::msg::SolidPrimitive c; c.type = c.BOX;
  c.dimensions = {0.05, 0.05, 0.05};
  geometry_msgs::msg::Pose c_pose;
  c_pose.orientation.w = 1.0;
  c_pose.position.x = 0.8; c_pose.position.z = 0.725;
  cube.primitives.push_back(c);
  cube.primitive_poses.push_back(c_pose);
  cube.operation = cube.ADD;

  psi.applyCollisionObjects({table, cube});
  RCLCPP_INFO(LOGGER, "Added collision objects to planning scene");
}

/* ---------- graspOffset, allLinks--------------------------- */
Eigen::Isometry3d MTCPickPlaceNode::graspOffset() const
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(0, 0, 0.15);
  return T;
}
std::vector<std::string>
MTCPickPlaceNode::allLinks(const std::string& group, const mtc::Task& task) const
{
  const auto* jmg = task.getRobotModel()->getJointModelGroup(group);
  return jmg ? jmg->getLinkModelNamesWithCollisionGeometry() : std::vector<std::string>{};
}

/* ---- full MTC pipeline ---- */
  
mtc::Task MTCPickPlaceNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("pick_place_task");
  task.loadRobotModel(node_);

  task.setProperty("group",    arm_group_name_);
  task.setProperty("eef",      eef_name_);
  task.setProperty("ik_frame", hand_frame_);

  // Disable warnings for this line
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

    /* ---------------- current state ---------------- */
    {
      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = stage_state_current.get();
      task.add(std::move(stage_state_current));
    }

    auto sampling_planner      =
        std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner =
        std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner     =
        std::make_shared<mtc::solvers::CartesianPath>();

    // Tune these for horizontal speed
    sampling_planner->setMaxVelocityScalingFactor(0.1);
    sampling_planner->setMaxAccelerationScalingFactor(0.1);
    // Tune these for vertical speed
    cartesian_planner->setMaxVelocityScalingFactor(0.1);
    cartesian_planner->setMaxAccelerationScalingFactor(0.1);
    cartesian_planner->setStepSize(0.01);

    // return to home speed
    interpolation_planner->setMaxVelocityScalingFactor(0.2);
    interpolation_planner->setMaxAccelerationScalingFactor(0.2);


    /* ---------------- open hand ---------------- */
    {
      auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name_);
      stage_open_hand->setGoal(open_pose_);
      task.add(std::move(stage_open_hand));
    }

    /* ---------------- move to pick ---------------- */
    {
      auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
          "move to pick",
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
      stage_move_to_pick->setTimeout(5.0);
      stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_pick));
    }

    mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator


    /* ========== PICK SERIAL CONTAINER ========== */
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
      grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                            { "eef", "group", "ik_frame" });

      /* approach */
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
        stage->properties().set("marker_ns", "approach_object");
        stage->properties().set("link", hand_frame_);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.01, 0.15);

        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame_;
        vec.vector.z = -1.0; // or -1?
        stage->setDirection(vec);
        grasp->insert(std::move(stage));

      }

      /* sample grasp pose + IK */
      {

        auto stage = 
        std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose(open_pose_);
        stage->setObject(target_object_);
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // Compute IK
        auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(graspOffset(), hand_frame_);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));

      }

      /* allow collision */
      {
        auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
        stage->allowCollisions(target_object_,
                              task.getRobotModel()
                                  ->getJointModelGroup(hand_group_name_)
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              true);
        grasp->insert(std::move(stage));
      }

      /* close hand */
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        stage->setGroup(hand_group_name_);
        stage->setGoal(close_pose_);
        grasp->insert(std::move(stage));
      }

      /* attach */
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        stage->attachObject(target_object_, hand_frame_);
        stage->allowCollisions(target_object_, {"simple_table"}, true);
        attach_object_stage = stage.get();
        grasp->insert(std::move(stage));
      }

      /* lift */
      {
        
        auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.15, 0.4);  // Increased from 0.1, 0.3
        stage->setIKFrame(hand_frame_);
        stage->properties().set("marker_ns", "lift_object");

        // Set upward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = table_reference_frame_;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }
        {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "forbid collision (hand,support)");
        s->allowCollisions("simple_table", allLinks(hand_group_name_, task), false);
        grasp->insert(std::move(s));
        }
      task.add(std::move(grasp));
    }

    /* ---------------- move to place ---------------- */
    {
      auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
          "move to place",
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
      stage_move_to_place->setTimeout(5.0);
      stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_place));
    }

    /* ========== PLACE SERIAL CONTAINER ========== */
    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      // /* lower object */
      // {
      //   auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      //   stage->properties().set("marker_ns", "lower_object");
      //   stage->properties().set("link", hand_frame_);
      //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      //   stage->setMinMaxDistance(0.03, 0.13);
      //   stage->setIKFrame(hand_frame_);

      //   geometry_msgs::msg::Vector3Stamped vec;
      //   vec.header.frame_id = table_reference_frame_;
      //   vec.vector.z = -1.0;
      //   stage->setDirection(vec);
      //   place->insert(std::move(stage));
      // }

      /* generate place pose */
      {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(target_object_);

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = table_reference_frame_;
        target_pose.pose.position.x = place_pose_x_;
        target_pose.pose.position.y = place_pose_y_;
        target_pose.pose.position.z = place_pose_z_ + 0.025;
        target_pose.pose.orientation.w = 1.0;
        stage->setPose(target_pose);
        stage->setMonitoredStage(attach_object_stage);
        
        // Compute IK
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(2);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(graspOffset(), hand_frame_);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }

      /* open hand */
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        stage->setGroup(hand_group_name_);
        stage->setGoal(open_pose_);
        place->insert(std::move(stage));
      }

      /* forbid collision (hand,object) */
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        stage->allowCollisions(target_object_, allLinks(hand_group_name_, task), false);
        place->insert(std::move(stage));
      }

      /* detach object */
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        stage->detachObject(target_object_, hand_frame_);
        place->insert(std::move(stage));
      }

      /* retreat */
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.10, 0.30);
        stage->setIKFrame(hand_frame_);
        stage->properties().set("marker_ns", "retreat");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame_;
        vec.vector.z = -0.5;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }

      task.add(std::move(place));
    }

    /* ---------------- return home ---------------- */
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
          "return home", interpolation_planner);
      stage->setGroup(arm_group_name_);
      stage->setGoal(ready_pose_);
      task.add(std::move(stage));
    }
    return task;
}

/* ---------------- doTask() ---------------------------------- */
void MTCPickPlaceNode::doTask()
{
  task_ = createTask();
  try { task_.init(); }
  catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e); return;
  }
  if (!task_.plan(5)) {
    RCLCPP_ERROR(LOGGER, "Task planning failed"); return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    RCLCPP_ERROR(LOGGER, "Task execution failed");
}

/* ========================================================================== */
/*                                   main                                     */
/* ========================================================================== */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<MTCPickPlaceNode>(options);
  rclcpp::executors::MultiThreadedExecutor exec;
  std::thread spin{[&]() {
      exec.add_node(node->getNodeBaseInterface());
      exec.spin();
      exec.remove_node(node->getNodeBaseInterface());
  }};

  node->setupPlanningScene();
  node->doTask();

  spin.join();
  rclcpp::shutdown();
  return 0;
}
