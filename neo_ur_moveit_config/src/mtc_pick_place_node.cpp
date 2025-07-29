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
/*                           ENUM DECLARATION                                 */
/* ========================================================================== */
enum class GraspOrientation { Vertical, Horizontal };

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
  Eigen::Isometry3d graspOffset(GraspOrientation orientation) const;
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
  node_->declare_parameter("target_object",    "can_1");
  node_->declare_parameter("table_reference_frame", "base_link");
  node_->declare_parameter("place_pose_x", 0.8);
  node_->declare_parameter("place_pose_y", 0.0);
  node_->declare_parameter("place_pose_z", 0.875);
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

  // Updated table to match your world
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = table_reference_frame_;
  table.id = "simple_table";
  shape_msgs::msg::SolidPrimitive tbl;
  tbl.type = tbl.BOX; tbl.dimensions = {0.8, 1.2, 0.7}; // Updated dimensions
  geometry_msgs::msg::Pose tbl_pose;
  tbl_pose.orientation.w = 1.0;
  tbl_pose.position.x = 0.8; tbl_pose.position.z = 0.35; // Updated position
  table.primitives.push_back(tbl);
  table.primitive_poses.push_back(tbl_pose);
  table.operation = table.ADD;

  // Bin walls instead of single storage bin
  // Front wall
  moveit_msgs::msg::CollisionObject bin_front_wall;
  bin_front_wall.header.frame_id = table_reference_frame_;
  bin_front_wall.id = "bin_front_wall";
  shape_msgs::msg::SolidPrimitive front_wall;
  front_wall.type = front_wall.BOX; 
  front_wall.dimensions = {0.6, 0.02, 0.30};
  geometry_msgs::msg::Pose front_wall_pose;
  front_wall_pose.orientation.w = 1.0;
  front_wall_pose.position.x = 0.8; front_wall_pose.position.y = 0.0; front_wall_pose.position.z = 0.85;
  bin_front_wall.primitives.push_back(front_wall);
  bin_front_wall.primitive_poses.push_back(front_wall_pose);
  bin_front_wall.operation = bin_front_wall.ADD;

  // Back wall
  moveit_msgs::msg::CollisionObject bin_back_wall;
  bin_back_wall.header.frame_id = table_reference_frame_;
  bin_back_wall.id = "bin_back_wall";
  shape_msgs::msg::SolidPrimitive back_wall;
  back_wall.type = back_wall.BOX; 
  back_wall.dimensions = {0.6, 0.02, 0.30};
  geometry_msgs::msg::Pose back_wall_pose;
  back_wall_pose.orientation.w = 1.0;
  back_wall_pose.position.x = 0.8; back_wall_pose.position.y = 0.30; back_wall_pose.position.z = 0.85;
  bin_back_wall.primitives.push_back(back_wall);
  bin_back_wall.primitive_poses.push_back(back_wall_pose);
  bin_back_wall.operation = bin_back_wall.ADD;

  // Left wall
  moveit_msgs::msg::CollisionObject bin_left_wall;
  bin_left_wall.header.frame_id = table_reference_frame_;
  bin_left_wall.id = "bin_left_wall";
  shape_msgs::msg::SolidPrimitive left_wall;
  left_wall.type = left_wall.BOX; 
  left_wall.dimensions = {0.02, 0.30, 0.30};
  geometry_msgs::msg::Pose left_wall_pose;
  left_wall_pose.orientation.w = 1.0;
  left_wall_pose.position.x = 0.50; left_wall_pose.position.y = 0.15; left_wall_pose.position.z = 0.85;
  bin_left_wall.primitives.push_back(left_wall);
  bin_left_wall.primitive_poses.push_back(left_wall_pose);
  bin_left_wall.operation = bin_left_wall.ADD;

  // Right wall
  moveit_msgs::msg::CollisionObject bin_right_wall;
  bin_right_wall.header.frame_id = table_reference_frame_;
  bin_right_wall.id = "bin_right_wall";
  shape_msgs::msg::SolidPrimitive right_wall;
  right_wall.type = right_wall.BOX; 
  right_wall.dimensions = {0.02, 0.30, 0.30};
  geometry_msgs::msg::Pose right_wall_pose;
  right_wall_pose.orientation.w = 1.0;
  right_wall_pose.position.x = 1.10; right_wall_pose.position.y = 0.15; right_wall_pose.position.z = 0.85;
  bin_right_wall.primitives.push_back(right_wall);
  bin_right_wall.primitive_poses.push_back(right_wall_pose);
  bin_right_wall.operation = bin_right_wall.ADD;

  // Coke Can 1 (target object) - updated dimensions and position
  moveit_msgs::msg::CollisionObject can_1;
  can_1.header.frame_id = table_reference_frame_;
  can_1.id = "can_1";
  shape_msgs::msg::SolidPrimitive c1; 
  c1.type = c1.CYLINDER;
  c1.dimensions = {0.145, 0.04}; // Updated: height 0.145m, diameter 0.04m
  geometry_msgs::msg::Pose c1_pose;
  c1_pose.orientation.w = 1.0;
  c1_pose.position.x = 0.7; c1_pose.position.y = -0.2; c1_pose.position.z = 0.7725; // Updated position
  can_1.primitives.push_back(c1);
  can_1.primitive_poses.push_back(c1_pose);
  can_1.operation = can_1.ADD;

  // Coke Can 2 - updated dimensions and position
  moveit_msgs::msg::CollisionObject can_2;
  can_2.header.frame_id = table_reference_frame_;
  can_2.id = "can_2";
  shape_msgs::msg::SolidPrimitive c2; 
  c2.type = c2.CYLINDER;
  c2.dimensions = {0.145, 0.04}; // Updated: height 0.145m, diameter 0.04m
  geometry_msgs::msg::Pose c2_pose;
  c2_pose.orientation.w = 1.0;
  c2_pose.position.x = 0.9; c2_pose.position.y = -0.4; c2_pose.position.z = 0.7725; // Updated position
  can_2.primitives.push_back(c2);
  can_2.primitive_poses.push_back(c2_pose);
  can_2.operation = can_2.ADD;

  psi.applyCollisionObjects({table, bin_front_wall, bin_back_wall, bin_left_wall, bin_right_wall, can_1, can_2});
  RCLCPP_INFO(LOGGER, "Added collision objects to planning scene");
}

/* ---------- graspOffset, allLinks--------------------------- */

Eigen::Isometry3d MTCPickPlaceNode::graspOffset(GraspOrientation orientation) const
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (orientation == GraspOrientation::Vertical) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(0, 0, 0.24); // Increased offset for better clearance
  } else { // Horizontal
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(0, 0, 0.18); // Increased offset for better clearance
  }
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
        stage->setMinMaxDistance(0.05, 0.20); // Increased distance for better approach

        // Set hand approach direction - approach from the side for cylindrical objects
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame_;
        vec.vector.x = -1.0; // Approach from the side instead of from above
        stage->setDirection(vec);
        grasp->insert(std::move(stage));

      }

/* ========== GRASP ORIENTATION FALLBACK ========== */
{
  auto grasp_fallback = std::make_unique<mtc::Fallbacks>("grasp orientation fallback");
grasp->properties().exposeTo(
   grasp_fallback->properties(),
   { "eef", "group", "ik_frame" }
);
  grasp_fallback->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });
                                              
  // --- vertical grasp
  {
    auto gen_vert = std::make_unique<mtc::stages::GenerateGraspPose>("generate vertical grasp");
    gen_vert->properties().configureInitFrom(mtc::Stage::PARENT);
    gen_vert->properties().set("marker_ns", "grasp_pose");         // your original marker_ns
    gen_vert->setPreGraspPose(open_pose_);                          
    gen_vert->setObject(target_object_);                            
    gen_vert->setAngleDelta(M_PI / 12);                            // your original angle delta
    gen_vert->setMonitoredStage(current_state_ptr);                

    auto ik_vert = std::make_unique<mtc::stages::ComputeIK>("vertical grasp IK", std::move(gen_vert));
    ik_vert->setMaxIKSolutions(8);
    ik_vert->setMinSolutionDistance(1.0);
    ik_vert->setIKFrame(graspOffset(GraspOrientation::Vertical), hand_frame_);
    ik_vert->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    ik_vert->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

    grasp_fallback->insert(std::move(ik_vert));
  }

  // --- horizontal grasp
  {
    auto gen_horiz = std::make_unique<mtc::stages::GenerateGraspPose>("generate horizontal grasp");
    gen_horiz->properties().configureInitFrom(mtc::Stage::PARENT);
    gen_horiz->properties().set("marker_ns", "grasp_pose");       // same marker_ns
    gen_horiz->setPreGraspPose(open_pose_);
    gen_horiz->setObject(target_object_);
    gen_horiz->setAngleDelta(M_PI / 12);                          // same angle delta
    gen_horiz->setMonitoredStage(current_state_ptr);

    auto ik_horiz = std::make_unique<mtc::stages::ComputeIK>("horizontal grasp IK", std::move(gen_horiz));
    ik_horiz->setMaxIKSolutions(8);
    ik_horiz->setMinSolutionDistance(1.0);
    ik_horiz->setIKFrame(graspOffset(GraspOrientation::Horizontal), hand_frame_);
    ik_horiz->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    ik_horiz->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

    grasp_fallback->insert(std::move(ik_horiz));
  }

  // insert the fallback into your pick container
  grasp->insert(std::move(grasp_fallback));
}


      /* allow collision (object,table) */
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,table)");
        stage->allowCollisions(target_object_, std::vector<std::string>{"simple_table"}, true);
        grasp->insert(std::move(stage));
      }

      /* allow collision (hand,object) */
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
        stage->allowCollisions(target_object_, std::vector<std::string>{"simple_table", "bin_front_wall", "bin_back_wall", "bin_left_wall", "bin_right_wall"}, true);
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

      task.add(std::move(grasp));
    }
    // // —— forbid collisions with the bin walls ——
    // {
    //   auto forbid_walls = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,walls)");
    //   auto hand_links = task.getRobotModel()
    //                       ->getJointModelGroup(hand_group_name_)
    //                       ->getLinkModelNamesWithCollisionGeometry();
    //   for (const auto* wall_id : {"bin_front_wall", "bin_back_wall", "bin_left_wall", "bin_right_wall"}) {
    //     forbid_walls->allowCollisions(wall_id, hand_links, false);
    //   }
    //   task.add(std::move(forbid_walls));
    // }
    // forbid collisions between the can and each wall
auto forbid_object_walls = std::make_unique<mtc::stages::ModifyPlanningScene>(
    "forbid collision (object,walls)");
for (const auto* wall_id : {"bin_front_wall",
                            "bin_back_wall",
                            "bin_left_wall",
                            "bin_right_wall"}) {
  // target_object_ is e.g. "can_1"
  forbid_object_walls->allowCollisions(target_object_,
                                       std::vector<std::string>{wall_id},
                                       /* allow = */ false);
}
task.add(std::move(forbid_object_walls));

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

      /* ========== PLACE ORIENTATION FALLBACK ========== */
      {
        auto place_fallback = std::make_unique<mtc::Fallbacks>("place orientation fallback");
        place->properties().exposeTo(
           place_fallback->properties(),
           { "eef", "group", "ik_frame" }
        );
        place_fallback->properties().configureInitFrom(mtc::Stage::PARENT,
                                                    { "eef", "group", "ik_frame" });
                                                        
        // --- vertical place
        {
          auto gen_vert = std::make_unique<mtc::stages::GeneratePlacePose>("generate vertical place pose");
          gen_vert->properties().configureInitFrom(mtc::Stage::PARENT);
          gen_vert->properties().set("marker_ns", "place_pose");
          gen_vert->setObject(target_object_);

          geometry_msgs::msg::PoseStamped target_pose;
          target_pose.header.frame_id = table_reference_frame_;
          target_pose.pose.position.x = place_pose_x_;
          target_pose.pose.position.y = place_pose_y_;
          target_pose.pose.position.z = place_pose_z_;
          target_pose.pose.orientation.w = 1.0;
          gen_vert->setPose(target_pose);
          gen_vert->setMonitoredStage(attach_object_stage);
          
          // Compute IK
          auto ik_vert = std::make_unique<mtc::stages::ComputeIK>("vertical place IK", std::move(gen_vert));
          ik_vert->setMaxIKSolutions(2);
          ik_vert->setMinSolutionDistance(1.0);
          ik_vert->setIKFrame(graspOffset(GraspOrientation::Vertical), hand_frame_);
          ik_vert->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
          ik_vert->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

          place_fallback->insert(std::move(ik_vert));
        }

        // --- horizontal place
        {
          auto gen_horiz = std::make_unique<mtc::stages::GeneratePlacePose>("generate horizontal place pose");
          gen_horiz->properties().configureInitFrom(mtc::Stage::PARENT);
          gen_horiz->properties().set("marker_ns", "place_pose");
          gen_horiz->setObject(target_object_);

          geometry_msgs::msg::PoseStamped target_pose;
          target_pose.header.frame_id = table_reference_frame_;
          target_pose.pose.position.x = place_pose_x_;
          target_pose.pose.position.y = place_pose_y_;
          target_pose.pose.position.z = place_pose_z_ + 0.06; // Place can inside bin
          target_pose.pose.orientation.w = 1.0;
          gen_horiz->setPose(target_pose);
          gen_horiz->setMonitoredStage(attach_object_stage);
          
          // Compute IK
          auto ik_horiz = std::make_unique<mtc::stages::ComputeIK>("horizontal place IK", std::move(gen_horiz));
          ik_horiz->setMaxIKSolutions(2);
          ik_horiz->setMinSolutionDistance(1.0);
          ik_horiz->setIKFrame(graspOffset(GraspOrientation::Horizontal), hand_frame_);
          ik_horiz->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
          ik_horiz->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

          place_fallback->insert(std::move(ik_horiz));
        }

        // insert the fallback into your place container
        place->insert(std::move(place_fallback));
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
