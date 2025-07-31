#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
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

#include <yaml-cpp/yaml.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_node");
namespace mtc = moveit::task_constructor;

/* ========================================================================== */
/*                       CONFIGURATION MANAGER                               */
/* ========================================================================== */
class ConfigurationManager
{
public:
  struct ObjectConfig {
    std::string id;
    enum class Type { Cylinder, Box } type;
    // Cylinder dimensions
    double height{0}, radius{0};
    // Box dimensions
    double size_x{0}, size_y{0}, size_z{0};
    // Pose
    double x{0}, y{0}, z{0}, qw{1}, qx{0}, qy{0}, qz{0};
  };
  struct TaskConfig {
    std::string object_id;
    enum class Grasp { Vertical, Horizontal, Both } grasp;
    double place_x{0}, place_y{0}, place_z{0}, place_qw{1}, place_qx{0}, place_qy{0}, place_qz{0};
  };

  static ConfigurationManager& getInstance() {
    static ConfigurationManager instance;
    return instance;
  }

  ConfigurationManager(ConfigurationManager const&) = delete;
  void operator=(ConfigurationManager const&) = delete;

void loadFromNode(const rclcpp::Node::SharedPtr& node)
{
  node->declare_parameter("config_file", "");
  std::string config_file = node->get_parameter("config_file").as_string();
  if (config_file.empty()) {
    RCLCPP_WARN(LOGGER, "No config file specified, using defaults");
    return;
  }

  try {
    YAML::Node config = YAML::LoadFile(config_file);

    // --- OBJECTS (must be a map) ---
    if (config["objects"] && config["objects"].IsMap()) {
      for (auto it = config["objects"].begin(); it != config["objects"].end(); ++it) {
        const std::string id = it->first.as<std::string>();
        const YAML::Node obj_node = it->second;

        ObjectConfig obj;
        obj.id = id;

        // type + dimensions
        const std::string t = obj_node["type"].as<std::string>();
        if (t == "cylinder") {
          obj.type   = ObjectConfig::Type::Cylinder;
          obj.height = obj_node["dimensions"]["height"].as<double>();
          obj.radius = obj_node["dimensions"]["radius"].as<double>();
        } else if (t == "box") {
          obj.type   = ObjectConfig::Type::Box;
          obj.size_x = obj_node["dimensions"]["x"].as<double>();
          obj.size_y = obj_node["dimensions"]["y"].as<double>();
          obj.size_z = obj_node["dimensions"]["z"].as<double>();
        }

        // pose
        auto p = obj_node["pose"];
        obj.x  = p["x"].as<double>();
        obj.y  = p["y"].as<double>();
        obj.z  = p["z"].as<double>();
        obj.qw = p["qw"] ? p["qw"].as<double>() : 1.0;
        obj.qx = p["qx"] ? p["qx"].as<double>() : 0.0;
        obj.qy = p["qy"] ? p["qy"].as<double>() : 0.0;
        obj.qz = p["qz"] ? p["qz"].as<double>() : 0.0;

        objects_[id] = obj;
      }
    }

    // --- TASKS (must be a sequence) ---
    if (config["tasks"] && config["tasks"].IsSequence()) {
      for (const auto& tnode : config["tasks"]) {
        TaskConfig tc;
        tc.object_id = tnode["object"].as<std::string>();

        auto grasp_orientation = tnode["grasp_orientation"].as<std::string>();
        if (grasp_orientation == "vertical")     tc.grasp = TaskConfig::Grasp::Vertical;
        else if (grasp_orientation == "horizontal") tc.grasp = TaskConfig::Grasp::Horizontal;
        else                                      tc.grasp = TaskConfig::Grasp::Both;

        auto pp = tnode["place_pose"];
        tc.place_x  = pp["x"].as<double>();
        tc.place_y  = pp["y"].as<double>();
        tc.place_z  = pp["z"].as<double>();
        tc.place_qw = pp["qw"] ? pp["qw"].as<double>() : 1.0;
        tc.place_qx = pp["qx"] ? pp["qx"].as<double>() : 0.0;
        tc.place_qy = pp["qy"] ? pp["qy"].as<double>() : 0.0;
        tc.place_qz = pp["qz"] ? pp["qz"].as<double>() : 0.0;

        tasks_.push_back(tc);
      }
    }

    RCLCPP_INFO(LOGGER, "Loaded %zu objects and %zu tasks", objects_.size(), tasks_.size());
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Failed to load config file: %s", e.what());
  }
}


  const std::vector<TaskConfig>& getTasks() const { return tasks_; }
  const ObjectConfig*               getObject(const std::string& id) const
  {
    auto it = objects_.find(id);
    return (it == objects_.end()) ? nullptr : &it->second;
  }

private:
  ConfigurationManager() = default;
  std::map<std::string, ObjectConfig>  objects_;
  std::vector<TaskConfig>              tasks_;
};

/* ========================================================================== */
/*                           OBJECT FACTORY                                  */
/* ========================================================================== */
class CollisionObjectFactory
{
public:
  static moveit_msgs::msg::CollisionObject createFromConfig(
      const ConfigurationManager::ObjectConfig& config,
      const std::string& frame_id)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id;
    object.id = config.id;
    
    shape_msgs::msg::SolidPrimitive primitive;
    
    if (config.type == ConfigurationManager::ObjectConfig::Type::Cylinder) {
      primitive.type = primitive.CYLINDER;
      primitive.dimensions = {config.height, config.radius};
    } else if (config.type == ConfigurationManager::ObjectConfig::Type::Box) {
      primitive.type = primitive.BOX;
      primitive.dimensions = {config.size_x, config.size_y, config.size_z};
    }
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = config.x;
    pose.position.y = config.y;
    pose.position.z = config.z;
    pose.orientation.w = config.qw;
    pose.orientation.x = config.qx;
    pose.orientation.y = config.qy;
    pose.orientation.z = config.qz;
    
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;
    
    return object;
  }
};



/* ========================================================================== */
/*                           ENUM DECLARATION                                 */
/* ========================================================================== */
enum class GraspOrientation { Vertical, Horizontal };

/* ========================================================================== */
/*                           MTCPickPlaceNode CLASS DECLARATION               */
/* ========================================================================== */
class MTCPickPlaceNode
{
public:
  explicit MTCPickPlaceNode(const rclcpp::NodeOptions& options);

  const std::string& getArmGroupName() const { return arm_group_name_; }
  const std::string& getHandGroupName() const { return hand_group_name_; }
  const std::string& getEefName() const { return eef_name_; }
  const std::string& getHandFrame() const { return hand_frame_; }
  const std::string& getTableReferenceFrame() const { return table_reference_frame_; }
  const std::string& getReadyPose() const { return ready_pose_; }
  const std::string& getIntermediatePose() const { return intermediate_pose_; }
  const std::string& getOpenPose() const { return open_pose_; }
  const std::string& getClosePose() const { return close_pose_; }
  rclcpp::Node::SharedPtr getNode() const { return node_; }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void doMultipleTasks();

  /* ----- helpers & members ------------------------------------------------ */
  Eigen::Isometry3d graspOffset(GraspOrientation orientation) const;
  std::vector<std::string> allLinks(const std::string& group,
                                    const mtc::Task& task) const;

private:

  /* ---------------- data -------------------------------------------------- */
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;

  std::string arm_group_name_, hand_group_name_, eef_name_, hand_frame_;
  std::string target_object_, table_reference_frame_;
  std::string ready_pose_, open_pose_, close_pose_, intermediate_pose_;

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
  node_->declare_parameter("hand_frame",       "neo_gripper_mount_link");
  node_->declare_parameter("target_object",    "can_1");
  node_->declare_parameter("table_reference_frame", "base_link");
  node_->declare_parameter("ready_pose",  "up");
  node_->declare_parameter("intermediate_pose", "intermediate_pose");
  node_->declare_parameter("open_pose",   "open");
  node_->declare_parameter("close_pose",  "close");

  arm_group_name_        = node_->get_parameter("arm_group_name").as_string();
  hand_group_name_       = node_->get_parameter("hand_group_name").as_string();
  eef_name_              = node_->get_parameter("eef_name").as_string();
  hand_frame_            = node_->get_parameter("hand_frame").as_string();
  target_object_         = node_->get_parameter("target_object").as_string();
  table_reference_frame_ = node_->get_parameter("table_reference_frame").as_string();
  ready_pose_            = node_->get_parameter("ready_pose").as_string();
  intermediate_pose_     = node_->get_parameter("intermediate_pose").as_string();
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

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = table_reference_frame_;
  table.id = "simple_table";
  shape_msgs::msg::SolidPrimitive tbl;
  tbl.type = tbl.BOX; tbl.dimensions = {0.8, 2, 0.7};
  geometry_msgs::msg::Pose tbl_pose;
  tbl_pose.orientation.w = 1.0;
  tbl_pose.position.x = 0.8; tbl_pose.position.z = 0.35;
  table.primitives.push_back(tbl);
  table.primitive_poses.push_back(tbl_pose);
  table.operation = table.ADD;
  collision_objects.push_back(table);

  // Shelf bottom
  moveit_msgs::msg::CollisionObject shelf_bottom;
  shelf_bottom.header.frame_id = table_reference_frame_;
  shelf_bottom.id = "simple_shelf_bottom";
  shape_msgs::msg::SolidPrimitive shelf_bottom_box;
  shelf_bottom_box.type = shelf_bottom_box.BOX;
  shelf_bottom_box.dimensions = {0.6, 0.3, 0.2};
  geometry_msgs::msg::Pose shelf_bottom_pose;
  shelf_bottom_pose.orientation.w = 1.0;
  shelf_bottom_pose.position.x = 0.8;
  shelf_bottom_pose.position.y = 0.5;
  shelf_bottom_pose.position.z = 0.8;
  shelf_bottom.primitives.push_back(shelf_bottom_box);
  shelf_bottom.primitive_poses.push_back(shelf_bottom_pose);
  shelf_bottom.operation = shelf_bottom.ADD;
  collision_objects.push_back(shelf_bottom);

  // Shelf top
  moveit_msgs::msg::CollisionObject shelf_top;
  shelf_top.header.frame_id = table_reference_frame_;
  shelf_top.id = "simple_shelf_top";
  shape_msgs::msg::SolidPrimitive shelf_top_box;
  shelf_top_box.type = shelf_top_box.BOX;
  shelf_top_box.dimensions = {0.6, 0.3, 0.2};
  geometry_msgs::msg::Pose shelf_top_pose;
  shelf_top_pose.orientation.w = 1.0;
  shelf_top_pose.position.x = 0.8;
  shelf_top_pose.position.y = 0.5;
  shelf_top_pose.position.z = 1.25;
  shelf_top.primitives.push_back(shelf_top_box);
  shelf_top.primitive_poses.push_back(shelf_top_pose);
  shelf_top.operation = shelf_top.ADD;
  collision_objects.push_back(shelf_top);

  // Shelf back
  moveit_msgs::msg::CollisionObject shelf_back;
  shelf_back.header.frame_id = table_reference_frame_;
  shelf_back.id = "simple_shelf_back";
  shape_msgs::msg::SolidPrimitive shelf_back_box;
  shelf_back_box.type = shelf_back_box.BOX;
  shelf_back_box.dimensions = {0.6, 0.02, 0.8};
  geometry_msgs::msg::Pose shelf_back_pose;
  shelf_back_pose.orientation.w = 1.0;
  shelf_back_pose.position.x = 0.8;
  shelf_back_pose.position.y = 0.65;
  shelf_back_pose.position.z = 1.1;
  shelf_back.primitives.push_back(shelf_back_box);
  shelf_back.primitive_poses.push_back(shelf_back_pose);
  shelf_back.operation = shelf_back.ADD;
  collision_objects.push_back(shelf_back);
  
  auto& config = ConfigurationManager::getInstance();
  for (const auto& task : config.getTasks()) {
    const auto* obj_config = config.getObject(task.object_id);
    if (obj_config) {
      auto collision_obj = CollisionObjectFactory::createFromConfig(*obj_config, table_reference_frame_);
      collision_objects.push_back(collision_obj);
    }
  }
  
  psi.applyCollisionObjects(collision_objects);
  RCLCPP_INFO(LOGGER, "Added %zu collision objects to planning scene", collision_objects.size());
}

/* ---------- graspOffset, allLinks--------------------------- */

Eigen::Isometry3d MTCPickPlaceNode::graspOffset(GraspOrientation orientation) const
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (orientation == GraspOrientation::Vertical) 
  {
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
    T.linear() = q.toRotationMatrix();
    // TODO: add vertical grasp offset and horizontal grasp offset to the YAML config as params
    T.translation() = Eigen::Vector3d(0, 0, -0.15); // offset along the local axis of the handFrame to grasp point
  }
  else { // Horizontal
    Eigen::Quaterniond q(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()));
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(0, 0, 0.15);
  }
  return T;
}

std::vector<std::string>
MTCPickPlaceNode::allLinks(const std::string& group, const mtc::Task& task) const
{
  const auto* jmg = task.getRobotModel()->getJointModelGroup(group);
  return jmg ? jmg->getLinkModelNamesWithCollisionGeometry() : std::vector<std::string>{};
}

/* ========================================================================== */
/*                   PICK TASK FACTORY                                        */
/* ========================================================================== */
class PickTaskFactory
{
public:
  static std::unique_ptr<mtc::SerialContainer> createPickContainer(
      const MTCPickPlaceNode* node,
      const std::string& target_object,
      const ConfigurationManager::TaskConfig::Grasp& grasp_type,
      mtc::Stage* current_state_ptr,
      mtc::Stage*& attach_object_stage_out,
      const mtc::Task& task)
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Setup planners
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.1);
    cartesian_planner->setMaxAccelerationScalingFactor(0.1);
    cartesian_planner->setStepSize(0.01);
    
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    interpolation_planner->setMaxVelocityScalingFactor(0.2);
    interpolation_planner->setMaxAccelerationScalingFactor(0.2);

    /* approach */
    {
      // this stage reads the incoming poase and transforms it to a new goal pose in world frame, offset from the hand frame
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner); // Propagating stage
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", node->getHandFrame()); // Which link’s pose to offset
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.1); // Increased distance for better approach

      // Set hand approach direction - approach from the side for cylindrical objects
      // in our code this can be specified in the YAML config
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = node->getHandFrame(); // In which frame “direction” vector is expressed
      vec.vector.z = 1.0; // forward axis
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /* ========== GRASP ORIENTATION FALLBACK ========== */
    {
      auto grasp_fallback = std::make_unique<mtc::Fallbacks>("grasp orientation fallback");
      grasp->properties().exposeTo(grasp_fallback->properties(), { "eef", "group", "ik_frame" });
      grasp_fallback->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      // Try both grasp orientations if no orientation is specified in the YAML config
      // Vertical grasp
      if (grasp_type == ConfigurationManager::TaskConfig::Grasp::Vertical ||
          grasp_type == ConfigurationManager::TaskConfig::Grasp::Both)
      {
        auto gen_vert = std::make_unique<mtc::stages::GenerateGraspPose>("generate vertical grasp");
        gen_vert->properties().configureInitFrom(mtc::Stage::PARENT);
        gen_vert->properties().set("marker_ns", "grasp_pose");
        gen_vert->setPreGraspPose(node->getOpenPose());
        gen_vert->setObject(target_object);
        gen_vert->setAngleDelta(M_PI / 12);
        gen_vert->setMonitoredStage(current_state_ptr);

        auto ik_vert = std::make_unique<mtc::stages::ComputeIK>("vertical grasp IK", std::move(gen_vert));
        ik_vert->setMaxIKSolutions(8);
        ik_vert->setMinSolutionDistance(1.0);
        // the offset is needed so that the hand is not inside the object, the frame in which the offset is expressed
        ik_vert->setIKFrame(node->graspOffset(GraspOrientation::Vertical), node->getHandFrame());
        ik_vert->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        ik_vert->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        grasp_fallback->insert(std::move(ik_vert));
      }

      if (grasp_type == ConfigurationManager::TaskConfig::Grasp::Horizontal ||
          grasp_type == ConfigurationManager::TaskConfig::Grasp::Both)
      // Horizontal grasp
      {
        auto gen_horiz = std::make_unique<mtc::stages::GenerateGraspPose>("generate horizontal grasp");
        gen_horiz->properties().configureInitFrom(mtc::Stage::PARENT);
        gen_horiz->properties().set("marker_ns", "grasp_pose");
        gen_horiz->setPreGraspPose(node->getOpenPose());
        gen_horiz->setObject(target_object);
        gen_horiz->setAngleDelta(M_PI / 12);
        gen_horiz->setMonitoredStage(current_state_ptr);

        auto ik_horiz = std::make_unique<mtc::stages::ComputeIK>("horizontal grasp IK", std::move(gen_horiz));
        ik_horiz->setMaxIKSolutions(8);
        ik_horiz->setMinSolutionDistance(1.0);
        // the offset is needed so that the hand is not inside the object
        ik_horiz->setIKFrame(node->graspOffset(GraspOrientation::Horizontal), node->getHandFrame());
        ik_horiz->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        ik_horiz->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        grasp_fallback->insert(std::move(ik_horiz));
      }

      // insert the fallback into your pick container
      grasp->insert(std::move(grasp_fallback));
    }

    /* allow collision (hand,object) */
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(target_object,
                            task.getRobotModel()
                                ->getJointModelGroup(node->getHandGroupName())
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }

    /* close hand */
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(node->getHandGroupName());
      stage->setGoal(node->getClosePose());
      grasp->insert(std::move(stage));
    }

    /* attach */
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(target_object, node->getHandFrame());
      stage->allowCollisions(target_object, std::vector<std::string>{"simple_table", "bin_front_wall", "bin_back_wall", "bin_left_wall", "bin_right_wall"}, true);
      attach_object_stage_out = stage.get();
      grasp->insert(std::move(stage));
    }

    /* lift */
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().set("marker_ns", "lift_object");
      stage->setIKFrame(node->getHandFrame());
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(.03, .13);  // Increased from 0.1, 0.3

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = node->getHandFrame();
      vec.vector.x = -0.5; // upward direction
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    return grasp;
  }
};

/* ========================================================================== */
/*                   PLACE TASK FACTORY                                       */
/* ========================================================================== */
class PlaceTaskFactory
{
public:
  static std::unique_ptr<mtc::SerialContainer> createPlaceContainer(
      const MTCPickPlaceNode* node,
      const std::string& target_object,
      const std::vector<double>& place_position,
      const ConfigurationManager::TaskConfig::Grasp& grasp_type,
      mtc::Stage* attach_object_stage,
      const mtc::Task& task)
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Setup planners
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.1);
    cartesian_planner->setMaxAccelerationScalingFactor(0.1);
    cartesian_planner->setStepSize(0.01);
    
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    interpolation_planner->setMaxVelocityScalingFactor(0.2);
    interpolation_planner->setMaxAccelerationScalingFactor(0.2);

    		/******************************************************
  ---- *          Lower Object                              *
		 *****************************************************/
		{
			auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
			stage->properties().set("marker_ns", "lower_object");
			stage->properties().set("link", node->getHandFrame()); // Which link’s pose to offset
			stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.03, .13);

			// Set downward direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = node->getTableReferenceFrame();
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

    /* ========== PLACE ORIENTATION FALLBACK ========== */
    {
      auto place_fallback = std::make_unique<mtc::Fallbacks>("place orientation fallback");
      place->properties().exposeTo(place_fallback->properties(), { "eef", "group", "ik_frame" });
      place_fallback->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
                                    
      // Try both grasp orientations if no orientation is specified in the YAML config
      if (grasp_type == ConfigurationManager::TaskConfig::Grasp::Vertical ||
          grasp_type == ConfigurationManager::TaskConfig::Grasp::Both)
      // --- vertical place
      {
        auto gen_vert = std::make_unique<mtc::stages::GeneratePlacePose>("generate vertical place pose");
        gen_vert->properties().configureInitFrom(mtc::Stage::PARENT);
        gen_vert->properties().set("marker_ns", "place_pose");
        gen_vert->setObject(target_object);

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = node->getTableReferenceFrame();
        target_pose.pose.position.x = place_position[0];
        target_pose.pose.position.y = place_position[1];
        target_pose.pose.position.z = place_position[2]+0.5; // Adjusted for vertical placement
        			// p.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;

             RCLCPP_INFO(LOGGER, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Place target pose for %s: [x: %.3f, y: %.3f, z: %.3f, qw: %.3f, qx: %.3f, qy: %.3f, qz: %.3f]",
    target_object.c_str(),
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z,
    target_pose.pose.orientation.w,
    target_pose.pose.orientation.x,
    target_pose.pose.orientation.y,
    target_pose.pose.orientation.z); 
        target_pose.pose.orientation.w = 1.0;
        gen_vert->setPose(target_pose);
        gen_vert->setMonitoredStage(attach_object_stage);
        
        // Compute IK
        auto ik_vert = std::make_unique<mtc::stages::ComputeIK>("vertical place IK", std::move(gen_vert));
        ik_vert->setMaxIKSolutions(2);
        ik_vert->setMinSolutionDistance(1.0);
        ik_vert->setIKFrame(node->graspOffset(GraspOrientation::Vertical), node->getHandFrame());
        ik_vert->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        ik_vert->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        place_fallback->insert(std::move(ik_vert));
      }

      if (grasp_type == ConfigurationManager::TaskConfig::Grasp::Horizontal ||
        grasp_type == ConfigurationManager::TaskConfig::Grasp::Both)
      // --- horizontal place
      {
        auto gen_horiz = std::make_unique<mtc::stages::GeneratePlacePose>("generate horizontal place pose");
        gen_horiz->properties().configureInitFrom(mtc::Stage::PARENT);
        gen_horiz->properties().set("marker_ns", "place_pose");
        gen_horiz->setObject(target_object);

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = node->getTableReferenceFrame();
        target_pose.pose.position.x = place_position[0];
        target_pose.pose.position.y = place_position[1];
        target_pose.pose.position.z = place_position[2]+0.5;
        target_pose.pose.orientation.w = 1.0;
        gen_horiz->setPose(target_pose);
        gen_horiz->setMonitoredStage(attach_object_stage);
        
        // Compute IK
        auto ik_horiz = std::make_unique<mtc::stages::ComputeIK>("horizontal place IK", std::move(gen_horiz));
        ik_horiz->setMaxIKSolutions(2);
        ik_horiz->setMinSolutionDistance(1.0);
        ik_horiz->setIKFrame(node->graspOffset(GraspOrientation::Horizontal), node->getHandFrame());
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
      stage->setGroup(node->getHandGroupName());
      stage->setGoal(node->getOpenPose());
      place->insert(std::move(stage));
    }

    /* forbid collision (hand,object) */
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(target_object, node->allLinks(node->getHandGroupName(), task), false);
      place->insert(std::move(stage));
    }

    /* detach object */
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(target_object, node->getHandFrame());
      place->insert(std::move(stage));
    }

    /* retreat */
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(node->getHandFrame());
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = node->getHandFrame(); //retreat in world frame
      vec.vector.x = -0.5; // upward direction
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    return place;
  }
};

/* ========================================================================== */
/*                   MTC TASK BUILDER                                        */
/* ========================================================================== */
class MTCTaskBuilder
{
public:
  MTCTaskBuilder(const MTCPickPlaceNode* node) : node_(node) {}
  
  mtc::Task buildPickPlaceTask(const ConfigurationManager::TaskConfig& task_config)
  {
    mtc::Task task;
    task.stages()->setName("pick_place_task");
    task.loadRobotModel(node_->getNode());
    
    task.setProperty("group", node_->getArmGroupName());
    task.setProperty("eef", node_->getEefName());
    task.setProperty("ik_frame", node_->getHandFrame());
    
    mtc::Stage* current_state_ptr = nullptr;
    mtc::Stage* attach_object_stage = nullptr;
    
    // Current state
    {
      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = stage_state_current.get();
      task.add(std::move(stage_state_current));
    }
    
    // Setup planners
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_->getNode());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    
    // Tune these for horizontal speed
    sampling_planner->setMaxVelocityScalingFactor(0.4);
    sampling_planner->setMaxAccelerationScalingFactor(0.4);
    // return to home speed
    interpolation_planner->setMaxVelocityScalingFactor(0.6);
    interpolation_planner->setMaxAccelerationScalingFactor(0.6);
    
    // Open hand
    {
      auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(node_->getHandGroupName());
      stage_open_hand->setGoal(node_->getOpenPose());
      task.add(std::move(stage_open_hand));
    }
    
    // Move to pick
    {
      auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
          "move to pick",
          mtc::stages::Connect::GroupPlannerVector{ { node_->getArmGroupName(), sampling_planner } });
      stage_move_to_pick->setTimeout(5.0);
      stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_pick));
    }
    
    // Pick container (using factory)
    auto pick_container = PickTaskFactory::createPickContainer(
        node_, task_config.object_id, task_config.grasp, 
        current_state_ptr, attach_object_stage, task);
    task.add(std::move(pick_container));
    
    // Forbid wall collisions
    {
      auto forbid_object_walls = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,walls)");
      for (const auto* wall_id : {"bin_front_wall", "bin_back_wall", "bin_left_wall", "bin_right_wall"}) {
        forbid_object_walls->allowCollisions(task_config.object_id, std::vector<std::string>{wall_id}, false);
      }
      task.add(std::move(forbid_object_walls));
    }
    
    // Move to place
    {
      auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
          "move to place",
          mtc::stages::Connect::GroupPlannerVector{ { node_->getArmGroupName(), sampling_planner } });
      stage_move_to_place->setTimeout(5.0);
      stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_place));
    }
    
    // Place container (using factory)
    std::vector<double> place_pos = {
      task_config.place_x, task_config.place_y, task_config.place_z,
      task_config.place_qw, task_config.place_qx, task_config.place_qy, task_config.place_qz
    };
    auto place_container = PlaceTaskFactory::createPlaceContainer(
        node_, task_config.object_id, place_pos, 
        task_config.grasp, attach_object_stage, task);
    task.add(std::move(place_container));
    
    // Return home
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
      stage->setGroup(node_->getArmGroupName());
      stage->setGoal(node_->getReadyPose());
      task.add(std::move(stage));
    }
    
    return task;
  }
  
private:
  const MTCPickPlaceNode* node_;
};

/* ========================================================================== */
/*                   DO MULTIPLE TASKS                                         */
/* ========================================================================== */
void MTCPickPlaceNode::doMultipleTasks()
{
  auto& config = ConfigurationManager::getInstance();
  MTCTaskBuilder builder(this);
  
  for (const auto& task_config : config.getTasks()) {
    RCLCPP_INFO(LOGGER, "Executing task for object: %s", task_config.object_id.c_str());
    
    mtc::Task task = builder.buildPickPlaceTask(task_config);
    
    try { 
      task.init(); 
    }
    catch (mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task init failed for " << task_config.object_id << ": " << e); 
      continue;
    }
    
    if (!task.plan(5)) {
        RCLCPP_ERROR(LOGGER, "Task planning failed for %s", task_config.object_id.c_str());
        // Publish all failed solutions for RViz introspection
        for (const auto& sol : task.solutions())
          task.introspection().publishSolution(*sol);
        continue;
    }
    
    task.introspection().publishSolution(*task.solutions().front());
    auto result = task.execute(*task.solutions().front());
    
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Task execution failed for %s", task_config.object_id.c_str());
      continue;
    }
    
    RCLCPP_INFO(LOGGER, "Successfully completed task for %s", task_config.object_id.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}

/* ========================================================================== */
/*                                   main                                     */
/* ========================================================================== */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<MTCPickPlaceNode>(options);

  // Load configuration from YAML
  ConfigurationManager::getInstance().loadFromNode(node->getNode());

  rclcpp::executors::MultiThreadedExecutor exec;
  std::thread spin{[&]() {
      exec.add_node(node->getNodeBaseInterface());
      exec.spin();
      exec.remove_node(node->getNodeBaseInterface());
  }};

  node->setupPlanningScene();
  node->doMultipleTasks();

  spin.join();
  rclcpp::shutdown();
  return 0;
}
