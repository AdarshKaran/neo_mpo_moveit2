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
    enum class Grasp { Vertical, Horizontal, TopDown } grasp;
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

        auto go = tnode["grasp_orientation"].as<std::string>();
        if (go == "vertical")     tc.grasp = TaskConfig::Grasp::Vertical;
        else if (go == "horizontal") tc.grasp = TaskConfig::Grasp::Horizontal;
        else                         tc.grasp = TaskConfig::Grasp::TopDown;

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
  const std::string& getOpenPose() const { return open_pose_; }
  const std::string& getClosePose() const { return close_pose_; }
  rclcpp::Node::SharedPtr getNode() const { return node_; }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void doMultipleTasks(bool sequential_mode = true);

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
  node_->declare_parameter("sequential_mode", true);

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
  bool sequential = node_->get_parameter("sequential_mode").as_bool();
  
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
  tbl.type = tbl.BOX; tbl.dimensions = {0.8, 1.2, 0.7};
  geometry_msgs::msg::Pose tbl_pose;
  tbl_pose.orientation.w = 1.0;
  tbl_pose.position.x = 0.8; tbl_pose.position.z = 0.35;
  table.primitives.push_back(tbl);
  table.primitive_poses.push_back(tbl_pose);
  table.operation = table.ADD;
  collision_objects.push_back(table);

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
  collision_objects.push_back(bin_front_wall);

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
  collision_objects.push_back(bin_back_wall);

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
  collision_objects.push_back(bin_left_wall);

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
  collision_objects.push_back(bin_right_wall);

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
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", node->getHandFrame());
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.20); // Increased distance for better approach

      // Set hand approach direction - approach from the side for cylindrical objects
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = node->getHandFrame();
      vec.vector.x = -1.0; // Approach from the side instead of from above
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /* ========== GRASP ORIENTATION FALLBACK ========== */
    {
      auto grasp_fallback = std::make_unique<mtc::Fallbacks>("grasp orientation fallback");
      grasp->properties().exposeTo(grasp_fallback->properties(), { "eef", "group", "ik_frame" });
      grasp_fallback->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
                                                  
      // --- vertical grasp
      {
        auto gen_vert = std::make_unique<mtc::stages::GenerateGraspPose>("generate vertical grasp");
        gen_vert->properties().configureInitFrom(mtc::Stage::PARENT);
        gen_vert->properties().set("marker_ns", "grasp_pose");         // your original marker_ns
        gen_vert->setPreGraspPose(node->getOpenPose());                          
        gen_vert->setObject(target_object);                            
        gen_vert->setAngleDelta(M_PI / 12);                            // your original angle delta
        gen_vert->setMonitoredStage(current_state_ptr);                

        auto ik_vert = std::make_unique<mtc::stages::ComputeIK>("vertical grasp IK", std::move(gen_vert));
        ik_vert->setMaxIKSolutions(8);
        ik_vert->setMinSolutionDistance(1.0);
        ik_vert->setIKFrame(node->graspOffset(GraspOrientation::Vertical), node->getHandFrame());
        ik_vert->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        ik_vert->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        grasp_fallback->insert(std::move(ik_vert));
      }

      // --- horizontal grasp
      {
        auto gen_horiz = std::make_unique<mtc::stages::GenerateGraspPose>("generate horizontal grasp");
        gen_horiz->properties().configureInitFrom(mtc::Stage::PARENT);
        gen_horiz->properties().set("marker_ns", "grasp_pose");       // same marker_ns
        gen_horiz->setPreGraspPose(node->getOpenPose());
        gen_horiz->setObject(target_object);
        gen_horiz->setAngleDelta(M_PI / 12);                          // same angle delta
        gen_horiz->setMonitoredStage(current_state_ptr);

        auto ik_horiz = std::make_unique<mtc::stages::ComputeIK>("horizontal grasp IK", std::move(gen_horiz));
        ik_horiz->setMaxIKSolutions(8);
        ik_horiz->setMinSolutionDistance(1.0);
        ik_horiz->setIKFrame(node->graspOffset(GraspOrientation::Horizontal), node->getHandFrame());
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
      stage->allowCollisions(target_object, std::vector<std::string>{"simple_table"}, true);
      grasp->insert(std::move(stage));
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
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.15, 0.4);  // Increased from 0.1, 0.3
      stage->setIKFrame(node->getHandFrame());
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = node->getTableReferenceFrame();
      vec.vector.z = 1.0;
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

    /* ========== PLACE ORIENTATION FALLBACK ========== */
    {
      auto place_fallback = std::make_unique<mtc::Fallbacks>("place orientation fallback");
      place->properties().exposeTo(place_fallback->properties(), { "eef", "group", "ik_frame" });
      place_fallback->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
                                                      
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
        target_pose.pose.position.z = place_position[2];
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
        target_pose.pose.position.z = place_position[2] + 0.06; // Place can inside bin
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
      stage->setMinMaxDistance(0.10, 0.30);
      stage->setIKFrame(node->getHandFrame());
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = node->getHandFrame();
      vec.vector.z = -0.5;
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
  
  mtc::Task buildSequentialPickPlaceTask(const std::vector<ConfigurationManager::TaskConfig>& task_configs)
  {
    mtc::Task task;
    task.stages()->setName("sequential_pick_place_task");
    task.loadRobotModel(node_->getNode());
    
    task.setProperty("group", node_->getArmGroupName());
    task.setProperty("eef", node_->getEefName());
    task.setProperty("ik_frame", node_->getHandFrame());
    
    mtc::Stage* current_state_ptr = nullptr;
    
    // Current state
    {
      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = stage_state_current.get();
      task.add(std::move(stage_state_current));
    }
    
    // Setup planners
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_->getNode());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    
    // Tune these for speed
    sampling_planner->setMaxVelocityScalingFactor(0.1);
    sampling_planner->setMaxAccelerationScalingFactor(0.1);
    interpolation_planner->setMaxVelocityScalingFactor(0.2);
    interpolation_planner->setMaxAccelerationScalingFactor(0.2);
    
    // Initial open hand
    {
      auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("initial open hand", interpolation_planner);
      stage_open_hand->setGroup(node_->getHandGroupName());
      stage_open_hand->setGoal(node_->getOpenPose());
      task.add(std::move(stage_open_hand));
    }
    
    // Process each object sequentially
    for (size_t i = 0; i < task_configs.size(); ++i) {
      const auto& task_config = task_configs[i];
      std::string task_prefix = "task_" + std::to_string(i + 1) + "_" + task_config.object_id;
      
      mtc::Stage* attach_object_stage = nullptr;
      
      // Move to pick (only for first object, others continue from previous place)
      if (i == 0) {
        auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
            task_prefix + "_move_to_pick",
            mtc::stages::Connect::GroupPlannerVector{ { node_->getArmGroupName(), sampling_planner } });
        stage_move_to_pick->setTimeout(5.0);
        stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_pick));
      } else {
        // For subsequent objects, add a direct move from place to next pick
        auto stage_move_to_next_pick = std::make_unique<mtc::stages::Connect>(
            task_prefix + "_move_to_next_pick",
            mtc::stages::Connect::GroupPlannerVector{ { node_->getArmGroupName(), sampling_planner } });
        stage_move_to_next_pick->setTimeout(5.0);
        stage_move_to_next_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_next_pick));
      }
      
      // Pick container
      auto pick_container = PickTaskFactory::createPickContainer(
          node_, task_config.object_id, task_config.grasp, 
          current_state_ptr, attach_object_stage, task);
      pick_container->setName(task_prefix + "_pick");
      task.add(std::move(pick_container));
      
      // Forbid wall collisions
      {
        auto forbid_object_walls = std::make_unique<mtc::stages::ModifyPlanningScene>(
            task_prefix + "_forbid_collision_walls");
        for (const auto* wall_id : {"bin_front_wall", "bin_back_wall", "bin_left_wall", "bin_right_wall"}) {
          forbid_object_walls->allowCollisions(task_config.object_id, std::vector<std::string>{wall_id}, false);
        }
        task.add(std::move(forbid_object_walls));
      }
      
      // Move to place
      {
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            task_prefix + "_move_to_place",
            mtc::stages::Connect::GroupPlannerVector{ { node_->getArmGroupName(), sampling_planner } });
        stage_move_to_place->setTimeout(5.0);
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_place));
      }
      
      // Place container
      std::vector<double> place_pos = {task_config.place_x, task_config.place_y, task_config.place_z};
      auto place_container = PlaceTaskFactory::createPlaceContainer(
          node_, task_config.object_id, place_pos, 
          task_config.grasp, attach_object_stage, task);
      place_container->setName(task_prefix + "_place");
      task.add(std::move(place_container));
    }
    
    // Only return home at the very end
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("final_return_home", interpolation_planner);
      stage->setGroup(node_->getArmGroupName());
      stage->setGoal(node_->getReadyPose());
      task.add(std::move(stage));
    }
    
    return task;
  }


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
    sampling_planner->setMaxVelocityScalingFactor(0.2);
    sampling_planner->setMaxAccelerationScalingFactor(0.2);
    // return to home speed
    interpolation_planner->setMaxVelocityScalingFactor(0.2);
    interpolation_planner->setMaxAccelerationScalingFactor(0.2);
    
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
    std::vector<double> place_pos = {task_config.place_x, task_config.place_y, task_config.place_z};
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
void MTCPickPlaceNode::doMultipleTasks(bool sequential_mode)
{
  auto& config = ConfigurationManager::getInstance();
  MTCTaskBuilder builder(this);

  const auto& task_configs = config.getTasks();

    if (sequential_mode) {
    // Sequential mode - one big task
    RCLCPP_INFO(LOGGER, "Executing SEQUENTIAL pick-place for %zu objects", task_configs.size());
    
    mtc::Task sequential_task = builder.buildSequentialPickPlaceTask(task_configs);
    
    try { 
      sequential_task.init(); 
    }
    catch (mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(LOGGER, "Sequential task init failed: " << e); 
      return;
    }
    
    if (!sequential_task.plan(10)) {
      RCLCPP_ERROR(LOGGER, "Sequential task planning failed"); 
      return;
    }
    
    sequential_task.introspection().publishSolution(*sequential_task.solutions().front());
    auto result = sequential_task.execute(*sequential_task.solutions().front());
    
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Sequential task execution failed");
      return;
    }
    
    RCLCPP_INFO(LOGGER, "Successfully completed all sequential tasks!");
  } else {
    // Individual mode - separate tasks (original behavior)
    RCLCPP_INFO(LOGGER, "Executing INDIVIDUAL pick-place for %zu objects", task_configs.size());
    
    for (const auto& task_config : task_configs) {
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
  bool sequential = node->getNode()->get_parameter("sequential_mode").as_bool();
  node->doMultipleTasks(sequential); // Change to false for individual tasks

  spin.join();
  rclcpp::shutdown();
  return 0;
}
