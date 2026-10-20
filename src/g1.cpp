#include "g1.h"
#include "config.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>
#include <RBDyn/parsers/urdf.h>
#include <functional>
#include <map>

namespace mc_robots
{

inline static std::string g1Variant(const std::string & variant)
{
  std::string fullName = "g1_" + variant;
  mc_rtc::log::info("G1RobotModule uses the G1 variant: '{}'", fullName);
  return fullName;
}

G1RobotModule::G1RobotModule(const std::string & variant)
: RobotModule(mc_rtc::G1_DESCRIPTION_PATH,
              g1Variant(variant),
              std::string(mc_rtc::G1_DESCRIPTION_PATH) + "/urdf/" + g1Variant(variant) + ".urdf")
{
  mc_rtc::log::success("G1RobotModule loaded with name: {}", name);
  rsdf_dir = std::string(mc_rtc::G1_DESCRIPTION_PATH) + "/rsdf";

  mc_rtc::log::success("G1RobotModule using URDF \"{}\"", urdf_path);
  mc_rtc::log::success("G1RobotModule using path \"{}\" for rsdf", rsdf_dir);

  // True if the robot has a fixed base, false otherwise
  bool fixed = false;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));

  // Reference joint order = G1 motor order (unitree_sdk2), keeping only the joints that are actuated in this
  // variant: 23dof (no waist roll/pitch, no wrist pitch/yaw), 29dof, or 29dof with locked waist (waist roll/pitch
  // are fixed joints in the Unitree URDF, so they are not part of the reference joint order)
  static const std::vector<std::pair<std::string, double>> g1JointsAndStance = {
      {"left_hip_pitch_joint", -0.312},   {"left_hip_roll_joint", 0.0},        {"left_hip_yaw_joint", 0.0},
      {"left_knee_joint", 0.669},         {"left_ankle_pitch_joint", -0.363},  {"left_ankle_roll_joint", 0.0},
      {"right_hip_pitch_joint", -0.312},  {"right_hip_roll_joint", 0.0},       {"right_hip_yaw_joint", 0.0},
      {"right_knee_joint", 0.669},        {"right_ankle_pitch_joint", -0.363}, {"right_ankle_roll_joint", 0.0},
      {"waist_yaw_joint", 0.0},           {"waist_roll_joint", 0.0},           {"waist_pitch_joint", 0.0},
      {"left_shoulder_pitch_joint", 0.2}, {"left_shoulder_roll_joint", 0.2},   {"left_shoulder_yaw_joint", 0.0},
      {"left_elbow_joint", 0.6},          {"left_wrist_roll_joint", 0.0},      {"left_wrist_pitch_joint", 0.0},
      {"left_wrist_yaw_joint", 0.0},      {"right_shoulder_pitch_joint", 0.2}, {"right_shoulder_roll_joint", -0.2},
      {"right_shoulder_yaw_joint", 0.0},  {"right_elbow_joint", 0.6},          {"right_wrist_roll_joint", 0.0},
      {"right_wrist_pitch_joint", 0.0},   {"right_wrist_yaw_joint", 0.0}};

  _ref_joint_order.clear();
  for(const auto & [jName, q] : g1JointsAndStance)
  {
    auto it = mb.jointIndexByName().find(jName);
    if(it == mb.jointIndexByName().end() || mb.joint(it->second).dof() != 1)
    {
      continue;
    }
    _ref_joint_order.push_back(jName);
    _stance[jName] = {q};
  }
  mc_rtc::log::info("G1RobotModule: {} actuated joints", _ref_joint_order.size());

  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.76}};

  // Sensors
  // The real G1 has two IMUs (imu_in_pelvis / imu_in_torso in the Unitree URDF):
  //  - the main IMU in the pelvis, published in rt/lowstate (imu_state) -> "Accelerometer" (default body sensor)
  //  - a secondary IMU in the torso, published in rt/secondary_imu       -> "TorsoAccelerometer"
  // The poses are read from the URDF fixed joints when available (the torso IMU height differs between URDF
  // revisions: 0.13792 or 0.14792), otherwise the values of the Unitree g1_23dof/g1_29dof URDF are used.
  auto imuPose = [this](const std::string & imuJoint, const std::string & defaultParent,
                        const Eigen::Vector3d & defaultPos) -> std::pair<std::string, sva::PTransformd>
  {
    const auto & jIdxByName = mb.jointIndexByName();
    auto it = jIdxByName.find(imuJoint);
    if(it != jIdxByName.end())
    {
      int jIdx = it->second;
      int parentBody = mb.parent(jIdx);
      if(parentBody >= 0)
      {
        return {mb.body(parentBody).name(), mb.transform(jIdx)};
      }
    }
    mc_rtc::log::warning("G1RobotModule: {} not found in the URDF, using the default IMU pose", imuJoint);
    return {defaultParent, sva::PTransformd(defaultPos)};
  };
  auto pelvisImu = imuPose("imu_in_pelvis_joint", "pelvis", Eigen::Vector3d(0.04525, 0., -0.08339));
  auto torsoImu = imuPose("imu_in_torso_joint", "torso_link", Eigen::Vector3d(-0.03959, -0.00224, 0.13792));
  _bodySensors.emplace_back("Accelerometer", pelvisImu.first, pelvisImu.second);
  _bodySensors.emplace_back("TorsoAccelerometer", torsoImu.first, torsoImu.second);
  _bodySensors.emplace_back("FloatingBase", "pelvis", sva::PTransformd::Identity());

  // pelvis_contour_link only exists in the deprecated and rev_1_0 models, the pelvis mesh includes it otherwise
  const std::string pelvisContour =
      mb.bodyIndexByName().count("pelvis_contour_link") ? "pelvis_contour_link" : "pelvis";
  _minimalSelfCollisions = {mc_rbdyn::Collision("torso_link", "left_shoulder_yaw_link", 0.02, 0.001, 0.),
                            mc_rbdyn::Collision("torso_link", "right_shoulder_yaw_link", 0.02, 0.001, 0.),
                            mc_rbdyn::Collision("torso_link", "left_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("torso_link", "right_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision(pelvisContour, "left_shoulder_yaw_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision(pelvisContour, "right_shoulder_yaw_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision(pelvisContour, "left_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision(pelvisContour, "right_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("left_hip_pitch_link", "right_hip_pitch_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_knee_link", "right_knee_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_pitch_link", "right_ankle_pitch_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_roll_link_0", "right_ankle_roll_link_0", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_pitch_link", "right_knee_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("right_ankle_pitch_link", "left_knee_link", 0.02, 0.01, 0.)};
  // Hand collisions (rubber hands of the 29dof models, wrist-roll hands of the 23dof models)
  for(const std::string side : {"left", "right"})
  {
    for(const auto & hand : {side + "_rubber_hand", side + "_wrist_roll_rubber_hand"})
    {
      if(!mb.bodyIndexByName().count(hand)) continue;
      _minimalSelfCollisions.emplace_back(hand, "torso_link", 0.05, 0.03, 0.);
      _minimalSelfCollisions.emplace_back(hand, pelvisContour, 0.05, 0.03, 0.);
    }
  }
  _commonSelfCollisions = _minimalSelfCollisions;
}

static mc_rbdyn::RobotModule * makeG1WithRevo2(const std::string & base_variant, const std::string & module_name)
{
  auto g1NoHands = std::make_shared<mc_robots::G1RobotModule>(base_variant + "_no_hands");

  auto leftRevo2 = mc_rbdyn::RobotLoader::get_robot_module("Revo2_LeftHand");
  auto rightRevo2 = mc_rbdyn::RobotLoader::get_robot_module("Revo2_RightHand");

  if(!leftRevo2 || !rightRevo2)
  {
    mc_rtc::log::error("Failed to load Revo2 modules while creating {}", module_name);
    return nullptr;
  }

  auto g1Left = g1NoHands->connect(*leftRevo2, "left_tool_attach", "left_base_link", "",
                                   mc_rbdyn::RobotModule::ConnectionParameters{}
                                       .name(module_name)
                                       .bodySensorMapping({{"FloatingBase", "FloatingBase_LeftHand"}}));

  auto g1Both = g1Left.connect(*rightRevo2, "right_tool_attach", "right_base_link", "",
                               mc_rbdyn::RobotModule::ConnectionParameters{}
                                   .name(module_name)
                                   .bodySensorMapping({{"FloatingBase", "FloatingBase_RightHand"}}));

  return new mc_rbdyn::RobotModule(g1Both);
}

/** G1 hardware variants: URDF variant name (urdf/g1_<variant>.urdf) and mode_machine reported by the robot.
 *  See https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description */
struct G1HardwareVariant
{
  std::string variant;
  int mode_machine;
};

static const std::vector<G1HardwareVariant> & g1HardwareVariants()
{
  static const std::vector<G1HardwareVariant> variants = {
      {"23dof", 1}, // deprecated
      {"29dof", 2}, // deprecated
      {"23dof_rev_1_0", 4},  {"29dof_rev_1_0", 5},  {"29dof_lock_waist_rev_1_0", 6},
      {"23dof_mode_10", 10}, {"29dof_mode_11", 11}, {"29dof_mode_12", 12},
      {"29dof_mode_13", 13}, {"29dof_mode_14", 14}, {"29dof_mode_15", 15},
      {"29dof_mode_16", 16}, {"29dof_mode_18", 18}};
  return variants;
}

static bool is29dof(const std::string & variant)
{
  return variant.rfind("29dof", 0) == 0;
}

/** Module name -> function creating the module */
static const std::map<std::string, std::function<mc_rbdyn::RobotModule *()>> & g1Modules()
{
  static const std::map<std::string, std::function<mc_rbdyn::RobotModule *()>> modules = []()
  {
    std::map<std::string, std::function<mc_rbdyn::RobotModule *()>> m;
    auto add = [&m](const std::string & name, const std::string & variant)
    {
      m[name] = [variant]() -> mc_rbdyn::RobotModule * { return new mc_robots::G1RobotModule(variant); };
    };
    auto addRevo2 = [&m](const std::string & name, const std::string & base_variant)
    {
      m[name] = [base_variant]() -> mc_rbdyn::RobotModule *
      { return makeG1WithRevo2(base_variant, "g1_" + base_variant + "_revo2"); };
    };
    for(const auto & v : g1HardwareVariants())
    {
      // Full names: G1_29dof_mode_15, G1_29dof_mode_15_no_hands, G1_29dof_mode_15_Revo2
      add("G1_" + v.variant, v.variant);
      // Short names from the mode_machine of the robot: G1_mode_15, G1_mode_15_no_hands, G1_mode_15_Revo2
      add("G1_mode_" + std::to_string(v.mode_machine), v.variant);
      if(is29dof(v.variant))
      {
        add("G1_" + v.variant + "_no_hands", v.variant + "_no_hands");
        add("G1_mode_" + std::to_string(v.mode_machine) + "_no_hands", v.variant + "_no_hands");
        addRevo2("G1_" + v.variant + "_Revo2", v.variant);
        addRevo2("G1_mode_" + std::to_string(v.mode_machine) + "_Revo2", v.variant);
      }
    }
    // Historical names (deprecated hardware models)
    add("G1", "29dof");
    add("G1_no_hands", "29dof_no_hands");
    m["G1_Revo2"] = []() -> mc_rbdyn::RobotModule * { return makeG1WithRevo2("29dof", "g1_29dof_revo2"); };
    return m;
  }();
  return modules;
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names.clear();
    for(const auto & m : mc_robots::g1Modules())
    {
      names.push_back(m.first);
    }
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("G1")
    const auto & modules = mc_robots::g1Modules();
    auto it = modules.find(n);
    if(it == modules.end())
    {
      mc_rtc::log::error("G1 module cannot create an object of type {}", n);
      return nullptr;
    }
    return it->second();
  }
}
