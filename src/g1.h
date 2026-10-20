#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI G1RobotModule : public mc_rbdyn::RobotModule
{
  G1RobotModule(const std::string & variant = "29dof");
};

} // namespace mc_robots
