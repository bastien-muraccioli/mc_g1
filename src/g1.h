#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI G1RobotModule : public mc_rbdyn::RobotModule
{
  G1RobotModule(const std::string & variant = "29dof");

  /** Add the collisions between a hand body and the legs (same side: hip roll, thigh, shin; other side: thigh, shin) */
  static void addHandLegCollisions(std::vector<mc_rbdyn::Collision> & collisions,
                                   const std::string & hand,
                                   const std::string & side);
};

} // namespace mc_robots
