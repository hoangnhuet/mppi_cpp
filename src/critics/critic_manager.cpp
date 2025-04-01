#include "critics/critic_manager.hpp"
#include "critics/goal_critic.hpp"

namespace mppi::critics
{

void CriticManager::loadCritics()
{
    critics_.emplace_back(std::make_unique<mppi::critics::GoalCritic>());
}

}  // namespace mppi
