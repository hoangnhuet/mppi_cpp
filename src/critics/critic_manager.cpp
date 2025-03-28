#include "critics/critic_manager.hpp"
#include "critics/goal_critic.hpp"

namespace mppi
{

void CriticManager::loadCritics()
{
    critics_.push_back(std::make_unique<critics::GoalCritic>());
    // Add other critics similarly
}

}  // namespace mppi
