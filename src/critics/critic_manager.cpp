#include "critics/critic_manager.hpp"
#include "critics/goal_critic.hpp"

namespace mppi::critics
{

void CriticManager::loadCritics()
{
    critics_.emplace_back(std::make_unique<mppi::critics::GoalCritic>());
}
void CriticManager::evalTrajectoriesScores(
    CriticData &data) const
{
  for (size_t q = 0; q < critics_.size(); q++)
  {
    critics_[q]->score(data);
  }
}

}  // namespace mppi
