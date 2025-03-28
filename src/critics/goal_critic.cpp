#include"critics/goal_critic.hpp"

namespace mppi::critics
{
    auto weight_ = Params().weight_;
    auto power_ = Params().power_;
    using xt::evaluation_strategy::immediate;
    void GoalCritic::score(CriticData &data)
    {
        const auto goal_x = data.path.x(0);
        const auto goal_y = data.path.y(0);
        const auto goal_yaw = data.path.yaws(0);

        const auto traj_x = xt::view(data.trajectories.x, xt::all(), xt::all());
        const auto traj_y = xt::view(data.trajectories.y, xt::all(), xt::all());
        auto dists = xt::sqrt(
            xt::pow(traj_x - goal_x, 2) +
            xt::pow(traj_y - goal_y, 2));

        data.costs += xt::pow(xt::mean(dists, {1}, immediate) * weight_, power_);
    }
} // namespace mppi::critics