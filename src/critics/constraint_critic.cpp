#include"constraint_critic.hpp"

namespace mppi::critics
{
    auto weight_ = Params().weight_;
    auto power_ = Params().power_;
    auto max_vel_ = Params().v_max;
    auto min_vel_ = Params().v_min;
    void ConstraintCritic::score(CriticData &data)
    {
        using xt::evaluation_strategy::immediate;
      
        auto sgn = xt::where(data.state.v > 0.0, 1.0, -1.0);
        auto vel_total = sgn * xt::sqrt(data.state.v * data.state.v);
        auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);
        auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);   
      
        data.costs += xt::pow(
          xt::sum(
            (std::move(out_of_max_bounds_motion) +
            std::move(out_of_min_bounds_motion)) *
            data.model_dt, {1}, immediate) * weight_, power_);
      }
}
