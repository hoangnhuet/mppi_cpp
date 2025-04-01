#include <memory>
#include <vector>
#include <xtensor/xtensor.hpp>

#include"models/state.hpp"
#include"models/path.hpp"
#include"models/trajectories.hpp"
#include"motion_model.hpp"

namespace mppi
{
    struct CriticData
    {
        const models::State & state;
        const models::Trajectories & trajectories;
        const models::Path & path;
        xt::xtensor<float, 1> & costs;
        float & model_dt;
        std::shared_ptr<MotionModel> motion_model;
    };
}