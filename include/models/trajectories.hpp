#include<xtensor/xtensor.hpp>
#include<xtensor/xview.hpp>
namespace mppi::models
{
    struct Trajectories
    {
        xt::xtensor<float,2> x;
        xt::xtensor<float,2> y;
        xt::xtensor<float,2> yaw;
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        // each state have batch_size (number of samples) and time_steps (time_horizon)
        x = xt::zeros<float>({batch_size, time_steps});
        y = xt::zeros<float>({batch_size, time_steps});
        yaw = xt::zeros<float>({batch_size, time_steps});
    }
    };
};