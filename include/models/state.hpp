#include<xtensor/xtensor.hpp>
#include<xtensor/xview.hpp>
#include<utils.hpp>
namespace mppi::models
{
    struct State
    {
        xt::xtensor<float,2> v;
        xt::xtensor<float,2> w;

        xt::xtensor<float,2> cv;
        xt::xtensor<float,2> cw;

        Twist speed;
        Pose pose;
        
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        v = xt::zeros<float>({batch_size, time_steps});
        w = xt::zeros<float>({batch_size, time_steps});

        cv = xt::zeros<float>({batch_size, time_steps});
        cw = xt::zeros<float>({batch_size, time_steps});
    }
    };
};