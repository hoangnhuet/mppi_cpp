#include"tools/noise_generator.hpp"
#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{
    void NoiseGenerator::generateNextNoise()
    {
        {
        std::unique_lock<std::mutex> lock(noise_lock_);
        ready_ = true;
        }
        noise_cond_.notify_all();
    }
    void NoiseGenerator::setNoisedControl(models::State &state, models::ControlSequence &control_sequence)
    {
    std::unique_lock<std::mutex> guard(noise_lock_);

        xt::noalias(state.cv) = control_sequence.v + noises_v_;
        xt::noalias(state.cw) = control_sequence.w + noises_w_;
    }
    
    void NoiseGenerator::reset()
    {
        {
            std::unique_lock<std::mutex> guard(noise_lock_);
            xt::noalias(noises_v_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
            xt::noalias(noises_w_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
            ready_ = true;
        }
    if (regenerate_noises_) {
        noise_cond_.notify_all();
    } else {
        generateNoisedControls();
    }

    }

    void NoiseGenerator::noiseThread()
    {
    do {
        std::unique_lock<std::mutex> guard(noise_lock_);
        noise_cond_.wait(guard, [this]() {return ready_;});
        ready_ = false;
        generateNoisedControls();
    } while (active_);

    }

    void NoiseGenerator::generateNoisedControls()
    {
    auto & s = settings_;
    xt::noalias(noises_v_) = xt::random::randn<float>(
        {s.batch_size, s.time_steps}, 0.0f,
        s.sampling_std.v);
    xt::noalias(noises_w_) = xt::random::randn<float>(
        {s.batch_size, s.time_steps}, 0.0f,
        s.sampling_std.w);
    }

}