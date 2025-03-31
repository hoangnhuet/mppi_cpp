#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "models/optimizer_settings.hpp"
#include "models/control_sequence.hpp"
#include "models/state.hpp"

namespace mppi
{
    class NoiseGenerator
    {
        public:
            /**
             * @brief signal that controller is ready to generate noise for thr next iteration
             */
            void generateNextNoise();
            /**
             * @brief set noised control to state controls 
             * @return v,w
             */
            void setNoisedControl(models::State &state, models::ControlSequence &controls);
            /** 
             * @brief reset 
             */
            void reset();
        protected:
            void noiseThread();

            /** 
             * @brief generate random controls by gaussian noise in control_sequence
             * @return shape [batch_size, timesteps_, control_dim = 2] 
             */
            void generateNoisedControls();

            xt::xtensor<float, 2> noises_v_;
            xt::xtensor<float, 2> noises_w_;

            mppi::models::OptimizerSettings settings_;

            std::thread noise_thread_;
            std::condition_variable noise_cond_;
            std::mutex noise_lock_;
            bool active_{false}, ready_{false}, regenerate_noises_{false};


    };
}