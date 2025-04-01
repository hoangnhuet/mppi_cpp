#include <string>
#include <memory>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>



#include "models/optimizer_settings.hpp"
#include "motion_model.hpp"
#include "critic_manager.hpp"
#include "models/state.hpp"
#include "models/trajectories.hpp"
#include "models/path.hpp"
#include "tools/noise_generator.hpp"

#include "utils/utils.hpp"

namespace mppi
{
    class Optimizer
    {
        public:
            Optimizer();
            ~Optimizer(){shutdown();};
            void shutdown();
            /**
             * @brief compute control
             * @param robot_pose pose at given time
             * @param robot_speed speed at given time
             * @param plan Path to follow
             * @return Twist control to apply
             */
            Twist evalControl(Pose &robot_pose, Twist &robot_speed, 
            Path &plan);
            void setSpeedLimit(double speed_limit);

        protected:
            void optimize();
            /**
             * @brief prepare state info on new request for tracjectory rollouts
             * @param robot_pose pose at given time
             * @param robot_speed speed at given time
             * @param plan Path to follow
             */
            void prepare(const Pose &robot_pose, const Twist &robot_speed, const Path &plan);
            /**
             * @brief shift optimal control sequence
             */
            void shiftControlSequence();
            /**
             * @brief update generated traj with noise
             */
            void generateNoisedTrajectories();
            /**
             * @brief apply vehicle constraints to control sequence
             */
            void applyControlSequenceCOnstraints();
            /**
             * @brief update velocity in state
             */
            void updateStateVelocities(models::State &state) const;
            /**
             * @brief update initial state velocities
             */
            void updateInitialStateVelocities(models::State &state) const;
            /**
             * @brief predict state using model
            */
            void propagateStateVelocitiesFromInitials(models::State &state) const;
            /**
             * @brief Rollout velocities in state to poses
             * @param trajectories to rollout
             * @param state fill state
             */
            void integrateStateVelocities(
                models::Trajectories & trajectories,
                const models::State & state) const;

            /**
             * @brief Rollout velocities in state to poses
             * @param trajectories to rollout
             * @param state fill state
             */
            void integrateStateVelocities(
                xt::xtensor<float, 2> & trajectories,
                const xt::xtensor<float, 2> & state) const;

            /**
             * @brief Update control sequence with state controls weighted by costs
             * using softmax function
             */
            void updateControlSequence();
        protected:
            models::OptimizerSettings settings_;
            std::shared_ptr<MotionModel> motion_model_;
            Costmap2D costmap_;
            CriticManager critic_manager_;
            NoiseGenerator noise_generator_;
            std::array<mppi::models::Control, 4> control_history_;
            models::State state_;
            models::Trajectories trajectories_;
            models::Path path_;
            xt::xtensor<float, 1> cost_;
            CriticData critic_data_ = {state_, trajectories_, path_, cost_, settings_.model_dt, motion_model_};

            



    };
}