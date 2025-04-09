#include"optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{
    using namespace xt::placeholders;  // NOLINT
    using xt::evaluation_strategy::immediate;
    void Optimizer::shutdown()
    {
        noise_generator_.reset();
    }
    Twist Optimizer::evalControl(Pose &robot_pose, Twist &robot_speed, 
    Path &plan)
    {
        prepare(robot_pose, robot_speed, plan);
        optimize();
        auto control = getControlFromSequenceAsTwist();
        if(settings_.shift_control_sequence)
        {
            shiftControlSequence();
        }
        return control;
    }
    void Optimizer::optimize()
    {
    for (size_t i = 0; i < settings_.iteration_count; ++i) {
        generateNoisedTrajectories();
        critic_manager_.evalTrajectoriesScores(critic_data_);
        updateControlSequence();
    }
    }
    void Optimizer::prepare(const Pose &robot_pose, const Twist &robot_speed, const Path &plan)
    {
        state_.pose = robot_pose;
        state_.speed = robot_speed;
        cost_.fill(0);
        critic_data_.motion_model = motion_model_;
    }
    void Optimizer::shiftControlSequence()
    {
        using namespace xt::placeholders;
        control_sequence_.v = xt::roll(control_sequence_.v, -1);
        control_sequence_.w = xt::roll(control_sequence_.w, -1);
        xt::view(control_sequence_.v, -1) =
            xt::view(control_sequence_.v, -2);

        xt::view(control_sequence_.w, -1) =
            xt::view(control_sequence_.w, -2);

    }
    void Optimizer::generateNoisedTrajectories()
    {
        noise_generator_.setNoisedControl(state_, control_sequence_);
        noise_generator_.generateNextNoise();
        updateStateVelocities(state_);
        integrateStateVelocities(generated_trajectories_, state_);
    }
    void Optimizer::applyControlSequenceConstraints()
    {
        auto &s = settings_;
        control_sequence_.v = xt::clip(control_sequence_.v, -s.constraints.v, s.constraints.v);
        control_sequence_.w = xt::clip(control_sequence_.w, -s.constraints.w, s.constraints.w);
    }


    void Optimizer::updateStateVelocities(models::State & state) const
    {
    updateInitialStateVelocities(state);
    propagateStateVelocitiesFromInitials(state);
    }

    void Optimizer::updateInitialStateVelocities(
    models::State & state) const
    {
    xt::noalias(xt::view(state.v, xt::all(), 0)) = state.speed.linear;
    xt::noalias(xt::view(state.w, xt::all(), 0)) = state.speed.angular;
    }

    void Optimizer::propagateStateVelocitiesFromInitials(
      models::State & state) const
    {
      motion_model_->predict(state);
    }

    void Optimizer::integrateStateVelocities(
      xt::xtensor<float, 2> & trajectory,
      const xt::xtensor<float, 2> & sequence) const
    {
      float initial_yaw = state_.pose.yaw;

      const auto v = xt::view(sequence, xt::all(), 0);
      const auto w = xt::view(sequence, xt::all(), 1);

      auto traj_x = xt::view(trajectory, xt::all(), 0);
      auto traj_y = xt::view(trajectory, xt::all(), 1);
      auto traj_yaws = xt::view(trajectory, xt::all(), 2);

      xt::noalias(traj_yaws) = xt::cumsum(w * settings_.model_dt, 0) + initial_yaw;

      auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
      auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

      const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

      xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
      xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
      xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
      xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

      auto && dx = xt::eval(v * yaw_cos);
      auto && dy = xt::eval(v * yaw_sin);

      xt::noalias(traj_x) = state_.pose.x + xt::cumsum(dx * settings_.model_dt, 0);
      xt::noalias(traj_y) = state_.pose.y + xt::cumsum(dy * settings_.model_dt, 0);
    }

    void Optimizer::integrateStateVelocities(
      models::Trajectories & trajectories,
      const models::State & state) const
    {
      const float initial_yaw = state.pose.yaw;

      xt::noalias(trajectories.yaw) =
        xt::cumsum(state.w * settings_.model_dt, 1) + initial_yaw;

      const auto yaws_cutted = xt::view(trajectories.yaw, xt::all(), xt::range(0, -1));

      auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaw.shape());
      auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaw.shape());
      xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
      xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
      xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
      xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

      auto && dx = xt::eval(state.v * yaw_cos);
      auto && dy = xt::eval(state.v * yaw_sin);


      xt::noalias(trajectories.x) = state.pose.x +
        xt::cumsum(dx * settings_.model_dt, 1);
      xt::noalias(trajectories.y) = state.pose.y +
        xt::cumsum(dy * settings_.model_dt, 1);
    }

    void Optimizer::updateControlSequence()
    {
      auto & s = settings_;
      auto bounded_noises_vx = state_.cv - control_sequence_.v;
      auto bounded_noises_wz = state_.cw - control_sequence_.w;
      xt::noalias(cost_) +=
        s.gamma / powf(s.sampling_std.v, 2) * xt::sum(
        xt::view(control_sequence_.v, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
      xt::noalias(cost_) +=
        s.gamma / powf(s.sampling_std.w, 2) * xt::sum(
        xt::view(control_sequence_.w, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);


      auto && costs_normalized = cost_ - xt::amin(cost_, immediate);
      auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
      auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
      auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

      xt::noalias(control_sequence_.v) = xt::sum(state_.cv * softmaxes_extened, 0, immediate);
      xt::noalias(control_sequence_.w) = xt::sum(state_.cw * softmaxes_extened, 0, immediate);
    }
    Twist Optimizer::getControlFromSequenceAsTwist()
    {
        Twist control;
        control.linear = xt::view(control_sequence_.v, -1);
        control.angular = xt::view(control_sequence_.w, -1);
        return control;
    }

}