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
    void Optimizer::applyControlSequenceCOnstraints()
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
      float initial_yaw = state_.pose.pose.orientation;

      const auto vx = xt::view(sequence, xt::all(), 0);
      const auto vy = xt::view(sequence, xt::all(), 2);
      const auto wz = xt::view(sequence, xt::all(), 1);

      auto traj_x = xt::view(trajectory, xt::all(), 0);
      auto traj_y = xt::view(trajectory, xt::all(), 1);
      auto traj_yaws = xt::view(trajectory, xt::all(), 2);

      xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

      auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
      auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

      const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

      xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
      xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
      xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
      xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

      auto && dx = xt::eval(vx * yaw_cos);
      auto && dy = xt::eval(vx * yaw_sin);

      xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
      xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
    }

    // void Optimizer::integrateStateVelocities(
    //   models::Trajectories & trajectories,
    //   const models::State & state) const
    // {
    //   const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

    //   xt::noalias(trajectories.yaws) =
    //     xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw;

    //   const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

    //   auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
    //   auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
    //   xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
    //   xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
    //   xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
    //   xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

    //   auto && dx = xt::eval(state.vx * yaw_cos);
    //   auto && dy = xt::eval(state.vx * yaw_sin);

    //   if (isHolonomic()) {
    //     dx = dx - state.vy * yaw_sin;
    //     dy = dy + state.vy * yaw_cos;
    //   }

    //   xt::noalias(trajectories.x) = state.pose.pose.position.x +
    //     xt::cumsum(dx * settings_.model_dt, 1);
    //   xt::noalias(trajectories.y) = state.pose.pose.position.y +
    //     xt::cumsum(dy * settings_.model_dt, 1);
    // }

    // xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
    // {
    //   auto && sequence =
    //     xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
    //   auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

    //   xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
    //   xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

    //   if (isHolonomic()) {
    //     xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
    //   }

    //   integrateStateVelocities(trajectories, sequence);
    //   return std::move(trajectories);
    // }

    // void Optimizer::updateControlSequence()
    // {
    //   auto & s = settings_;
    //   auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
    //   auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
    //   xt::noalias(costs_) +=
    //     s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
    //     xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
    //   xt::noalias(costs_) +=
    //     s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
    //     xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

    //   if (isHolonomic()) {
    //     auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
    //     xt::noalias(costs_) +=
    //       s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
    //       xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
    //       1, immediate);
    //   }

    //   auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
    //   auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
    //   auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
    //   auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

    //   xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
    //   xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
    //   if (isHolonomic()) {
    //     xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
    //   }

    //   applyControlSequenceConstraints();
    // }

    // geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
    //   const builtin_interfaces::msg::Time & stamp)
    // {
    //   unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

    //   auto vx = control_sequence_.vx(offset);
    //   auto wz = control_sequence_.wz(offset);

    //   if (isHolonomic()) {
    //     auto vy = control_sequence_.vy(offset);
    //     return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
    //   }

    //   return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
    // }

    // void Optimizer::setMotionModel(const std::string & model)
    // {
    //   if (model == "DiffDrive") {
    //     motion_model_ = std::make_shared<DiffDriveMotionModel>();
    //   } else if (model == "Omni") {
    //     motion_model_ = std::make_shared<OmniMotionModel>();
    //   } else if (model == "Ackermann") {
    //     motion_model_ = std::make_shared<AckermannMotionModel>(parameters_handler_);
    //   } else {
    //     throw std::runtime_error(
    //             std::string(
    //               "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
    //               "or Ackermann"));
    //   }
    // }

    // void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
    // {
    //   auto & s = settings_;
    //   if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    //     s.constraints.vx_max = s.base_constraints.vx_max;
    //     s.constraints.vx_min = s.base_constraints.vx_min;
    //     s.constraints.vy = s.base_constraints.vy;
    //     s.constraints.wz = s.base_constraints.wz;
    //   } else {
    //     if (percentage) {
    //       // Speed limit is expressed in % from maximum speed of robot
    //       double ratio = speed_limit / 100.0;
    //       s.constraints.vx_max = s.base_constraints.vx_max * ratio;
    //       s.constraints.vx_min = s.base_constraints.vx_min * ratio;
    //       s.constraints.vy = s.base_constraints.vy * ratio;
    //       s.constraints.wz = s.base_constraints.wz * ratio;
    //     } else {
    //       // Speed limit is expressed in absolute value
    //       double ratio = speed_limit / s.base_constraints.vx_max;
    //       s.constraints.vx_max = s.base_constraints.vx_max * ratio;
    //       s.constraints.vx_min = s.base_constraints.vx_min * ratio;
    //       s.constraints.vy = s.base_constraints.vy * ratio;
    //       s.constraints.wz = s.base_constraints.wz * ratio;
    //     }
    //   }
    // }

    // models::Trajectories & Optimizer::getGeneratedTrajectories()
    // {
    //   return generated_trajectories_;
    // }


}