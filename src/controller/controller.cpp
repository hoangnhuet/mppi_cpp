#include <stdint.h>
#include <chrono>
#include "controller/controller.hpp"
#include "utils/utils.hpp"

namespace nav2_mppi_controller
{
    void MPPIController::cleanup()
    {
        optimizer_.shutdown();
    }
    Twist MPPIController::computeVelocityCommands(
        const Pose &robot_pose,
        const Twist &robot_speed,
        const Path &plan)
    {
        auto control = optimizer_.evalControl(robot_pose, robot_speed, plan);
        return control;
    }
}