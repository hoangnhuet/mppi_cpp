#include <string>
#include <memory>
#include"models/constraints.hpp"
#include"utils/utils.hpp"
#include"optimizer.hpp"
#include"utils/costmap2d.hpp"

namespace nav2_mppi_controller
{
    using namespace mppi;
    class MPPIController
    {
        public:
            MPPIController();
            ~MPPIController();
            void cleanup();
            void reset();
            Twist computeVelocityCommands(
                const Pose &robot_pose,
                const Twist &robot_speed,
                const Path &plan);
        protected:
            Optimizer optimizer_;
            std::shared_ptr<Costmap2D> costmap_;
    };
}