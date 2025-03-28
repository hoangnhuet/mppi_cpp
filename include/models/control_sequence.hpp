#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Control
 * @brief A set of controls
 */
struct Control
{
  float v;
  float w;
};

/**
 * @struct mppi::models::ControlSequence
 * @brief A control sequence over time (e.g. trajectory)
 */
struct ControlSequence
{
  xt::xtensor<float, 1> v;
  xt::xtensor<float, 1> w;

  void reset(unsigned int time_steps)
  {
    v = xt::zeros<float>({time_steps});
    w = xt::zeros<float>({time_steps});
  }
};

}  // namespace mppi::models