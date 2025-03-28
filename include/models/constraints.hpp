namespace mppi::models
{

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints
{
  float v;
  float w;
};

/**
 * @struct mppi::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 */
struct SamplingStd
{
  float v;
  float w;
};

}  // 