#include <cstddef>
#include "models/constraints.hpp"

namespace mppi::models
{

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 */
struct OptimizerSettings
{
  models::ControlConstraints base_constraints{0, 0}; //v and w
  models::ControlConstraints constraints{0, 0}; 
  models::SamplingStd sampling_std{0, 0}; // v and w
  float model_dt{0};
  float temperature{0};
  float gamma{0};
  unsigned int batch_size{0};
  unsigned int time_steps{0};
  unsigned int iteration_count{0};
  bool shift_control_sequence{false};
  size_t retry_attempt_limit{0};
};

}  // namespace mppi::models