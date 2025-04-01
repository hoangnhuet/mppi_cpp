#include"critics/critic_function.hpp"
#include"models/state.hpp"
#include"critics/params.hpp"

namespace mppi::critics
{
    class ConstraintCritic : public mppi::critics::CriticFunction
    {
        public:
            ConstraintCritic();
            ~ConstraintCritic();
            void score(mppi::CriticData &data) override;
        protected:
            unsigned int power_{0};
            float weight_{0};
            float v_max;
            float v_min;
    };
}