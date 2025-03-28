#include"critics/critic_function.hpp"
#include"models/state.hpp"
namespace mppi::critics
{
    class GoalCritic : public CriticFunction
    {
        public:
            GoalCritic();
            ~GoalCritic();
            void score(CriticData &data) override;
        protected:
            unsigned int power_{0};
            float weight_{0};
            float threshold_to_consider_{0};
    };
    
} // namespace mppi::critics

