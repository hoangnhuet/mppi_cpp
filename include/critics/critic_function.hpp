#include <string>
#include <memory>
#include"critics/critic_data.hpp"
#include"utils/utils.hpp"
#include"utils/costmap2d.hpp"
#include"critics/params.hpp"
namespace mppi::critics
{
    struct CollisionCost
    {
        float cost{0};
    };
    class CriticFunction
    {
        public:
            CriticFunction();
            ~CriticFunction();
            virtual void score(CriticData &data);
        protected:
            Costmap2D costmap;
    };
}