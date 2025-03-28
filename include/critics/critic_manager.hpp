#include <vector>
#include <memory>
#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "utils/costmap2d.hpp"

namespace mppi
{

class CriticManager
{
public:
    CriticManager() = default;
    ~CriticManager() = default;

    void evalTrajectories(CriticData &data) const
    {
        for (const auto &critic : critics_) {
            critic->score(data);
        }
    }

    void loadCritics();

private:
    std::shared_ptr<Costmap2D> costmap_;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics_;
};

}  // namespace mppi