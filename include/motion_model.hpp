#include "models/control_sequence.hpp"
#include "models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{
    class MotionModel
    {
        public:
            MotionModel();
            ~MotionModel();
            virtual void predict(models::State& state, const models::ControlSequence &controls, float dt);
            virtual void applyConstraint(models::ControlSequence &);
            virtual bool isHolonomic();

    };

    class DiffDriveMotionModel : public MotionModel
    {
        public:
            bool isHolonomic() override
            {
                return false;
            }
    };
};