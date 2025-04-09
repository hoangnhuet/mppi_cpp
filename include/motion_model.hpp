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
            virtual void predict(models::State& state);
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
            void predict(models::State& state, const models::ControlSequence &controls, float dt) override
            {
            using namespace xt::placeholders;  // NOLINT
            xt::noalias(xt::view(state.v, xt::all(), xt::range(1, _))) =
            xt::view(state.cv, xt::all(), xt::range(0, -1));

            xt::noalias(xt::view(state.w, xt::all(), xt::range(1, _))) =
            xt::view(state.cw, xt::all(), xt::range(0, -1));
            }
    };
}