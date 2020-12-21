#include "hRScurve.h"

namespace HybridAStar {
namespace Common {
  hRScurve::hRScurve(const SE2StatePtr& start,
                     const SE2StatePtr& goal): 
                     RSPath4Fork(start, goal){
}

  double hRScurve::getDistance() {
    double step_size = 0.2;
    setStepKappa(1/Common::ForkProperty::rad_, 0.1)

    return ShortestRSPlength();
  }
} // namespace Common
} // namespace HybridAStar
