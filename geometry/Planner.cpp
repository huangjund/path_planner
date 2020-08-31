#include "Planner.h"

namespace HybridAStar{
namespace Geometry{
  Planner::Planner(std::shared_ptr<Common::SE2State> start,Common::SE2State &goal):
    start_(start),goal_(goal),carPlant_(std::make_unique<Multibody::SingleForkLiftPlant>()){
  }
} // namespace Geometry
} // namespace HybridAStar
