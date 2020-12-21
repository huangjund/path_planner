#include "hAStar.h"
#include "common/PlannerTerminationCondition.h"

namespace HybridAStar
{
namespace Common
{
  hAStar::hAStar(const GridStatePtr& start,
                 const GridStatePtr& goal,
                 const Common::CollisionDetectionPtr& config):
                 AStar(start,goal,config){}

  double hAStar::getDistance() {
    PlannerTerminationCondition ptc([]()->bool{return true;})
    solve(ptc);
    return goal_->getG();
  }
    
} // namespace Common
  
} // namespace HybridAstar