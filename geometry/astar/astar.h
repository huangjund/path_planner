#ifndef _HYBRID_A_STAR_A_STAR_H_
#define _HYBRID_A_STAR_A_STAR_H_

#include "common/statespace/State.h"
#include "common/statespace/GridState.h"
#include "common/map/planningMap.h"
#include "common/collision/clsDetection.h"
#include "common/parameters/parameters.h"
#include "geometry/Planner.h"

#include "nav_msgs/OccupancyGrid.h"

#include <memory>
#include <utility>
#include <boost/heap/binomial_heap.hpp>

namespace HybridAStar {
namespace Geometry {

struct CompareNodes {
  bool operator()(const std::shared_ptr<GridState> lhs, const std::shared_ptr<GridState> rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

class AStar : public Planner {
 public:
  AStar() = default;
  AStar(const Common::StatePtr&, 
        const Common::StatePtr&,
        const Common::CollisionDetectionPtr&);
  virtual ~AStar() override = default;

  AStar(const AStar& ) = delete;
  AStar& operator=(const AStar&) = delete;

  void setMap(const nav_msgs::OccupancyGrid::Ptr&);
  void setSS(unsigned int count);
  std::unique_ptr<Common::PlanningMap<Common::GridState>>& returnMap();

  virtual void solve(const Common::PlannerTerminationCondition&) override;

 private:
  std::unique_ptr<Common::PlanningMap<Common::GridState>> pMap_;
  Common::CollisionDetectionPtr config_;
};
} // namespace Geometry
} // namespace HybridAStar

#endif