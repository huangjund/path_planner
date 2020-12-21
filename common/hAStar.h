#ifndef _HEURISTIC_ASTAR_H
#define _HEURISTIC_ASTAR_H

#include "Heuristic.h"
#include "common/statespace/GridState.h"
#include "geometry/astar/astar.h"

#include <memory>

namespace HybridAStar{
namespace Common{
  class hAStar : public AStar, public Heuristic {
  public:
    hAStar() = default;
    hAStar(const GridStatePtr&,
           const GridStatePtr&,
           const Common::CollisionDetectionPtr&);
    virtual ~hAStar() override = default;

    hAStar(const hAStar &) = delete;
    hAStar &operator=(const hAStar&) = delete;

    virtual double getDistance() override;
  };
} // namespace Common
} // namespace HybridAstar
#endif
