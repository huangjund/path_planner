#ifndef _HEURISTIC_ASTAR_H
#define _HEURISTIC_ASTAR_H

#include "Heuristic.h"
#include "common/statespace/GridState.h"
#include "common/statespace/SE2State.h"
#include "common/collision/clsDetection.h"
#include "common/map/map.h"

#include <memory>
#include <boost/heap/binomial_heap.hpp>

namespace HybridAStar{
namespace Common{
  class hAStar : public Heuristic {
  private:
    std::shared_ptr<GridState> start_;
    GridState goal_;
    std::shared_ptr<CollisionDetection> config_;
    std::unique_ptr<Map<GridState>> pMap_;
  public:
    hAStar() = default;
    hAStar(const std::shared_ptr<GridState>&,const GridState&,std::shared_ptr<CollisionDetection>&);
    ~hAStar() = default;

    hAStar(const hAStar &) = delete;
    hAStar &operator=(const hAStar&) = delete;

    void setStart(const std::shared_ptr<GridState>&);
    void setGoal(const GridState& goal);
    void setStartGoal(const std::shared_ptr<GridState>&, const GridState&);
    void setRawmap(const nav_msgs::OccupancyGrid::Ptr &);
    void setSS();
    std::unique_ptr<Map<GridState>>& returnMap();
    double getDistance();
  };
} // namespace Common
} // namespace HybridAstar
#endif
