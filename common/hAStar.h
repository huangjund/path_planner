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
  /**
   * @brief A star planner as a heuristic
   * 
   */
  class hAStar : public Heuristic {
  private:
    std::shared_ptr<GridState> start_;  ///< start point in planning
    GridState goal_;  ///< end point in planning
    std::shared_ptr<CollisionDetection> config_;  ///< configuration space
    std::unique_ptr<Map<GridState>> pMap_;  ///< planning map
  public:
    hAStar() = default;
    hAStar(const std::shared_ptr<GridState>&,
          const GridState&,
          std::shared_ptr<CollisionDetection>&);
    ~hAStar() = default;

    hAStar(const hAStar &) = delete;
    hAStar &operator=(const hAStar&) = delete;

    void setStart(const std::shared_ptr<GridState>&);
    void setGoal(const GridState& goal);
    void setStartGoal(const std::shared_ptr<GridState>&, 
                      const GridState&);
    void setRawmap(const nav_msgs::OccupancyGrid::Ptr &);

    /**
     * @brief set state space
     * 
     */
    void setSS();
    std::unique_ptr<Map<GridState>>& returnMap();

    /**
     * @brief Get the Distance between \p start_ and \p goal_
     * 
     * @return double 
     */
    double getDistance();
  };
} // namespace Common
} // namespace HybridAstar
#endif
