#pragma once

#include "geometry/Planner.h"
#include "geometry/RRTxstatic/RRTXstatic.h"
//#include "geometry/dubins/dubins.h"
#include "geometry/ReedsShepp/RSPath4Fork.h"
#include "geometry/ReedsShepp/Node3d.h"
#include "common/statespace/SE2State.h"
#include "common/statespace/GridState.h"
#include "common/map/map.h"
//#include "common/hRRTx.h"
#include "common/hAStar.h"
#include "common/hRScurve.h"
#include "common/collision/clsDetection.h"

#include <memory>
#include <iostream>
#include <vector>

namespace HybridAStar {
namespace Geometry {
  using std::unique_ptr;
  using std::shared_ptr;
  using Common::SE2State;
  using Common::GridState;
  using Common::CollisionDetection;
  using Common::hRScurve;
  // using Common::hRRTx;
  using Common::hAStar;
  using Common::Map;

  /**
   * @brief Hybrid A Star Planner
   * 
   */
  class HAstar : Planner {
  private:
    shared_ptr<SE2State> start_;  ///< start point
    SE2State goal_; ///< goal point
    shared_ptr<CollisionDetection> configSpace_;  ///< configuration space
    unique_ptr<hRScurve> rsPlanner_;  ///< heuristic reeds shepp planner
    //unique_ptr<hRRTx> rrtxPlanner_;
    unique_ptr<hAStar> aStarPlanner_; ///< heuristic a star planner
    shared_ptr<Map<SE2State>> pMap_;  ///< planning map
    // TODO: set this as a parameter
    int defaultIter = 3000; ///< searching iterations(searching depth)
  public:
    HAstar() = default;
    explicit HAstar(shared_ptr<SE2State>,SE2State &,shared_ptr<Map<SE2State>>,shared_ptr<CollisionDetection>&);
    ~HAstar() = default;

    HAstar(const HAstar &) = delete;
    HAstar &operator=(const HAstar &) = delete;

    /**
     * @brief 
     * 
     * @return shared_ptr<SE2State> the end point of found path
     */
    shared_ptr<SE2State> solve();

    /**
     * @brief update heuristic of current point
     * 
     * @param current current searching point
     * @param goal 
     */
    void updateHeuristic(SE2State &current,SE2State &goal);

    /**
     * @brief get shortest dubins curve from current to goal
     * 
     * @param current current position
     * @param goal goal position
     * @return shared_ptr<SE2State> return last point on generated dubins curve
     */
    shared_ptr<SE2State> dubinsShot(shared_ptr<SE2State> current, const SE2State& goal);

    /**
     * @brief get shortest reeds shepp curve from current to goal
     * 
     * @param current current position
     * @param goal goal position
     * @return shared_ptr<SE2State> return last point on generated reeds shepp curve
     */
    shared_ptr<SE2State> ReedsShepp(shared_ptr<SE2State> current, const SE2State& goal);
    
    SE2State &getStart();
    SE2State &getGoal();
    
    /**
     * @brief Set the Start point, this function can only be used after the tree has been constructed
     * 
     */
    void setStart(std::shared_ptr<SE2State>);
  };
} // namespace Geometry
} // namespace HybridAstar