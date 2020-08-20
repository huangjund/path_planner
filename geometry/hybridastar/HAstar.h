#pragma once

#include "geometry/Planner.h"
#include "geometry/RRTxstatic/RRTXstatic.h"
#include "geometry/dubins/dubins.h"
#include "common/statespace/SE2State.h"
#include "common/statespace/GridState.h"
#include "common/map/map.h"
#include "common/hRRTx.h"
#include "common/hRScurve.h"
#include "common/collision/clsDetection.h"

#include <memory>

namespace HybridAStar {
namespace Geometry {
  using std::unique_ptr;
  using std::shared_ptr;
  using Common::SE2State;
  using Common::GridState;
  using Common::CollisionDetection;
  using Common::hRScurve;
  using Common::hRRTx;
  using Common::Map;

  class HAstar : Planner {
  private:
    SE2State start_;
    SE2State goal_;
    CollisionDetection &configSpace_;
    unique_ptr<hRScurve> rsPlanner_;
    unique_ptr<hRRTx> rrtxPlanner_;
    unique_ptr<Map<GridState>> cMap_; // map used for collision
    unique_ptr<Map<SE2State>> pMap_;  // map used for planning
    int defaultIter = 3000;
    
  public:
    explicit HAstar(SE2State &,SE2State &,unique_ptr<Map<GridState>>&,unique_ptr<Map<SE2State>>&,CollisionDetection &);
    ~HAstar();

    HAstar(const HAstar &) = delete;
    HAstar &operator=(const HAstar &) = delete;

    // TODO: return some prompts
    SE2State* solve();
    void updateHeuristic(SE2State &,SE2State &);
    SE2State* dubinsShot(SE2State&, const SE2State&);
    SE2State &getStart();
    SE2State &getGoal();
  };
} // namespace Geometry
} // namespace HybridAstar