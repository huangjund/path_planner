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

  class HAstar : Planner {
  private:
    shared_ptr<SE2State> start_;
    SE2State goal_;
    shared_ptr<CollisionDetection> configSpace_;
    unique_ptr<hRScurve> rsPlanner_;
    //unique_ptr<hRRTx> rrtxPlanner_;
    unique_ptr<hAStar> aStarPlanner_;
    shared_ptr<Map<SE2State>> pMap_;  // map used for planning
    // TODO: set this as a parameter
    int defaultIter = 3000;
  public:
    HAstar() = default;
    explicit HAstar(shared_ptr<SE2State>,SE2State &,shared_ptr<Map<SE2State>>,shared_ptr<CollisionDetection>&);
    ~HAstar() = default;

    HAstar(const HAstar &) = delete;
    HAstar &operator=(const HAstar &) = delete;

    // TODO: return some prompts
    shared_ptr<SE2State> solve();
    void updateHeuristic(SE2State &,SE2State &);
    shared_ptr<SE2State> dubinsShot(shared_ptr<SE2State>, const SE2State&);
    shared_ptr<SE2State> ReedsShepp(shared_ptr<SE2State>, const SE2State&);
    
    SE2State &getStart();
    SE2State &getGoal();
    // this function can only be used after the tree has been constructed
    void setStart(std::shared_ptr<SE2State>);
  };
} // namespace Geometry
} // namespace HybridAstar