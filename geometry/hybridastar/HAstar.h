#pragma once

#include "geometry/Planner.h"
#include "geometry/RRTxstatic/RRTXstatic.h"
//#include "geometry/dubins/dubins.h"
#include "geometry/ReedsShepp/RSPath4Fork.h"
#include "common/statespace/SE2HAStarState.h"
#include "common/statespace/GridState.h"
#include "common/map/planningMap.h"
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
  using Common::SE2HAStarState;
  using Common::GridState;
  using Common::CollisionDetection;
  using Common::hRScurve;
  // using Common::hRRTx;
  using Common::hAStar;
  using Common::PlanningMap;

  class HAstar final: public CompoundPlanner {
   private:
    using PlanningMapPtr = std::shared_ptr<PlanningMap<SE2HAStarState>>;

    Common::CollisionDetectionPtr configSpace_;
    PlanningMapPtr pMap_;  // map used for planning
    unique_ptr<hRScurve> rsPlanner_;
    //unique_ptr<hRRTx> rrtxPlanner_;
    unique_ptr<hAStar> aStarPlanner_;
    
    int defaultIter = 3000;
   public:
    HAstar() = default;
    explicit HAstar(const Common::SE2StatePtr&,
                    const Common::SE2StatePtr&,
                    const PlanningMapPtr&,
                    const Common::CollisionDetectionPtr&);
    virtual ~HAstar() override = default;

    HAstar(const HAstar &) = delete;
    HAstar &operator=(const HAstar &) = delete;

    shared_ptr<SE2State> solve(); // override
    void updateHeuristic(SE2State &,SE2State &);
    shared_ptr<SE2State> dubinsShot(shared_ptr<SE2State>, const SE2State&);
    shared_ptr<SE2State> ReedsShepp(shared_ptr<SE2State>, const SE2State&);
    
    // this function can only be used after the tree has been constructed
    void setStart(Common::SE2StatePtr);
  };
} // namespace Geometry
} // namespace HybridAstar