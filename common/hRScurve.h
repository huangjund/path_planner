#pragma once

#include "Heuristic.h"
#include "multibody/SingleForkLiftPlant.h"
#include "common/statespace/SE2State.h"
#include "geometry/ReedsShepp/RSPath4Fork.h"
#include "geometry/ReedsShepp/Node3d.h"

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <memory>

namespace HybridAStar {
namespace Common {
  /**
   * @brief Reeds Shepp planner to get heuristic
   * 
   */
  class hRScurve : Heuristic
  {
  private:
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;  ///< lift car configuration parameters
    SE2State start_;  ///< start point in planning
    SE2State goal_; ///< end point in planning
  public:
    // how to implent the start_ and goal_ into base class?
    hRScurve() = default;
    hRScurve(SE2State &start, SE2State &goal);
    ~hRScurve() = default;

    hRScurve(const hRScurve &) = delete;
    hRScurve &operator=(const hRScurve &) = delete;
    void setStart(SE2State &start);
    void setGoal(SE2State &goal);
    void setStartGoal(SE2State &start, SE2State &goal);

    /**
     * @brief Get the Distance between \p start_ and \p goal_
     * 
     * @return double 
     */
    double getDistance();
  };
} // namespace Common
} // namespace HybridAStar
