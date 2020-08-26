#pragma once

#include "Heuristic.h"
#include "multibody/SingleForkLiftPlant.h"
#include "common/statespace/SE2State.h"

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <memory>

namespace HybridAStar {
namespace Common {
  class hRScurve : Heuristic
  {
  private:
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;
    SE2State start_;
    SE2State goal_;
  public:
    // TODO: this state can be put into base class using dynamic binding
    // how to implent the start_ and goal_ into base class?
    hRScurve() = default;
    hRScurve(SE2State &start, SE2State &goal);
    ~hRScurve() = default;

    hRScurve(const hRScurve &) = delete;
    hRScurve &operator=(const hRScurve &) = delete;
    void setStart(SE2State &start);
    void setGoal(SE2State &goal);
    void setStartGoal(SE2State &start, SE2State &goal);
    double getDistance();
  };
} // namespace Common
} // namespace HybridAStar
