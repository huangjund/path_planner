#pragma once

#include "Heuristic.h"
#include "common/parameters/parameters.h"
#include "common/statespace/SE2State.h"
#include "geometry/ReedsShepp/RSPath4Fork.h"

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <memory>

namespace HybridAStar {
namespace Common {
  class hRScurve : public Geometry::RSPath4Fork, public Heuristic
  {
  public:
    hRScurve() = default;
    hRScurve(const SE2StatePtr&,
             const SE2StatePtr&);
    ~hRScurve() = default;

    hRScurve(const hRScurve &) = delete;
    hRScurve &operator=(const hRScurve &) = delete;
    
    virtual double getDistance() override;
  };
} // namespace Common
} // namespace HybridAStar
