#pragma once

#include "ompl/util/ClassForward.h"
#include "statespace/StateSpace.h"

#include <functional>
#include <memory>

namespace HybridAStar {
namespace Common {
  OMPL_CLASS_FORWARD(Map);
  class Map {
  public:
    Map() = default;
    explicit Map(StateSpacePtr space);
    ~Map();
    const StateSpacePtr& getStateSpace() const;
    unsigned int getDimensions() const;
  };
} // namespace Common
} // namespace HybridAStar