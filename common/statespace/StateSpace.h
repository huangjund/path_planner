#pragma once

#include "ompl/util/ClassForward.h"

#include "State.h"

namespace HybridAStar {
namespace Common {
  OMPL_CLASS_FORWARD(StateSpace);
  class StateSpace {
  public:
    StateSpace();
    ~StateSpace();
  };
} // namespace Common
} // namespace HybridAStar