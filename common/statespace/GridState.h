#pragma once

#include "State.h"

#include <iostream>

namespace HybridAStar {
namespace Common {
  class GridState : State{
  private:
    unsigned int dimension = 2;
  public:
    GridState();
    ~GridState();
    double distance (const State *state1, const State *state2) const;
    void freeState (State *state) const;
    void printState (const State *state, std::ostream &out) const;
  };
}
}