#pragma once

#include "Planner.h"

namespace HybridAStar {
namespace Geometry {
  class HAstar : Planner {
  private:
    
  public:
    HAstar();
    ~HAstar();
    void solve();
  };
} // namespace Geometry
} // namespace HybridAstar