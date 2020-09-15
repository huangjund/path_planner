#ifndef _HYBRID_A_STAR_B_SPLINE_H
#define _HYBRID_A_STAR_B_SPLINE_H

#include "smoother.h"
#include "vector2d.h"
#include "../common/statespace/SE2State.h"
#include "bsplinebasis.h"

#include <vector>
#include <cassert>

namespace HybridAStar
{
class BSpline : public Smoother {
  public:
    struct ctrlPoint {
      double value;
      int multiplicity = 1;
    };
  private:
    std::vector<std::vector<ctrlPoint>> ctrlPointSet;
    std::vector<std::vector<Vector2D>> trajPointSet;
    std::vector<unsigned int> splineOrder;
  public:
    BSpline();
    virtual ~BSpline();
    
    virtual void tracePath(const std::shared_ptr<Common::SE2State> node);
    virtual void smoothPath(float width, float height);
    virtual bool isCusp(int i);
    virtual void clearPath();
    void initializeSplineOrders(bool autogen = true);
    void setCtrlPoints(bool autogen = true);
}; // class bspline
} // namespace HybridAStar

#endif