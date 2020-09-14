#include "bspline.h"

using namespace HybridAStar;

BSpline::BSpline():Smoother(){}

BSpline::~BSpline() {
  ctrlPointSet.clear();
  trajPointSet.clear();
  splineOrder.clear();
}

void BSpline::tracePath(const std::shared_ptr<Common::SE2State> node) {
  Smoother::tracePath(node);  // trace a whole path
  auto pathlength = path_.size();

  Vector2D point;
  std::vector<Vector2D> traj;
  int i;
  for (i = 0; i < pathlength-1; ++i)
  {
    point[0] = path_[i]->getX(); point[1] = path_[i]->getY();
    traj.push_back(point);
    if (isCusp(i)) {  // if the i th point is a direction changing point
      trajPointSet.push_back(traj);
      traj.clear();
      traj.push_back(point);
    }
  }
  point[0] = path_[i]->getX(); point[1] = path_[i]->getY();
  traj.push_back(point);
  trajPointSet.push_back(traj); // push back the last piece of trajectory
}

// automatically set the base spline order to 3 or 2
void BSpline::initializeSplineOrders(bool autogen) {
  unsigned int order = 3;
  auto size = trajPointSet.size();
  while (size--) {
    splineOrder.push_back(order);
  }
}

// set control points for quasi-uniform b spline
void BSpline::setCtrlPoints(bool autogen) {
  auto size = trajPointSet.size();
  for (size_t i = 0; i < size; ++i)
  {
    auto n = trajPointSet[i].size() - 1;
    auto &k = splineOrder[i];
    if(n < k){
      assert(n==2);
      k = n;  // constraint for quasi-uniform distributed control points, at least 3 points
    }
    auto ctrlpSize = n - k + 2; // size for quasi-uniform b spline

    // set control points for one piece of trajectory
    ctrlPoint temp;
    for (size_t j = 0; j < ctrlpSize; ++j)
    {
      if(j == 0) {  // if at the first point
        temp.value = 0;
        temp.multiplicity = k+1;
      }
      else if(j == ctrlpSize-1) { // if at the last point
        temp.value = ctrlpSize-1;
        temp.multiplicity = k+1;
      }
      else {  // if at the medium points
        temp.value = j;
        temp.multiplicity = 1;
      }
      ctrlPointSet[i].push_back(temp);
    }
  }
  
}

