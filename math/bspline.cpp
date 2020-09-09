#include "bspline.h"

using namespace HybridAStar;

BSpline::BSpline():Smoother(){}


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

void BSpline::setSplineOrder(bool autogen) {
  unsigned int order = 3;
  auto size = trajPointSet.size();
  while (size--) {
    splineOrder.push_back(order);
  }
}

void BSpline::setCtrlPoints(bool autogen) {
  auto size = trajPointSet.size();
  for (size_t i = 0; i < size; ++i)
  {
    auto n = trajPointSet[i].size();
    auto &k = splineOrder[i];
    if(n < k-1) 
      k = n+1;  // constraint for asym-equality distributed control points
    auto ctrlpSize = n + k + 1;

    // set control points for one piece of trajectory
    for (size_t j = 0; j < ctrlpSize; ++j)
    {
      if(j < k)
        ctrlPointSet[i].push_back(0);
      else if(j >= n+1)
        ctrlPointSet[i].push_back(10);
      else
        ctrlPointSet[i].push_back((n-k+2)*(j-k+1)/10);
    }
  }
  
}

