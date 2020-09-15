#include "bspline.h"

using namespace HybridAStar;

BSpline::BSpline():Smoother(){}

BSpline::~BSpline() {
  ctrlPointSet.clear();
  trajPointSet.clear();
  splineOrder.clear();
}

bool BSpline::isCusp(int i) {
  bool revi   = path_[i]->getPrim() < 3 ? true : false;
  bool revip1 = path_[i + 1]->getPrim() < 3 ? true : false;

  // if the point is at the direction changing point
  // return true
  if(revi != revip1) return true;
  return false;
}

void BSpline::tracePath(const std::shared_ptr<Common::SE2State> node) {
  Smoother::tracePath(node);  // trace a whole path
  auto pathlength = path_.size();

  Vector2D point;
  std::vector<Vector2D> traj;
  int i;
  // initialize trajectory point set
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

void BSpline::smoothPath(float width, float height) {
  setWidthHeight(width,height);
  unsigned int iter = 0;
  unsigned int maxIter = 50;
  // for every piece of trajectories
  size_t i = 0;
  for(auto traj = trajPointSet.begin(); traj != trajPointSet.end(); traj++) {
    if (traj->size() >= 5){
      while (iter < maxIter) {
        // optimize every proper points in every trajectories
        for (auto p = traj->begin()+2; p != traj->end()-2; p++)
        {
          Vector2D xim2((p-2)->getX(), (p-2)->getY());
          Vector2D xim1((p-1)->getX(), (p-1)->getY());
          Vector2D xi(p->getX(), p->getY());
          Vector2D xip1((p+1)->getX(), (p+1)->getY());
          Vector2D xip2((p+2)->getX(), (p+2)->getY());
          Vector2D correction;

          correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
          if (!isOnGrid(xi + correction)) {continue;}

          correction = correction - curvatureTerm(xim1, xi, xip1);
          if (!isOnGrid(xi + correction)) {continue;};

          float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;

          xi = xi + alpha * correction/totalWeight; // gradient descent
          p->setX(xi.getX());
          p->setY(xi.getY());
        }
        iter++;
      }
    }
    for (auto p = traj->begin(); p != traj->end()-1; ++p,++i) {
      path_[i]->setX(p->getX());
      path_[i]->setY(p->getY());
    }
  }
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
      assert(n>=2);
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

void BSpline::clearPath() {
  Smoother::clearPath();
  splineOrder.clear();
  ctrlPointSet.clear();
  trajPointSet.clear();
}