#include "bspline.h"

using namespace HybridAStar;

BSpline::BSpline():Smoother(){
  // visualization
  pub = n.advertise<nav_msgs::Path>("/bspline",1);
  path.header.frame_id = "path";
}

BSpline::~BSpline() {
  ctrlPointSet.clear();
  trajPointSet.clear();
  trajDirctionSet.clear();
  splineOrder.clear();
  bpath_.clear();
}

bool BSpline::isCusp(int i) {
  //  as the path_ is from goal to start, we compare i-1 with i
  bool revim1 = path_[i-1]->getPrim() < 3 ? true : false;
  bool revi   = path_[i]->getPrim() < 3 ? true : false;

  // if the point is at the direction changing point
  // return true
  if(revi != revim1) return true;
  return false;
}

// complete function
void BSpline::tracePath(const std::shared_ptr<Common::SE2State> node) {
  Smoother::tracePath(node);  // trace a whole path
  auto pathlength = path_.size();

  Vector2D point;
  std::vector<Vector2D> traj;

  // initialize trajectory point set
  // the first point is not a cusp point
  // path_: 0     ->      n
  //        goal  ->    start
  // trajPointSet : 0     ->    n
  //                goal  ->    start
  int i = pathlength - 1;
  assert(pathlength > 1);
  point[0] = path_[i]->getX(); point[1] = path_[i]->getY();
  traj.push_back(point);
  for (i--; i > 0; --i)
  {
    point[0] = path_[i]->getX(); point[1] = path_[i]->getY();
    traj.push_back(point);
    if (isCusp(i)) {  // if the i th point is a direction changing point
      trajPointSet.push_back(traj);
      trajDirctionSet.push_back((path_[i]->getPrim() < 3 ? true : false));
      traj.clear();
      traj.push_back(point);
    }
  }
  point[0] = path_[i]->getX(); point[1] = path_[i]->getY();
  traj.push_back(point);
  trajPointSet.push_back(traj); // push back the last piece of trajectory
  trajDirctionSet.push_back(path_[i]->getPrim() < 3 ? true : false);
}

//  this function is for agv, not for inner planning
void BSpline::addPointBetweenLine() {
  // add one point between two point lines
  for(int i = 0; i < outputTrajPointSet.size(); ++i) {
    if (outputTrajPointSet[i].size() == 2) {  // if there are 2 points, interpolate to 5
      auto p0 = outputTrajPointSet[i].cbegin();
      auto p2 = p0 + 1;
      auto p1 = (*p0)/2 + (*p2)/2;
      std::vector<Vector2D> temp{p1};
      outputTrajPointSet[i].insert(p2, temp.begin(), temp.end());
    }
  }

  // insert arc point in every piece of lines
  if (outputTrajPointSet.size() > 1) {
    for (int i = 1; i < outputTrajPointSet.size(); ++i) {
      auto v1 = outputTrajPointSet[i][0];
      auto vm1 = *(outputTrajPointSet[i-1].end()-2);  // v minus 1
      auto vp1 = outputTrajPointSet[i][1];  // v plus 1
      double dp = v1.distanseFrom(vp1); // distance plus
      // 0.2 is the step length of reeds shepp curve
      if (dp < 0.25) continue;
      double dm = v1.distanseFrom(vm1); // distance minus
      auto vp = vp1 - v1; // v plus
      auto vm = vm1 - v1; // v minus
      double rp = vp.radian();  // radian plus
      double rm = vm.radian();  // radian minus
      double theta = (std::fmod(rp-rm+M_PI, 2*M_PI) - M_PI)/2 + rm;
      Vector2D bisectrix(std::cos(theta), std::sin(theta));
      auto bisectrixMinus = bisectrix*dm/2 + v1;
      auto bisectrixPlus = bisectrix*dp/2 + v1;

      outputTrajPointSet[i-1].insert(outputTrajPointSet[i-1].cend() - 1, bisectrixMinus);
      outputTrajPointSet[i].insert(outputTrajPointSet[i].cbegin() + 1, bisectrixPlus);
    }
  }
  
}

// complete function
void BSpline::smoothPath(float width, float height) {
  setWidthHeight(width,height);
  unsigned int iter = 0;
  unsigned int maxIter = 100;
  // for every piece of trajectories
  size_t i = path_.size()-1;
  for(auto traj = trajPointSet.begin(); traj != trajPointSet.end(); traj++) {
    if (traj->size() >= 5){
      while (iter < maxIter) {
        // optimize every proper points in every trajectories
        for (auto p = traj->begin()+2; p != traj->end()-2; p++)
        {
          Vector2D xim2((p-2)->getX(), (p-2)->getY());  // xi-2
          Vector2D xim1((p-1)->getX(), (p-1)->getY());  // xi-1 minus
          Vector2D xi(p->getX(), p->getY());
          Vector2D xip1((p+1)->getX(), (p+1)->getY());  // xi+1 plus
          Vector2D xip2((p+2)->getX(), (p+2)->getY());  // xi+2
          Vector2D correction;

          correction = correction - wSmoothness*smoothnessTerm(xim2, xim1, xi, xip1, xip2);
          if (!isOnGrid(xi + correction)) {continue;}

          correction = correction - wCurvature*curvatureTerm(xim1, xi, xip1);
          if (!isOnGrid(xi + correction)) {continue;};

          float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;

          xi = xi + alpha * correction/totalWeight; // gradient descent
          p->setX(xi.getX());
          p->setY(xi.getY());
        }
        iter++;
      }
    }
    for (auto p = traj->begin(); p != traj->end()-1; ++p,--i) {
      path_[i]->setX(p->getX());
      path_[i]->setY(p->getY());
    }
  }

  // modify trajectory outlet to agv
  outputTrajPointSet = trajPointSet;
  addPointBetweenLine();
}

// automatically set the base spline order to 3 or 2
// for quasi-uniform b spline
void BSpline::initializeSplineOrders(bool autogen) {
  auto size = trajPointSet.size();
  for (size_t i = 0; i < size; i++) {
    auto num = trajPointSet[i].size();
    assert(num>=2);
    if (num == 2) // this piece of trajectory will not generate spline
      splineOrder.push_back(0);
    else if(num == 3)  // trajectory with 3 points will generate a 2 order spline
      splineOrder.push_back(2);
    else                // trajectory with more than 3 points will generate a 3 order spline
      splineOrder.push_back(3);   }
}

// set control points for quasi-uniform b spline
void BSpline::setCtrlPoints(bool autogen) {
  auto size = trajPointSet.size();
  for (size_t i = 0; i < size; ++i)
  {
    auto n = trajPointSet[i].size() - 1;
    auto &k = splineOrder[i];

    auto ctrlpSize = n - k + 2; // size for quasi-uniform b spline

    // set control points for b spline generation
    // from one piece of trajectory
    ctrlPoint temp;
    std::vector<ctrlPoint> vtemp;
    if (k != 0) {
      for (size_t j = 0; j < ctrlpSize; ++j)
      {
        if(j == 0 || j == ctrlpSize-1) {  // if at the first point or last point
          temp.multiplicity = k+1;
        }
        else {  // if at the medium points
          temp.value = j;
          temp.multiplicity = 1;
        }
        temp.value = j;
        vtemp.push_back(temp);
      }
    }
    ctrlPointSet.push_back(std::move(vtemp));
  }
  
}

void BSpline::clearPath() {
  Smoother::clearPath();
  splineOrder.clear();
  ctrlPointSet.clear();
  trajPointSet.clear();
  trajDirctionSet.clear();
  bpath_.clear();
}

void BSpline::interpolate() {
  // initialization
  initializeSplineOrders();
  setCtrlPoints();

  auto trajpieces = trajPointSet.size();
  for (size_t i = 0; i < trajpieces; i++) {
    auto splineorder = splineOrder[i];
    if (splineorder != 0){  // if the spline order > 0
      bsplinebasis c(trajPointSet[i],splineorder);
      
      double t = 0;
      double steplen = 0.1;
      double len = ctrlPointSet[i].size()-1;
      while (t < len) {
        auto v = c.compute(t);
        bpath_.push_back(std::make_shared<Common::SE2State>(v.getX(),v.getY(),0));
        t += steplen;
      }
    } else {  // else
      // bpath_.push_back(std::make_shared<Common::SE2State>(
      //                  trajPointSet[i][0].getX(),trajPointSet[i][0].getY(),0));
      ;
    }
  }
}
