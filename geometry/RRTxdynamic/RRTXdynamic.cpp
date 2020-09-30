#include "RRTXdynamic.h"

namespace HybridAStar
{
namespace Geometry
{
  RRTXdynamic::RRTXdynamic(const point_t& goal, const point_t& start):
                          v_goal_(goal),v_start_(start),v_bot_(start),
                          vertexSet_(pointVec{v_goal_}),
                          visableTree_(kdtree(vertexSet_)),
                          u(std::uniform_real_distribution<double>(0,1)) {
    gama = 6*30*60; // 2^D(1+1/D)*S(x_free)
  }

  void RRTXdynamic::solve(const Common::PlannerTerminationCondition& ptc) {
    while(1) {
      auto radius = shrinkingBallRadius(vertexSet_.size());
      updateObstacles();
      auto v = genRandom();
      auto v_nearest = nearest(v);

      // if the new point is greater than the largest radius
      // give the new vertex an offset
      if (Distance(v.first, v_nearest.first) > delta) 
        saturate(v, v_nearest);

      
    }
  }

  void RRTXdynamic::solve(double solvetime) {
    if (solvetime < 1.0)
      return solve(Common::timedPlannerTerminationCondition(solvetime));
    return solve(Common::timedPlannerTerminationCondition(solvetime, std::min(solvetime / 100.0, 0.1)));
  }

  double RRTXdynamic::shrinkingBallRadius(const size_t& vsetSize) {
    double temp = sqrt(gama*std::log10(vsetSize)/(vsetSize*M_PI));
    return std::min(temp, delta);
  }

  void RRTXdynamic::updateObstacles() {}

  RRTXdynamic::point_t RRTXdynamic::genRandom() {
    Point<2> temp;
    temp[0] = u(randEngine);
    temp[1] = u(randEngine);
    return point_t(temp,temp);
  }

  RRTXdynamic::point_t RRTXdynamic::nearest(const point_t& v) {}

  void RRTXdynamic::saturate(point_t& _v, const point_t& _v_nearest) {}
} // namespace Geometry
} // namespace HybridAStar
