#ifndef _HYBRID_A_STAR_RRTX_DYNAMIC_H
#define _HYBRID_A_STAR_RRTX_DYNAMIC_H

#include <vector>
#include <functional>
#include <utility>

#include "common/PlannerTerminationCondition.h"
#include "math/kdtree.h"

namespace HybridAStar
{
namespace Geometry
{
  struct Vertex {
    double x;
    double y;
    Vertex() : Vertex(0,0) {}
    Vertex(const double& xcor, const double& ycor) : x(xcor), y(ycor) {}
    Vertex(const Vertex& v) : x(v.x), y(v.y) {}
    Vertex& operator=(const Vertex& v) {x = v.x; y = v.y;}
  };



  // TODO: this class should be inherit from Planner
  class RRTXdynamic
  {
    using point_t = std::pair<Point<2>, unsigned int>;  /// a 2 dimensional point
    using pointVec = std::vector<point_t>;
    using kdtree = KDTree<1, unsigned int>;
  private:
    point_t v_goal_;
    point_t v_start_;
    point_t v_bot_;
    pointVec vertexSet_;  // store all vertices into this structure
    kdtree isibleTree_;


    double r_;
  public:
    RRTXdynamic(const point_t&, const point_t&);
    ~RRTXdynamic();

    void setvGoal(const double& x, const double& y);
    void setvStart(const double& x, const double& y);
    void setvRobot(const double&, const double&);
    const point_t& getvRobot() const {return v_bot_;}
    point_t& getMutableVRob() {return v_bot_;}

    // TODO: just like solve function in RRTXstatic,
    // set validator as a function pointer parameter
    void solve(const Common::PlannerTerminationCondition& ptc);
    void solve(double solvetime);
    double shrinkingBallRadius(const size_t&);
    void updateObstacles();
    point_t genRandom();
    point_t nearest(const point_t&);
  };

} // namespace Geometry
} // namespace HybridAStar


#endif