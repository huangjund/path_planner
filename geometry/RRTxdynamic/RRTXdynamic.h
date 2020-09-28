#ifndef _HYBRID_A_STAR_RRTX_DYNAMIC_H
#define _HYBRID_A_STAR_RRTX_DYNAMIC_H

#include <vector>
#include <functional>

#include "common/PlannerTerminationCondition.h"

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
  private:
    std::vector<Vertex> vertexSet_;
    Vertex v_goal_;
    Vertex v_start_;
    Vertex v_bot_;

    double r_;
  public:
    RRTXdynamic(const Vertex&, const Vertex&);
    ~RRTXdynamic();

    void setvGoal(const double& x, const double& y);
    void setvStart(const double& x, const double& y);
    void setvRobot(const double&, const double&);
    const Vertex& getvRobot() const {return v_bot_;}
    Vertex& getMutableVRob() {return v_bot_;}

    // TODO: just like solve function in RRTXstatic,
    // set validator as a function pointer parameter
    void solve(const Common::PlannerTerminationCondition& ptc);
    void solve(double solvetime);
    double shrinkingBallRadius(const size_t&);
  };

} // namespace Geometry
} // namespace HybridAStar


#endif