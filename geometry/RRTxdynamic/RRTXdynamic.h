#ifndef _HYBRID_A_STAR_RRTX_DYNAMIC_H
#define _HYBRID_A_STAR_RRTX_DYNAMIC_H

#include <vector>
#include <functional>
#include <utility>
#include <random>
#include <memory>
#include <map>
#include <ros/ros.h>

#include "common/PlannerTerminationCondition.h"
#include "common/collision/clsDetection.h"
#include "common/ValidityChecker.h"
#include "common/statespace/SE2State.h"
#include "math/kdtree.h"

#include <visualization_msgs/MarkerArray.h>

namespace HybridAStar
{
namespace Geometry
{
  /**
   * @brief reference to [RRTx: Asymptotically Optimal Single-Query Sampling-Based Motion]
   * Planning with Quick Replanning
   * not in use now
   */
  class RRTXdynamic
  {
   public:
    using point_t = Point<2>;  /// first: geometry position and value carried by the position
    using pointVec = std::vector<std::shared_ptr<point_t>>;
    using kdtree = KDTree<2>;
   private:
    struct less
    {
      bool operator()(const std::shared_ptr<point_t>& _x,
                      const std::shared_ptr<point_t>& _y) const {
        std::pair<double,double> x(std::min(_x->getG(),_x->getLMC()),_x->getG());
        std::pair<double,double> y(std::min(_y->getG(),_y->getLMC()),_y->getG());
        return (x.first < y.first || (x.first == y.first && x.second < y.second));
      }
    };
    

    point_t v_goal_;
    point_t v_start_;
    point_t v_bot_;
    pointVec vertexSet_;  // store all vertices into this structure
    std::map<std::shared_ptr<point_t>, double, less> orphanSet_;  // store all orphan vertices  vertexSet_/visableTree_ = orphanSet_
    kdtree visableTree_;
    std::map<std::shared_ptr<point_t>, double, less> Q_; 

    const double delta = 1.5;  // the fixed maximum radius
    double gama;
    const double epsilon = 0; // epsilon-consistent parameter

    std::default_random_engine randEngine;
    std::uniform_real_distribution<double> u;
    double map_width_, map_height_;

    std::unique_ptr<ValidityChecker> stateChecker;
    std::unique_ptr<MotionChecker> motionChecker;

    // uniform sampler
    double shrinkingBallRadius(const size_t&);

    point_t genRandom();
    
    std::shared_ptr<point_t> nearest(const point_t&);
    
    void saturate(point_t& _v, const point_t& _v_nearest);
    
    void extend(point_t& _v, const double r, std::shared_ptr<point_t>& v);
    
    void rewireNeighbors(std::shared_ptr<point_t>& _v, const double& r);
    
    void cullNeighbors(std::shared_ptr<point_t>& _v, const double& r);
    
    void makeParentOf(std::shared_ptr<point_t>& _v, std::shared_ptr<point_t>& _u);
    
    void verrifyQueue(std::shared_ptr<point_t>& _u);
    
    void findParent(point_t&,const BoundedPQueue<std::shared_ptr<Point<2>>>&, const double& r);
    
    void reduceInconsistency(const double& radius);

    void updateLMC(std::shared_ptr<point_t>& v, const double& r);

    void connectNearestFeasible(std::shared_ptr<point_t>& v, const double r);
   public:
    RRTXdynamic(const point_t&, const point_t&,
                double, double,
                std::shared_ptr<Common::CollisionDetection>&);  // [unit: meters]
    ~RRTXdynamic() = default;

    void setvGoal(const double& x, const double& y);
    void setvStart(const double& x, const double& y);
    void setvRobot(const double&, const double&);
    const point_t& getvRobot() const {return v_bot_;}
    point_t& getMutableVRob() {return v_bot_;}

    // TODO: just like solve function in RRTXstatic,
    // set validator as a function pointer parameter
    void solve(const Common::PlannerTerminationCondition& ptc);
    void solve(double solvetime);
    void updateObstacles();
    void setStateValidityChecker(const std::shared_ptr<Common::CollisionDetection>&);
    void setMotionValidityChecker(const std::shared_ptr<Common::CollisionDetection>&);
   private: // for visualization
    struct Visualizer {
      ros::NodeHandle n;
      ros::Publisher pub;
      Visualizer();
    } visualizer;

    void visual(std::shared_ptr<point_t>, visualization_msgs::Marker&);
  };

} // namespace Geometry
} // namespace HybridAStar


#endif