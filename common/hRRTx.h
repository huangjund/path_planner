#ifndef _HEURISTIC_RRTX_H
#define _HEURISTIC_RRTX_H

#include "Heuristic.h"
#include "common/statespace/SE2State.h"
#include "geometry/RRTxstatic/RRTXstatic.h"
#include "geometry/RRTxdynamic/RRTXdynamic.h"
#include "common/collision/clsDetection.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/MotionValidator.h>

#include <memory>
#include <iostream>
#include <vector>

namespace HybridAStar {
namespace Common {
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
  class hRRTx : public Heuristic{
  private:
    static constexpr int ITER_ = 1;  // iterations ran for RRTx replanning
    bool treeConstructed_ = false;
    static bool terminationFn();
    // TODO: change this two state into Heuristic class (high priority)
    SE2State start_;
    SE2State goal_;
    // TODO: change this class into a functional constructor
    class rrtValidityChecker : public ob::StateValidityChecker {
      public:
        rrtValidityChecker(const ob::SpaceInformationPtr &si, std::shared_ptr<CollisionDetection>& config) :
          ob::StateValidityChecker(si), config_(config) {}
        virtual bool isValid(const ob::State *state) const {
          return checker_(state);
        }
      private:
        std::shared_ptr<CollisionDetection> config_;
        bool checker_(const ob::State*) const;
    };

    class rrtMotionChecker : public ob::MotionValidator {
      private:
        std::shared_ptr<CollisionDetection> config_;
      public:
        rrtMotionChecker(const ob::SpaceInformationPtr& si, std::shared_ptr<CollisionDetection>& config) :
          ob::MotionValidator(si), config_(config) {}
        virtual bool checkMotion(const ob::State*, const ob::State*) const;
        virtual bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const;
    };
    
    std::shared_ptr<ob::RealVectorStateSpace> r2Space_;
    std::shared_ptr<ob::SpaceInformation> spaceInfo_;
    std::shared_ptr<ob::ProblemDefinition> problemDef_;
    std::unique_ptr<og::RRTXstatic> rrtxPlanner_;
  public:
    hRRTx() = default;
    hRRTx(SE2State &, SE2State &,const double &,const double &,std::shared_ptr<CollisionDetection>&);
    ~hRRTx() = default;

    hRRTx(const hRRTx &) = delete;
    hRRTx &operator=(const hRRTx &) = delete;

    double getDistance();
    bool isTreeconstructed() {return treeConstructed_;};

    void setStart(SE2State &start);
    void setGoal(SE2State &goal);
    void setStartGoal(SE2State &start, SE2State &goal);
  };
} // namespace Common
} // namespace Hybri

#endif