#include "hRRTx.h"

#define TIME 3

namespace HybridAStar  {
namespace Common {
  hRRTx::hRRTx(SE2State &start, SE2State &goal,const int& cwidth,const int &cheight,CollisionDetection& config): 
  start_(start), goal_(goal),
  r2Space_(std::make_shared<ob::RealVectorStateSpace>(2)),
  spaceInfo_(std::make_shared<ob::SpaceInformation>(r2Space_)),
  problemDef_(std::make_shared<ob::ProblemDefinition>(spaceInfo_)),
  rrtxPlanner_(std::make_unique<og::RRTXstatic>(spaceInfo_)) {
    // R2 space map size set
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0,0);
    bounds.setLow(1,0);
    bounds.setHigh(0,cwidth);
    bounds.setHigh(1,cheight);
    r2Space_->setBounds(bounds);

    spaceInfo_->setStateValidityChecker(std::make_shared<rrtValidityChecker>(spaceInfo_,config));

    ob::ScopedState<ob::RealVectorStateSpace> inception(r2Space_);
    ob::ScopedState<ob::RealVectorStateSpace> destination(r2Space_);
    inception[0] = start.getX(); inception[1] = start.getY();
    destination[0] = goal.getX(); destination[1] = goal.getY();

    // planning problem define
    problemDef_->setStartAndGoalStates(destination,inception);
    rrtxPlanner_->setProblemDefinition(problemDef_);

    // planner setup
    rrtxPlanner_->setup();
    rrtxPlanner_->ob::Planner::solve(TIME);
    treeConstructed_ = true;
  }

  bool hRRTx::terminationFn() {
    static int counter = ITER_;
    if(counter--) return false;
    else {
        counter = ITER_;
        return true;
    }
  }

  void hRRTx::setStart(SE2State &start) {
    start_ = start;
  }

  void hRRTx::setGoal(SE2State &goal) {
    goal_ = goal;
  }

  void hRRTx::setStartGoal(SE2State &start, SE2State &goal) {
    start_ = start;
    goal_ = goal;
  }

  bool hRRTx::rrtValidityChecker::checker_(const ob::State* state) const {
    // extract position from state
    auto r2state = state->as<ob::RealVectorStateSpace::StateType>()->values;
    
    // initialize a 3d node
    auto x = static_cast<float>(r2state[0]);
    auto y = static_cast<float>(r2state[1]);
    auto node = std::make_shared<SE2State>(x, y, 0, 0, 0, nullptr, 0, 0);
    // check for collision
    config_.isTraversable<SE2State>(node.get(), true);
    return true;
  }

  double hRRTx::getDistance() {
    problemDef_->clearSolutionPaths();
    ob::ScopedState<ob::RealVectorStateSpace> goal(r2Space_);
    ob::ScopedState<ob::RealVectorStateSpace> start(r2Space_);
    if(isTreeconstructed()) {
      start[0] = start_.getX();
      start[1] = start_.getY();
      problemDef_->setGoalState(start);
    } else {
      start[0] = start_.getX(); start[1] = start_.getY();
      goal[0] = goal_.getX(); goal[1] = goal_.getY();
      problemDef_->setStartAndGoalStates(goal,start);
    }
    rrtxPlanner_->solve(ob::PlannerTerminationCondition(terminationFn));
    return problemDef_->getSolutionDifference();
  }
} // namespace Common
} // namespace HybridAStar