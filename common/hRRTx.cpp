#include "hRRTx.h"

#define TIME 10

namespace HybridAStar  {
namespace Common {
  hRRTx::hRRTx(SE2State &start, SE2State &goal,const double &mapwidth,
              const double &mapheight,std::shared_ptr<CollisionDetection>& config): 
  start_(start), goal_(goal),
  r2Space_(std::make_shared<ob::RealVectorStateSpace>(2)),
  spaceInfo_(std::make_shared<ob::SpaceInformation>(r2Space_)),
  problemDef_(std::make_shared<ob::ProblemDefinition>(spaceInfo_)),
  rrtxPlanner_(std::make_unique<og::RRTXstatic>(spaceInfo_)) {
    // R2 space map size set
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0,0);
    bounds.setLow(1,0);
    bounds.setHigh(0,mapwidth);
    bounds.setHigh(1,mapheight);
    r2Space_->setBounds(bounds);

    spaceInfo_->setMotionValidator(std::make_shared<rrtMotionChecker>(spaceInfo_, config));
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

    //====================================================================================
    // Geometry::RRTXdynamic::point_t s,e;
    // s[0] = start.getX(); s[1] = start.getY();
    // e[0] = goal.getX(); e[1] = goal.getY();
    // Geometry::RRTXdynamic testPlanner(s,e,mapwidth,mapheight,config);
    // testPlanner.solve(10);
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
    auto node = std::make_shared<SE2State>(x, y, 0, 0, 0, nullptr, 0, 0, 0);
    
    // check for collision
    return config_->isTraversable<SE2State>(node.get(), true);;
  }

  bool hRRTx::rrtMotionChecker::checkMotion(const ob::State* s1, const ob::State* s2) const {
    auto r2state1 = s1->as<ob::RealVectorStateSpace::StateType>()->values;
    auto r2state2 = s2->as<ob::RealVectorStateSpace::StateType>()->values;

    return config_->fastSearch(r2state1,r2state2);
  }

  bool hRRTx::rrtMotionChecker::checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double>&) const {
    return this->checkMotion(s1, s2);
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
    if(problemDef_->hasSolution())
      return problemDef_->getSolutionPath()->length();
    else
      std::cerr << "there is no solution path: "<< isTreeconstructed() << std::endl;
  }
} // namespace Common
} // namespace HybridAStar