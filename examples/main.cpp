#include <memory>

#include <gflags/gflags.h>

#include "main.h"

namespace HybridAStar{

  Interface::Interface():automata_(initial),
    planningMap(std::make_shared<Map<SE2State>>()),
    configSpace(std::make_shared<CollisionDetection>()),
    planner_(std::make_unique<HAstar>())
  {
    isStartvalid_ = false;
    isGoalvalid_ = false;
    hasMap_ = false;
    subGoal_ = n_.subscribe("/move_base_simple/goal", 1, &Interface::makeGoal, this);
    subStart_ = n_.subscribe("/initialpose", 1, &Interface::makeStart, this);
    subMap_ = n_.subscribe("/map", 1, &Interface::setMap, this);
  }

  void Interface::makeStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
    // TODO: lack of a solution to no map loaded
    float x = start->pose.pose.position.x / grid_->info.resolution;   //[unit: collision cells]
    float y = start->pose.pose.position.y / grid_->info.resolution;
    float t = tf::getYaw(start->pose.pose.orientation);

    if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0) {
      isStartvalid_ = true;
      Action action = startSetting;
      start_ = *start;
      start_.pose.pose.position.x = 43.302;
      start_.pose.pose.position.y = 10.267;
      start_.pose.pose.position.z = 0;
      start_.pose.pose.orientation.w = -0.853;
      start_.pose.pose.orientation.x = 0;
      start_.pose.pose.orientation.y = 0;
      start_.pose.pose.orientation.z = 1;
      
      // automata
      outputAutomata(action);
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << t << std::endl;
    }

    
  }

  void Interface::makeGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    // retrieving goal position
    float x = goal->pose.position.x / grid_->info.resolution; // [unit:collision cells]
    float y = goal->pose.position.y / grid_->info.resolution;
    float t = tf::getYaw(goal->pose.orientation);

    if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0) {
      isGoalvalid_ = true;
      Action action = goalSetting;
      goal_ = *goal;
      goal_.pose.position.x = 45.139;
      goal_.pose.position.y = 5.067;
      goal_.pose.position.z = 0;
      goal_.pose.orientation.w = 0.542;
      goal_.pose.orientation.x = 0;
      goal_.pose.orientation.y = 0;
      goal_.pose.orientation.z = -0.84;

      // automata
      outputAutomata(action);
    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << t << std::endl;
    }
  }
  
  void Interface::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    grid_ = map;
    hasMap_ = true;
    Action action = Action::mapSetting;
    clearStartandGoal();
    
    // automata
    outputAutomata(action);
  }

  bool Interface::outputAutomata(Action action) {
    switch (automata_)
    {
    case initial:
      if (hasMap_ && isStartvalid_ && isGoalvalid_){
        automata_ = plan;
        setAllOutput();
        simulate(planningMap);
        automata_ = planned;
      }
      break;
    case plan:
      /* no action executed */
      /* this action should no really exist in this automata
         as it is a temporary value */
      break;
    case planned:
      if (action == Action::mapSetting || action == goalSetting) {
        automata_ = initial;
        outputAutomata(action);
      } else if(action == startSetting) {
        automata_ = replan;
        outputAutomata(action);
      }
      break;
    case replan:
      setStartOutput();
      simulate(planningMap);
      automata_ = planned;
      break;
    default:
      std::cerr << "AUTOMATA ERROR" << std::endl;
      break;
    }
  }

  bool** Interface::makeBinMap() {
    bool **p = new bool*[grid_->info.width];
    for(size_t i = 0; i<grid_->info.width; ++i) {
      p[i] = new bool[grid_->info.height];
    }

    for (size_t i = 0; i<grid_->info.width; ++i) {
      for (size_t j = 0; j<grid_->info.height; ++j) {
        p[i][j] = grid_->data[j*grid_->info.width+i] ? true : false;
      }
    }

    return p;
  }

  bool Interface::setAllOutput() {
    // output to map object
    planningMap->setMap(grid_);
    configSpace->setGrid(grid_); // set the grid for configuration space
    configSpace->makeClsLookup();  // make up look up table in configuration space


    // output to planner
    // initialize using planning map resolution
    // TODO: change this cell size and angle size to a class
    auto start = std::make_shared<SE2State>(0.5,0.08726646);
    SE2State goal(carPlant_->planResolution,carPlant_->planAngleResolution);
    start->setX(start_.pose.pose.position.x);
    start->setY(start_.pose.pose.position.y);
    goal.setX(goal_.pose.position.x);
    goal.setY(goal_.pose.position.y);
    start->setT(Utils::normalizeHeadingRad(tf::getYaw(start_.pose.pose.orientation)));
    goal.setT(Utils::normalizeHeadingRad(tf::getYaw(goal_.pose.orientation)));
    // TODO: extract this planner into a specified class
    planner_.reset(std::make_unique<HAstar>(start,goal,planningMap,configSpace).release());

    return true;
  }

  void Interface::simulate(std::shared_ptr<Map<SE2State>> pmap) {
    // retrieve start point and goal point
    auto nStart = planner_->getStart();
    auto nGoal = planner_->getGoal();
    
    visualizer_.clear();
    smoother_.clearPath();
    path_.clear();
    smoothedPath_.clear();

    auto t0 = ros::Time::now();
    auto nSolution = planner_->solve(); auto t1 = ros::Time::now();
    smoother_.tracePath(nSolution);
    path_.updatePath(smoother_.getPath());
    smoother_.smoothPath(pmap->info_.width, pmap->info_.height); 
    auto t4 = ros::Time::now();
    smoothedPath_.updatePath(smoother_.getPath());
    ros::Duration d1(t1 - t0);
    ros::Duration d2(t4 - t1);

    std::cout << "TIME in ms:" << d1*1000 << '\t' << d2*1000 << std::endl;

    path_.publishPath();
    path_.publishPathNodes();
    path_.publishPathVehicles();
    smoothedPath_.publishPath();
    smoothedPath_.publishPathNodes();
    smoothedPath_.publishPathVehicles();
    visualizer_.publishNode3DCosts(pmap);

    smoother_.splinePub();
  }

  void Interface::setStartOutput(){
    planningMap->resetSS();
    // TODO: due to undefined reference to carPlant->planResolution
    auto start = std::make_shared<SE2State>(0.5,0.08726646);
    start->setX(start_.pose.pose.position.x);
    start->setY(start_.pose.pose.position.y);
    start->setT(Utils::normalizeHeadingRad(tf::getYaw(start_.pose.pose.orientation)));

    planner_->setStart(start);
  }

  void Interface::clearStartandGoal() {
    isStartvalid_ = false;
    isGoalvalid_ = false;
  }
} // namespace HybridAStar

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "a_star");
  auto interface(std::make_unique<HybridAStar::Interface>());
  ros::spin();
  return 0;
}
