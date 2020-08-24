#include <memory>

#include <gflags/gflags.h>

#include "main.h"

namespace HybridAStar{

  Interface::Interface(){
    planningMap = std::make_shared<Map<SE2State>>();
    CollisionDetection configSpace();
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
      start_ = *start;
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << t << std::endl;
    }

    if(isabletoPlanning())
      setOutput();
  }

  void Interface::makeGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    // retrieving goal position
    float x = goal->pose.position.x / grid_->info.resolution; // [unit:collision cells]
    float y = goal->pose.position.y / grid_->info.resolution;
    float t = tf::getYaw(goal->pose.orientation);

    if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0) {
      isGoalvalid_ = true;
      goal_ = *goal;
    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << t << std::endl;
    }

    if(isabletoPlanning())
      setOutput();
  }
  
  void Interface::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    grid_ = map;
    hasMap_ = true;
    clearStartandGoal();
    
    if(isabletoPlanning())
      setOutput();
  }

  inline bool Interface::isabletoPlanning() const {
    return hasMap_&&isStartvalid_&&isGoalvalid_;
  }

  bool Interface::setOutput() {
    // output to map object
    //auto collisionMap = std::make_unique<Map<GridState>>(grid_);
    planningMap->setMap(grid_);
    configSpace.setGrid(grid_); // set the grid for configuration space
    configSpace.makeClsLookup();  // make up look up table in configuration space

    // output to planner
    SE2State start(0.5,0.08726646); // initialize using planning map resolution
    SE2State goal(0.5,0.08726646);
    start.setX(start_.pose.pose.position.x);
    start.setY(start_.pose.pose.position.y);
    goal.setX(goal_.pose.position.x);
    goal.setY(goal_.pose.position.y);
    start.setT(Utils::normalizeHeadingRad(tf::getYaw(start_.pose.pose.orientation)));
    goal.setT(Utils::normalizeHeadingRad(tf::getYaw(goal_.pose.orientation)));
    // TODO: extract this planner into a specified class
    auto planner = std::make_shared<HAstar>(start,goal,planningMap,configSpace);

    // run simulation
    simulate(planningMap,planner);
    return true;
  }

  void Interface::clearStartandGoal() {
    isStartvalid_ = false;
    isGoalvalid_ = false;
  }

  void Interface::simulate(std::shared_ptr<Map<SE2State>> pmap,
                          std::shared_ptr<HAstar> planner) {
    // retrieve start point and goal point
    auto nStart = planner->getStart();
    auto nGoal = planner->getGoal();
    
    visualizer_.clear();
    path_.clear();
    smoothedPath_.clear();

    SE2State *nSolution = planner->solve();
    smoother_.tracePath(nSolution);
    path_.updatePath(smoother_.getPath());
    smoother_.smoothPath(voronoiDiagram_);
    smoothedPath_.updatePath(smoother_.getPath());

    path_.publishPath();
    path_.publishPathNodes();
    path_.publishPathVehicles();
    smoothedPath_.publishPath();
    smoothedPath_.publishPathNodes();
    smoothedPath_.publishPathVehicles();
    visualizer_.publishNode3DCosts(pmap);
  }
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "a_star");
  auto interface(std::make_unique<HybridAStar::Interface>());
  ros::spin();
  return 0;
}
