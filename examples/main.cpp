#include <memory>

#include <gflags/gflags.h>

#include "main.h"

namespace HybridAStar{
  using HybridAStar::Common::CollisionDetection;
  using HybridAStar::Common::GridState;
  using HybridAStar::Common::SE2State;
  using HybridAStar::Common::Map;
  using HybridAStar::Geometry::HAstar;
  using HybridAStar::Multibody::SingleForkLiftPlant;

  Interface::Interface(){
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
  }
  
  void Interface::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    grid_ = map;
  }

  int do_main(){
    ros::NodeHandle rosHandler;
    ros::Subscriber subMap;
    auto interface(std::make_shared<Interface>());
    std::unique_ptr<SingleForkLiftPlant> plant = std::make_unique<SingleForkLiftPlant>();
    std::unique_ptr<Map<GridState>> collisionMap = std::make_unique<Map<GridState>>();
    std::unique_ptr<Map<SE2State>> planningMap = std::make_unique<Map<SE2State>>();
    std::unique_ptr<CollisionDetection> configSpace = std::make_unique<CollisionDetection>(); // the grid is a nullptr
    
    std::unique_ptr<HAstar> planner = std::make_unique<HAstar>();
    return 0;
  }
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "a_star");
  return HybridAStar::do_main();
}
