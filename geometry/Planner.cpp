#include "Planner.h"

namespace HybridAStar{
namespace Geometry{
  Planner::Planner(){
    subGoal_ = n_.subscribe("/move_base_simple/goal", 1, &Planner::makeGoal, this);
    subStart_ = n_.subscribe("/initialpose", 1, &Planner::makeStart, this);
  }

  void Planner::makeStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
    float x = start->pose.pose.position.x / collisionMapCellSize;
    float y = start->pose.pose.position.y / Node3D::collisionMapCellSize;
    float t = tf::getYaw(start->pose.pose.orientation);

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      validStart = true;
      start = *start;

      if (Constants::manual) { plan();}
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << t << std::endl;
    }
  }

  void Planner::makeGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    // retrieving goal position
    float x = end->pose.position.x / Node3D::collisionMapCellSize;
    float y = end->pose.position.y / Node3D::collisionMapCellSize;
    float t = tf::getYaw(end->pose.orientation);

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      validGoal = true;
      goal = *end;

      if (Constants::manual) { plan();}

    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << t << std::endl;
    }
  }
} // namespace Geometry
} // namespace HybridAStar
