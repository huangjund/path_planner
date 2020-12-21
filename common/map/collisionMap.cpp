#include "collisionMap.h"

namespace HybridAStar {
namespace Common {
  CollisionMap::CollisionMap(const nav_msgs::OccupancyGrid::Ptr& map): 
    width_(map->info.width), height_(map->info.height), 
    colResolution_(map->info.resolution), data_(map->data) {}

  void CollisionMap::setMap(const nav_msgs::OccupancyGrid::Ptr& map) {
    width_ = map->info.width;
    height_ = map->info.height;
    colResolution_ = map->info.resolution;
    data_ = map->data;
  }

} // namespace Common
} // namespace HybridAStar