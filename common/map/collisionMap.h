#ifndef _HYBRID_A_STAR_COLLISION_MAP_H_
#define _HYBRID_A_STAR_COLLISION_MAP_H_

#include "map.h"
#include <memory>

#include "nav_msgs/OccupancyGrid.h"

namespace HybridAStar {
namespace Common {
using CollisionMapPtr = std::shared_ptr<CollisionMap>;
using CollisionMapUnique = std::unique_ptr<CollisionMap>;
class CollisionMap : public Map<int> {
  public:
    // non-copyable
    CollisionMap(const CollisionMap&) = delete;
    CollisionMap& operator=(const CollisionMap&) = delete;
    // constructor
    CollisionMap() = default;
    CollisionMap(const nav_msgs::OccupancyGrid::Ptr& map);
    virtual ~CollisionMap() override = default;

    double colResolution_{0};   // [unit: meters/cell]

    nav_msgs::OccupancyGrid::_data_type data_;

    void setMap(const nav_msgs::OccupancyGrid::Ptr& map);
};
} // namespace Common
} // namespace HybridAStar

#endif