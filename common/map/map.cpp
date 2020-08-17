#include "map.h"

namespace HybridAStar {
namespace Common {
  template <typename T>
  Map<T>::Map(const nav_msgs::OccupancyGrid::Ptr readinMap) {
    info_.data = readinMap->data;
    info_.width = readinMap->info.width;
    info_.height = readinMap->info.height;
    info_.resolution = readinMap->info.resolution;

    statespace.reserve(200000);
    statespace = std::vector<T>(info_.width*info_.height, T());

    hasStateSpace_ = true;
  }
  template <typename T>
  void Map<T>::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    info_.data = map->data;
    info_.width = map->info.width;
    info_.height = map->info.height;
    info_.resolution = map->info.resolution;

    hasStateSpace_ = false;
  }

  template <typename T>
  const std::vector<T>& Map<T>::getStateSpace() const {
    if (!hasStateSpace()){
      std::cerr << "ERROR: THIS MAP HAS NO STATE SPACE" << std::endl;
      return std::vector<T>(T());
    }
    return statespace;
  }
}
}