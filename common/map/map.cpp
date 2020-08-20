#include "map.h"


namespace HybridAStar {
namespace Common {
  template <class T>
  Map<T>::Map(const nav_msgs::OccupancyGrid::Ptr map) {
    setMap(map);
  }
  
  template <class T>
  void Map<T>::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    info_.data = map->data;
    info_.width = map->info.width;
    info_.height = map->info.height;
    info_.resolution = map->info.resolution;

    setSS();
  }

  template <class T>
  void Map<T>::setSS() {
    statespace = std::vector<T>(info_.width*info_.height, T(info_.resolution,0.087266));
    hasStateSpace_ = true;
  }

  // TODO: enable this member function
  // template <class T>
  // const std::vector<T>& Map<T>::getStateSpace() const {
  //   if (!hasStateSpace()){
  //     std::cerr << "ERROR: THIS MAP HAS NO STATE SPACE" << std::endl;
  //     return std::vector<T>(T());
  //   }
  //   return statespace;
  // }

  template class Map<GridState>;
  template class Map<SE2State>;
} // namespace Common
} // namespace HybridAStar