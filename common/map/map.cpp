#include "map.h"


namespace HybridAStar {
namespace Common {
  template <class T>
  Map<T>::Map(const nav_msgs::OccupancyGrid::Ptr map) {
    setMap(map);
  }

  template <class T>
  void Map<T>::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    info_.width = map->info.width;
    info_.height = map->info.height;
    info_.resolution = map->info.resolution;
    info_.data = map->data;

    setSS();
  }

  template <class T>
  void Map<T>::setSS() {
    T temp;
    int pwidth = info_.width*info_.resolution/info_.planResolution;
    int pheight = info_.height*info_.resolution/info_.planResolution;
    // TODO: needs to be fixed
    if(temp.getDimensions() == 2)
      statespace = std::vector<T>(pwidth*pheight, T(info_.planResolution,0));
    else
      statespace = std::vector<T>(pwidth*pheight*72, T(info_.planResolution,0.087266));
    hasStateSpace_ = true;
  }

  template <class T>
  void Map<T>::resetSS() {
    assert(statespace.size());
    for (auto i = statespace.begin(); i < statespace.end(); ++i)
      i->clear();
    hasStateSpace_ = true;
  }

  template class Map<GridState>;
  template class Map<SE2State>;
} // namespace Common
} // namespace HybridAStar