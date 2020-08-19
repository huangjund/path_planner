#pragma once

#include <ros/ros.h>
#include <ompl/util/ClassForward.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../statespace/GridState.h"
#include "../statespace/SE2State.h"

#include <functional>
#include <memory>
#include <vector>
#include <iostream>

namespace HybridAStar {
namespace Common {
  // OMPL_CLASS_FORWARD(Map);
  template <class T>
  class Map {
  private:
    bool hasStateSpace_ = false;
    ros::NodeHandle n;
    ros::Subscriber sub;
  public:
    // non-copyable
    Map(const Map &) = delete;
    Map &operator=(const Map &) = delete;
    // constructor
    explicit Map();
    virtual ~Map() = default;

    struct characteristic {
      int width;
      int height;
      float resolution;

      nav_msgs::OccupancyGrid::_data_type data;
      characteristic(){width = 0; height = 0; resolution = 0; data.reserve(200000);}
    } info_;
    
    std::vector<T> statespace;
    
    void setMap(const nav_msgs::OccupancyGrid::Ptr map);
    void setSS();
    // const std::vector<T>& getStateSpace() const;
    const bool hasStateSpace() { return hasStateSpace_;}
  };

} // namespace Common
} // namespace HybridAStar
