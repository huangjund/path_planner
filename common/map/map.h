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
  template <class T>
  class Map {
  private:
    bool hasStateSpace_ = false;
  public:
    /**
     * @brief map read from map server
     * 
     */
    struct RawMap {
      int width;  // [unit: cell]
      int height; // [unit: cell]
      float resolution;   // [unit: meters/cell]
      float planResolution; // [unit: meters/cell]

      nav_msgs::OccupancyGrid::_data_type data;
      RawMap(){width = 0; height = 0; resolution = 0; planResolution = 0.5;
       //data.reserve(10000000);
       }
      // RawMap& operator=(const RawMap&);
    } info_;
    
    // non-copyable
    Map(const Map &) = delete;
    Map &operator=(const Map &) = delete;
    // constructor
    Map() = default;
    explicit Map(const nav_msgs::OccupancyGrid::Ptr map);
    virtual ~Map() = default;

    std::vector<T> statespace;  ///< state for every block
    void setMap(const nav_msgs::OccupancyGrid::Ptr map);

    /**
     * @brief set state space
     * 
     */
    void setSS();
    void resetSS();
    // const std::vector<T>& getStateSpace() const;
    const bool hasStateSpace() { return hasStateSpace_;}
  };

} // namespace Common
} // namespace HybridAStar
