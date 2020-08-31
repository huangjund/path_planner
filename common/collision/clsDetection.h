#pragma once

#include "multibody/SingleForkLiftPlant.h"
#include "common/statespace/GridState.h"
#include "common/statespace/SE2State.h"
#include "common/map/map.h"

#include <nav_msgs/OccupancyGrid.h>

#include <memory>
#include <cmath>
#include <limits>

namespace HybridAStar {
namespace Common {
		// TODO: this class needs to be modified
  class CollisionDetection
  {
  private:
    /// Planning Map with occupancy property [planning map]
    struct PlanOccupancyGrid {
      int width; // [unit:cells] planning map
      int height; // [unit:cells] planning map
      std::shared_ptr<float> planGrid; //[unit:percent] counting for occupancy rate of per planning map cells
      float threshold = 0.01;
    }pGrid_;
    struct relPos {
      /// the x position relative to the center
      int x;
      /// the y position relative to the center
      int y;
    };
    /// A structure capturing the lookup for each theta configuration
    struct configuration{
      /// the number of cells occupied by this configuration of the vehicle
      int length;
      /*!
        \var relPos pos[64]
        \brief The maximum number of occupied cells
        \todo needs to be dynamic
      */
     //TODO: this car related term should not be at here
      relPos pos[2000];
    };
    
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;
    // TODO: change this grid to a Map class
    nav_msgs::OccupancyGrid::Ptr grid_;
    
    int sign(double);
    /// The collision lookup table
    configuration *collisionLookup_;

  public:
    CollisionDetection(const CollisionDetection&);
    CollisionDetection &operator=(const CollisionDetection &) = delete;
    // constructor
    CollisionDetection();
    CollisionDetection(nav_msgs::OccupancyGrid::Ptr &);
    ~CollisionDetection();

    void setGrid(nav_msgs::OccupancyGrid::Ptr &);
    void setPGrid();
    void getConfiguration(const SE2State*, float &,float &, float &) ;
    void getConfiguration(const GridState*, float &, float &, float &) ;

    template <class T>
    bool isTraversable(const T*) ;
    template <class T>
    bool isTraversable(const T*, const bool) ;  // when it is rrtx

    bool configinCFree(float x, float y, float t) ;
    
    void makeClsLookup();
  };
} // namespace Common
} // namespace HybridAStar
