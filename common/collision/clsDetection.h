#ifndef _HYBRID_A_STAR_COLLISION_DETECTION_H
#define _HYBRID_A_STAR_COLLISION_DETECTION_H

#include "common/parameters/parameters.h"
#include "common/statespace/GridState.h"
#include "common/statespace/SE2HAStarState.h"
#include "common/map/planningMap.h"
#include "common/map/collisionMap.h"
#include "utils/PtrWrapper.h"

#include <nav_msgs/OccupancyGrid.h>

#include <memory>
#include <cmath>
#include <limits>

namespace HybridAStar {
namespace Common {
  CLASS_SHARED(CollisionDetection);
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
      int x;
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
      relPos pos[2000];
    };
    
    CollisionMapUnique clsMap_;

    int sign(double);
    /// The collision lookup table
    std::unique_ptr<configuration> collisionLookup_;

   public:
    CollisionDetection(const CollisionDetection&) = delete;
    CollisionDetection &operator=(const CollisionDetection &) = delete;
    // constructor
    CollisionDetection();
    CollisionDetection(const nav_msgs::OccupancyGrid::Ptr&);
    ~CollisionDetection() = default;

    void setGrid(const nav_msgs::OccupancyGrid::Ptr&);
    void setPGrid();
    void getConfiguration(const SE2State*, float &,float &, float &) ;
    void getConfiguration(const GridState*, float &, float &, float &) ;

    template <class T>
    bool isTraversable(const T*) ;
    template <class T>
    bool isTraversable(const T*, const bool) ;  // when it is rrtx

    bool fastSearch(double*, double*); // search if it is traversable for a line

    bool configinCFree(float x, float y, float t) ;
    
    void makeClsLookup();
  };
} // namespace Common
} // namespace HybridAStar


#endif