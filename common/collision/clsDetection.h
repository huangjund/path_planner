#ifndef _HYBRID_A_STAR_COLLISION_DETECTION_H
#define _HYBRID_A_STAR_COLLISION_DETECTION_H

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
  /**
   * @brief this class is used for collision detection(configuration space)
   * 
   */
  class CollisionDetection
  {
   private:
    /**
     * @brief planning map with occupancy percentage
     */
    struct PlanOccupancyGrid {
      int width; // [unit:cells] planning map
      int height; // [unit:cells] planning map
      std::shared_ptr<float> planGrid; //[unit:percent] counting for occupancy rate of per planning map cells
      float threshold = 0.01;
    }pGrid_;

    /**
     * @brief 2 dimensional point
     * 
     */
    struct relPos {
      /// the x position relative to the center
      int x;
      /// the y position relative to the center
      int y;
    };

    /// A structure capturing the lookup for each theta configuration
    /**
     * @brief configuration space of the car.
     *      for every theta posture, there is one collision configuration
     */
    struct configuration{
      /// the number of cells occupied by this configuration of the vehicle
      int length;
      /*!
        \var relPos pos[64]
        \brief The maximum number of occupied cells
      */
      relPos pos[2000];
    };
    
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_; ///< car configuration parameters
    nav_msgs::OccupancyGrid::Ptr grid_; ///< collision map 
    
    int sign(double);
    configuration *collisionLookup_;  ///< The configuration space lookup table

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

    /**
     * @brief whether the state is collision free
     * 
     * @tparam T 
     * @return true 
     * @return false 
     */
    template <class T>
    bool isTraversable(const T*) ;

    /**
     * @brief whether the state is collision free. this function is for rrtx planner state
     * 
     * @tparam T 
     * @return true 
     * @return false 
     */
    template <class T>
    bool isTraversable(const T*, const bool) ;  // when it is rrtx

    /**
     * @brief search if the path between two state is traversable
     * 
     * @param state1 
     * @param state2 
     * @return true 
     * @return false 
     */
    bool fastSearch(double* state1, double*state2); // search if it is traversable for a line

    /**
     * @brief configuration in collision free space
     * 
     * @param x 
     * @param y 
     * @param t 
     * @return true 
     * @return false 
     */
    bool configinCFree(float x, float y, float t) ;
    
    /**
     * @brief make up collision lookup table \p collisionLookup_
     * 
     */
    void makeClsLookup();
  };
} // namespace Common
} // namespace HybridAStar


#endif