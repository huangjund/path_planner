#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <vector>

typedef ompl::base::SE2StateSpace::StateType State;

#include "planningmap.h"
#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"

namespace HybridAStar {
class PlanMapNode;
class Node2D;
class Visualize;

/*!
 * \brief A class that encompasses the functions central to the search.
 */
class Algorithm {
 public:
  /// The deault constructor
  Algorithm() {}

  // HYBRID A* ALGORITHM
  /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
     \param width the width of the grid in number of cells
     \param height the height of the grid in number of cells
     \param configurationSpace the lookup of configurations and their spatial occupancy enumeration
     \param dubinsLookup the lookup of analytical solutions (Dubin's paths)
     \param visualization the visualization object publishing the search to RViz
     \return the pointer to the node satisfying the goal condition
  */
  PlanMapNode* hybridAStar(PlanMapNode&, const PlanMapNode&, PlanMapNode*, Node2D* ,
                             const int, const int, CollisionDetection&, float*, Visualize& );

};
}
#endif // ALGORITHM_H
