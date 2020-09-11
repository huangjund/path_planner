#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

//#include "../geometry/dynamicvoronoi.h"
#include "../common/statespace/SE2State.h"
#include "../utils/Helper.h"
#include "../math/vector2d.h"
#include "../multibody/SingleForkLiftPlant.h"

namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
class Smoother {
 public:
  Smoother();

  /*!
     \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.

     During the different interations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
  */
  void smoothPath(float width, float height);

  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
  */
  void tracePath(const std::shared_ptr<Common::SE2State> node);

  /// returns the path of the smoother object
  std::vector<std::shared_ptr<Common::SE2State>> getPath() {return path_;}

  /// obstacleCost - pushes the path away from obstacles
  Vector2D obstacleTerm(Vector2D xi);

  /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
  Vector2D curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);

  /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
  Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

  /// voronoiCost - trade off between path length and closeness to obstaclesg
  //   Vector2D voronoiTerm(Vector2D xi);

  /// a boolean test, whether vector is on the grid or not
  bool isOnGrid(Vector2D vec) {
    if (vec.getX() >= 0 && vec.getX() < width &&
        vec.getY() >= 0 && vec.getY() < height) {
      return true;
    }
    return false;
  }

  void clearPath();

 private:
  std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;
  /// maximum possible curvature of the non-holonomic vehicle
  float kappaMax = 1.f / carPlant_->rad_;
  /// maximum distance to obstacles that is penalized
  //   TODO: this should be in the shield?
  // [m] --- The minimum width of a safe road for the vehicle at hand
  float obsDMax = 1;
  // TODO: below
  /// maximum distance for obstacles to influence the voronoi field
  // [m] --- The minimum width of a safe road for the vehicle at hand
  float vorObsDMax = 1;
  /// falloff rate for the voronoi field
  float alpha = 0.1;
  /// weight for the obstacle term
  float wObstacle = 0;
  /// weight for the voronoi term
  float wVoronoi = 0;
  /// weight for the curvature term
  float wCurvature = 0;
  /// weight for the smoothness term
  float wSmoothness = 0.2;
  /// width of the map
  // collision map width
  int width; 
  /// height of the map
  // collision map height
  int height;
  /// path to be smoothed
  std::vector<std::shared_ptr<Common::SE2State>> path_;
};
}
#endif // SMOOTHER_H
