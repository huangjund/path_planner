#include "smoother.h"
using namespace HybridAStar;

Smoother::Smoother(){}

Smoother::~Smoother() {
  carPlant_.reset();
  for (auto i = path_.begin(); i != path_.end(); i++)
  {
    (*i)->~SE2State();
    i->reset();
  }
  
}
//###################################################
//                                     CUSP DETECTION
//###################################################
// TODO: in RS curve piece, this function cannot extrapolate
// the changing of direction
bool Smoother::isCusp(int i) {
  bool revim2 = path_[i - 2]->getPrim() < 3 ? true : false;
  bool revim1 = path_[i - 1]->getPrim() < 3 ? true : false;
  bool revi   = path_[i]->getPrim() < 3 ? true : false;
  bool revip1 = path_[i + 1]->getPrim() < 3 ? true : false;
  bool revip2 = path_[i + 2]->getPrim() < 3 ? true : false;

  if (revim2 != revim1 || revim1 != revi || revi != revip1 || revip1 != revip2) { return true; }

  return false;
}

void Smoother::clearPath() {
  path_.clear();
}

//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::smoothPath(float width, float height) {
  // load the current voronoi diagram into the smoother
  this->width = width; // collision map width
  this->height = height; // collision map height
  // current number of iterations of the gradient descent smoother
  int iterations = 0;
  // the maximum iterations for the GD smoother
  int maxIterations = 50;
  // the length of the path in number of nodes
  int pathLength = 0;

  // path objects with all nodes oldPath the original, newPath the resulting smoothed path
  pathLength = path_.size();

  // descent along the gradient untill the maximum number of iterations has been reached
  // TODO: can be upgrade
  float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;

  while (iterations < maxIterations) {

    // choose the first three nodes of the path
    for (int i = 2; i < pathLength-2; ++i) {

      Vector2D xim2(path_[i - 2]->getX(), path_[i - 2]->getY());
      Vector2D xim1(path_[i - 1]->getX(), path_[i - 1]->getY());
      Vector2D xi(path_[i]->getX(), path_[i]->getY());
      Vector2D xip1(path_[i + 1]->getX(), path_[i + 1]->getY());
      Vector2D xip2(path_[i + 2]->getX(), path_[i + 2]->getY());
      Vector2D correction;

      // the following points shall not be smoothed
      // keep these points fixed if they are a cusp point or adjacent to one
      if (isCusp(i)) { continue; }

      // TODO: Voronoi Diagram not implemented yet
      // voronoiTerm(); 

      // ensure that it is on the grid
      correction = correction - wSmoothness*smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid
      correction = correction - wCurvature*curvatureTerm(xim1, xi, xip1);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid

      xi = xi + alpha * correction/totalWeight;
      path_[i]->setX(xi.getX());
      path_[i]->setY(xi.getY());
      Vector2D Dxi = xi - xim1;
      path_[i - 1]->setT(std::atan2(Dxi.getY(), Dxi.getX()));
    }

    iterations++;
  }
}

void Smoother::tracePath(const std::shared_ptr<Common::SE2State> node) {
  auto waypoint = node;
  while (waypoint!=nullptr){
    path_.push_back(waypoint);
    waypoint = waypoint->getPred();
  }
}

// //###################################################
// //                                      OBSTACLE TERM
// //###################################################
// Vector2D Smoother::obstacleTerm(Vector2D xi) {
//   Vector2D gradient;
//   // the distance to the closest obstacle from the current node
//   float tempColCellx = xi.getX()/Common::SE2State::collisionMapCellSize;
//   float tempColCelly = xi.getY()/Common::SE2State::collisionMapCellSize;
//   float obsDst = voronoi.getDistance(tempColCellx, tempColCelly);
//   // the vector determining where the obstacle is
//   int x = (int)tempColCellx;
//   int y = (int)tempColCelly;
//   // if the node is within the map
//   if (x < width && x >= 0 && y < height && y >= 0) {
//     Vector2D obsVct(tempColCellx - voronoi.data[(int)tempColCellx][(int)tempColCelly].obstX,
//                     tempColCelly - voronoi.data[(int)tempColCellx][(int)tempColCelly].obstY);

//     // the closest obstacle is closer than desired correct the path for that
//     if (obsDst < obsDMax) {
//       return gradient = wObstacle * 2 * (obsDst - obsDMax) * obsVct / obsDst;
//     }
//   }
//   return gradient;
// }

//###################################################
//                                       VORONOI TERM
//###################################################
// Vector2D Smoother::voronoiTerm(Vector2D xi) {
//   Vector2D gradient;
//   //    alpha > 0 = falloff rate
//   //    dObs(x,y) = distance to nearest obstacle
//   //    dEge(x,y) = distance to nearest edge of the GVD
//   //    dObsMax   = maximum distance for the cost to be applicable
//   // distance to the closest obstacle
//   float obsDst = voronoi.getDistance(xi.getX(), xi.getY());
//   // distance to the closest voronoi edge
//   float edgDst; //todo
//   // the vector determining where the obstacle is
//   Vector2D obsVct(xi.getX() - voronoi.data[(int)xi.getX()][(int)xi.getY()].obstX,
//                     xi.getY() - voronoi.data[(int)xi.getX()][(int)xi.getY()].obstY);
//   // the vector determining where the voronoi edge is
//   Vector2D edgVct; //todo
//   //calculate the distance to the closest obstacle from the current node
//   //obsDist =  voronoiDiagram.getDistance(node->getX(),node->getY())

//   if (obsDst < vorObsDMax) {
//     //calculate the distance to the closest GVD edge from the current node
//     // the node is away from the optimal free space area
//     if (edgDst > 0) {
//       float PobsDst_Pxi; //todo = obsVct / obsDst;
//       float PedgDst_Pxi; //todo = edgVct / edgDst;
//       float PvorPtn_PedgDst = alpha * obsDst * std::pow(obsDst - vorObsDMax, 2) / (std::pow(vorObsDMax, 2)
//                               * (obsDst + alpha) * std::pow(edgDst + obsDst, 2));

//       float PvorPtn_PobsDst = (alpha * edgDst * (obsDst - vorObsDMax) * ((edgDst + 2 * vorObsDMax + alpha)
//                                * obsDst + (vorObsDMax + 2 * alpha) * edgDst + alpha * vorObsDMax))
//                               / (std::pow(vorObsDMax, 2) * std::pow(obsDst + alpha, 2) * std::pow(obsDst + edgDst, 2));
//       gradient = wVoronoi * PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi;

//       return gradient;
//     }
//       return gradient;
//   }
//     return gradient;
// }

//###################################################
//                                     CURVATURE TERM
//###################################################
Vector2D Smoother::curvatureTerm(Vector2D xim1, Vector2D xi, Vector2D xip1) {
  Vector2D gradient;
  // the vectors between the nodes
  Vector2D Dxi = xi - xim1; // delta xi
  Vector2D Dxip1 = xip1 - xi; // delta x(i plus 1)
  // orthogonal complements vector
  Vector2D p1, p2;

  // the distance of the vectors
  float absDxi = Dxi.length();
  float absDxip1 = Dxip1.length();

  // ensure that the absolute values are not null
  if (absDxi > 0 && absDxip1 > 0) {
    // the angular change at the node
    float Dphi = std::acos(Utils::clamp(Dxi.dot(Dxip1) / (absDxi * absDxip1),-1,1));
    float kappa = Dphi / absDxi;

    // if the curvature is smaller then the maximum do nothing
    if (kappa <= kappaMax) {
      Vector2D zeros;
      return zeros;
    } else {
      if (-Dxip1.getX()*Dxi.getY()+Dxip1.getY()*Dxi.getX() < 0 )
        Dphi = -Dphi;
      float absDxiInv = 1 / absDxi;
      float absDxip1Inv = 1/absDxip1;
      float msinDphiInv = -1 / std::sin(Dphi);
      float u = absDxiInv * msinDphiInv;
      float v = Dphi*absDxiInv*absDxiInv;
      // calculate the p1 and p2 terms
      p1 = absDxiInv*absDxip1Inv*(xip1 - 2*xi + xim1) - 
           Dxi.dot(Dxip1)*absDxiInv*absDxiInv*absDxip1Inv*absDxip1Inv*(
             absDxip1*absDxiInv*(xi - xim1) + absDxi*absDxip1Inv*(xi - xip1));
      p2 = absDxiInv*(xi - xim1);
      // calculate the gradient
      gradient = 2*(Dphi*absDxiInv - kappaMax)*(u*p1 - v*p2);

      if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
        std::cout << "nan values in curvature term" << std::endl;
        Vector2D zeros;
        return zeros;
      }
      // return gradient of 0
      else {
        return gradient;
      }
    }
  }
  // return gradient of 0
  else {
    std::cout << "abs values not larger than 0" << std::endl;
    Vector2D zeros;
    return zeros;
  }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
  auto temp = (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
  return temp;
}

