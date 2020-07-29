#include "planningmap.h"

using namespace HybridAStar;

//###################################################
//                                        IS IN RANGE
//###################################################
// TODO: need to be changed to varying according to heuristics
bool PlanMapNode::isInRange(const PlanMapNode& goal) const {
  int random = rand() % 10 + 1;
  float dx = std::abs(rx - goal.rx) / random;
  float dy = std::abs(ry - goal.ry) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotTrigDistance;
}

//###################################################
//                                      MOVEMENT COST
//###################################################
// TODO: penalization of cos so far is not reasonable
void PlanMapNode::updateG() {
  // forward driving
  if (prim < 3) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim > 2) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning;
      }
    } else {
      g += dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < 3) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } else {
      g += dx[0] * Constants::penaltyReversing;
    }
  }
}


//###################################################
//                                 3D NODE COMPARISON
//###################################################
bool PlanMapNode::operator == (const PlanMapNode& rhs) const {
  return (int)rx == (int)rhs.rx &&
         (int)ry == (int)rhs.ry &&
         (int)rt == (int)rhs.rt;
        //  (std::abs(rt - rhs.rt) <= Constants::deltaHeadingRad ||
        //   std::abs(rt - rhs.rt) >= Constants::deltaHeadingNegRad);
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
PlanMapNode* PlanMapNode::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;
  float theta = this->getT()*M_PI/180; // get the actual angle
  float x = this->getX(); // get the actual x position
  float y = this->getY(); // get the actual y position

  //_______________________________________
  // calculate successor positions forward
  // transformation
  //   / cos(t) -sin(t) \ / dx \ 
  //   \ sin(t)  cos(t) / \ dy /
  //_______________________________________
  if (i < 3) {
    xSucc = (x + dx[i]*cos(theta) - dy[i]*sin(theta))/planmapCellSize;
    ySucc = (y + dx[i]*sin(theta) + dy[i]*cos(theta))/planmapCellSize;
    tSucc = Helper::Rad2Deg(theta + dt[i]); // normalized when transformed to degree
  }
  // backwards
  else {
    xSucc = (x - dx[i - 3]*cos(theta) - dy[i - 3]*sin(theta))/planmapCellSize;
    ySucc = (y - dx[i - 3]*sin(theta) + dy[i - 3]*cos(theta))/planmapCellSize;
    tSucc = Helper::Rad2Deg(theta - dt[i - 3]); // normalized when transformed to degree
  }

  return new PlanMapNode(xSucc, ySucc, tSucc, g, 0, this, i);
}