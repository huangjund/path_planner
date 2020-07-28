#include "planningmap.h"

using namespace HybridAStar;

//###################################################
//                                        IS IN RANGE
//###################################################
bool PlanMapNode::isInRange(const PlanMapNode& goal) const {
  int random = rand() % 10 + 1;
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotTrigDistance;
}

//###################################################
//                                      MOVEMENT COST
//###################################################
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
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
PlanMapNode* PlanMapNode::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor positions forward
  if (i < 3) {
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else {
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
  }

  return new PlanMapNode(xSucc, ySucc, tSucc, g, 0, this, i);
}