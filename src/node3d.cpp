#include "node3d.h"

using namespace HybridAStar;

// possible directions
const int Node3D::dir = 3;

// R = 6, 6.75 DEG
const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097}; // unit: rad

//###################################################
//                                         IS ON GRID
//###################################################
bool Node3D::isOnGrid(const int gridWidth, const int gridHeight, const float gridCellSize) const {
  return x >= 0 && y >= 0 &&
         x < gridWidth*gridCellSize && y < gridHeight*gridCellSize;
}
