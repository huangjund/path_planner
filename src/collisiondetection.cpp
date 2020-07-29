#include "collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) {
  int X = (int)(x/Constants::collisionMapCellSize);
  int Y = (int)(y/Constants::collisionMapCellSize);
  int iX = (int)((x - (long)x)*Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y)*Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t/Constants::deltaHeadingDeg);
  int idx = iY*Constants::positionResolution*Constants::vehicleHeadings + iX*Constants::vehicleHeadings + iT;
  int cX; // [unit: cell]
  int cY; // [unit: cell]

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].contour[i].x);
    cY = (Y + collisionLookup[idx].contour[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {
        return false;
      }
    }
  }

  return true;
}
