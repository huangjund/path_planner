#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.
   the angle needs to be within 0 to 360 deg when in this class
   Each node has a unique configuration (x, y, theta) in the configuration space C.
*/
class Node3D {
 public:
  /// The default constructor for 3D array initialization
  Node3D(): Node3D(0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node3D(float x, float y, float t, const Node3D* pred) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->pred = pred;
  }

  // GETTER METHODS
  /// get the x position
  float getX() const { return x; }
  /// get the y position
  float getY() const { return y; }
  /// get the heading theta
  float getT() const { return t; }
  /// determine whether the node is open
  const Node3D* getnodePred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const float& x) { this->x = x; }
  /// set the y position
  void setY(const float& y) { this->y = y; }
  /// set the heading theta
  void setT(const float& t) { this->t = t; }
  /// set a pointer to the predecessor of the node
  void setnodePred(const Node3D* pred) { this->pred = pred; }

  // GRID CHECKING
  /// Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height, const float gridCellSize) const;

  // CONSTANT VALUES
  /// Number of possible directions
  static const int dir;
  /// Possible movements in the x direction
  static const float dx[];
  /// Possible movements in the y direction
  static const float dy[];
  /// Possible movements regarding heading theta
  static const float dt[];

 private:
  /// the actual x position [unit: meters]
  float x;
  /// the actual y position [unit: meters]
  float y;
  /// the actual heading theta [unit: degree]
  float t;
  /// the predecessor pointer
  const Node3D* pred;


};
}
#endif // NODE3D_H
