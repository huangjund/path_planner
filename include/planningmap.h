#pragma once

#include <cmath>

#include "node3d.h"
#include "constants.h"
#include "helper.h"

namespace HybridAStar {
class PlanMapNode : public Node3D {
  // TODO: in class normalization
  // param t should be normalized before sent to this class
public:
	PlanMapNode():PlanMapNode(0,0,0,0,0,nullptr){}
	PlanMapNode(float x, float y, float t, float g, float h, const PlanMapNode* pred, int prim = 0):
							Node3D(x*planmapCellSize, y*planmapCellSize, t*planmapAngleSize,pred){
                this->rx = x;
                this->ry = y;
                this->rt = t;
								this->g = g;
								this->h = h;
                this->pred = pred;
								this->o = false;
								this->c = false;
								this->idx = -1;
								this->prim = prim;
							}
	~PlanMapNode();

	/// ============GET===============
  // get the relative x value [unit:cells]
  float getrx() const {return rx;}
  // get the relative y value [unit:cells]
  float getry() const {return ry;}
  // get the relative t value [unit:cells]
  float getrt() const {return rt;}
	/// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  // get predecessor
  const PlanMapNode* getPred() const {return pred;}

	
	/// ==========SET==================
  /// set relative x
  void setrx(const float x) {this->rx = x; this->setX(x*planmapCellSize);}
  /// set relative y
  void setry(const float y) {this->ry = y; this->setX(y*planmapCellSize);}
  /// set relative t
  void setrt(const float t) {this->rt = t; this->setX(t*planmapAngleSize);}
	/// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }
  /// set and get the index of the node in the 3D grid
  int setIdx(int width, int height) { this->idx = (int)(rt)*width*height + (int)(ry)*width + (int)(rx); return idx;}
  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }
  /// set PREDECESSOR
  void setPred(const PlanMapNode* pred) {this->pred = pred;this->setnodePred(pred);}

	// UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG();

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator == (const PlanMapNode& rhs) const;

  // RANGE CHECKING
  /// Determines whether it is appropriate to find an analytical solution.
  bool isInRange(const PlanMapNode& goal) const;

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  PlanMapNode* createSuccessor(const int i);

public:
  /// the size of planning map cell [unit:meters/cell]
  const static float planmapCellSize = 0.5;
  /// the size of planning map angle piece [unit:degree/piece]
  const static float planmapAngleSize = Constants::deltaHeadingDeg;

private:
  /// the relative x position, [unit: cells]
  float rx;
  /// the relative y position, [unit: cells]
  float ry;
  /// the relative heading theta, [unit: pieces]
  float rt;
  /// the cost-so-far
	float g;
  /// the cost-to-go
  float h;
  /// the index of the node in the 3D array
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the predecessor pointer
  const PlanMapNode* pred;
  /// the motion primitive of this node from its predecessor
  int prim;
};
} // namespace HybridAStar
