#pragma once

#include <cmath>

#include "node3d.h"
#include "constants.h"
#include "helper.h"

namespace HybridAStar {
class PlanMapNode : public Node3D {
public:
	PlanMapNode():PlanMapNode(0,0,0,0,0,nullptr){}
	PlanMapNode(float x, float y, float t, float g, float h, const PlanMapNode* pred, int prim = 0):
							Node3D(x*planmapCellSize,y*planmapCellSize,t*Constants::deltaHeadingDeg,pred){
								this->g = g;
								this->h = h;
								this->o = false;
								this->c = false;
								this->idx = -1;
								this->prim = prim;
							}
	~PlanMapNode();

	/// ============GET===============
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

	
	/// ==========SET==================
	/// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }
  /// set and get the index of the node in the 3D grid
  int setIdx(int width, int height) { this->idx = (int)(t / Constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); return idx;}
  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }

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

  private:
  /// the relative x position, [unit: cells]
  float x;
  /// the relative y position, [unit: cells]
  float y;
  /// the relative heading theta, [unit: pieces]
  float t;
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
  /// the size of planning map cell [unit:meters]
  const float planmapCellSize = 0.5; 
};
} // namespace HybridAStar
