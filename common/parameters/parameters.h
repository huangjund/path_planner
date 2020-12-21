#ifndef _HYBRID_A_STAR_PARAMETERS_H_
#define _HYBRID_A_STAR_PARAMETERS_H_

#include <cmath>

namespace HybridAStar {
namespace Common {
typedef struct {
  static constexpr bool isReversable_ = true;
  // [m] padding around the vechicle
  static constexpr double bloating_ = 0;
  static constexpr double width_ = 1.2+2*bloating_;
  static constexpr double length_ = 2.5+2*bloating_;
  // minimum turning radius of the vehicle
  static constexpr double rad_ = 0.6;
  /// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
  static constexpr float penaltyTurning_ = 1.05;
  /// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
  static constexpr float penaltyReversing_ = 2.0;
  /// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
  static constexpr float penaltyCOD_ = 2.0;

  /// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
  static constexpr float dubinsStepSize_ = 0.5;
  static constexpr double reedsheppStepSize_ = 0.5;
  /// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
  static constexpr int dubinsWidth_ = 1;
  /// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
  static constexpr int dubinsArea_ = dubinsWidth_ * dubinsWidth_;
} ForkProperty;

typedef struct {
  /// Number of possible directions
  static const int dir{3};
  /// Possible movements in the x direction
  static const double dx[];
  /// Possible movements in the y direction
  static const double dy[];
  /// Possible movements regarding heading theta
  static const double dt[];
  static constexpr int headings = 72;
  /// [c*M_PI] --- The discretization value of heading (goal condition)
  static constexpr float deltaHeadingRad = 2*M_PI/(float)headings;
  /// [°] --- The discretization value of the heading (goal condition)
  static constexpr float deltaHeadingDeg = 360/(float)headings;
  /// [c*M_PI] --- The heading part of the goal condition
  static constexpr float deltaHeadingNegRad = 2*M_PI - deltaHeadingRad;
  /// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
  static constexpr float dubinsShotDistance_ = 100;
} HaStarMotionPrimitives;

typedef struct {
  /// Number of possible directions
  static const int dir{8};
  /// Possible movements in the x direction
  static const int dx[];
  /// Possible movements in the y direction
  static const int dy[];
} aStarMotionPrimitives;

typedef struct {
  static const double cellSize{0.5};  // [unit: meters/cell]
  static const double angleSize{0.08726646};  // [unit: rad/cell]
  /*!
      \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell


      As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe 
      that the algorithm would prefer the predecessor rather than the successor.
      This would lead to the fact that the successor would never be placed and the 
      the one cell could only expand one node. The tieBreaker artificially increases 
      the cost of the predecessor to allow the successor being placed in the same cell.
    */
  static constexpr float tieBreaker_ = 0.01;
} PlanningMapConst;

typedef struct {
  static const double cellSize{0.01}; // [unit: meters/cell]/// [#] --- The sqrt of the number of discrete positions per cell
  static constexpr int positionResolution_ = 1;
  /// [#] --- The number of discrete positions per cell
  static constexpr int positions_ = positionResolution_ * positionResolution_;
} CollisionMapConst;


const double HaStarMotionPrimitives::dx[] = { 0.7,   0.6577848,   0.6577848};
const double HaStarMotionPrimitives::dy[] = { 0,     -0.2394141,  0.2394141};
const double HaStarMotionPrimitives::dt[] = { 0,  0.34906585,   -0.34906585};

const int aStarMotionPrimitives::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int aStarMotionPrimitives::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

} // namespace Common
} // namespace HybridAStar

#endif