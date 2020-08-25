#pragma once

#include <cmath>

namespace HybridAStar{
namespace Multibody{
  class SingleForkLiftPlant
  {
  public:
    static constexpr bool isReversable_ = true;
    // [m] padding around the vechicle
    static constexpr double bloating_ = 0;
    static constexpr double width_ = 1.5+2*bloating_;
    static constexpr double length_ = 2.8+2*bloating_;
    // minimum turning radius of the vehicle
    static constexpr double rad_ = 0.6;
    static constexpr int headings_ = 72;
    /// [°] --- The discretization value of the heading (goal condition)
    static constexpr float deltaHeadingDeg_ = 360 / (float)headings_;
    /// [c*M_PI] --- The discretization value of heading (goal condition)
    static constexpr float deltaHeadingRad_ = 2 * M_PI / (float)headings_;
    /// [c*M_PI] --- The heading part of the goal condition
    static constexpr float deltaHeadingNegRad_ = 2 * M_PI - deltaHeadingRad_;
    /// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
    static constexpr float penaltyTurning_ = 1.05;
    /// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
    static constexpr float penaltyReversing_ = 2.0;
    /// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
    static constexpr float penaltyCOD_ = 2.0;
    /// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
    // TODO: need to change
    static constexpr float dubinsShotDistance_ = 100;
    /// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
    static constexpr float dubinsStepSize_ = 0.5;
    static constexpr double reedsheppStepSize_ = 0.5;
    /// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
    static constexpr int dubinsWidth_ = 1;
    /// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
    static constexpr int dubinsArea_ = dubinsWidth_ * dubinsWidth_;
    /// [#] --- The sqrt of the number of discrete positions per cell
    static constexpr int positionResolution_ = 1;
    /// [#] --- The number of discrete positions per cell
    static constexpr int positions_ = positionResolution_ * positionResolution_;
    /*!
      \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell


      As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
      This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
      to allow the successor being placed in the same cell.
    */
    static constexpr float tieBreaker_ = 0.01;

    static constexpr float planResolution = 0.5; // [unit: meters/cell]

    static constexpr float planAngleResolution = 2*M_PI/headings_; // [unit : rad/piece]

    
  public:
    SingleForkLiftPlant() = default;
    ~SingleForkLiftPlant() = default;
    SingleForkLiftPlant(const SingleForkLiftPlant &) = delete;
    SingleForkLiftPlant &operator=(const SingleForkLiftPlant &) = delete;
  };
} // namespace Multibody
} // namespace HybridAStar
