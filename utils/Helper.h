#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <math/vector2d.h>

namespace HybridAStar {
/*!
    \brief The namespace that wraps helper.h
    \namespace Helper
*/
namespace Utils {

/*!
   \fn  float normalizeHeading(float t)
   \brief Normalizes a heading given in degrees to (0,360]
   \param t heading in degrees
*/
 float normalizeHeading(float t);

/*!
   \fn float normalizeHeadingRad(float t)
   \brief Normalizes a heading given in rad to (0,2PI]
   \param t heading in rad
*/
inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

/*!
   \fn float toDeg(float t)
   \brief Converts and normalizes a heading given in rad to deg
   \param t heading in deg
*/
 float toDeg(float t);

/*!
   \fn float toRad(float t)
   \brief Converts and normalizes a heading given in deg to rad
   \param t heading in rad
*/
 float toRad(float t);

/*!
   \fn float clamp(float n, float lower, float upper)
   \brief Clamps a number between a lower and an upper bound
*/
inline float clamp(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

/**
 * @brief 
 * 
 */
  std::string writePath2String(const std::vector<bool>& forward,
                               const std::vector<std::vector<Vector2D>>& path);
}
}