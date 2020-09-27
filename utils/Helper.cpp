#include "Helper.h"

namespace HybridAStar {
namespace Utils {
  float normalizeHeading(float t) {
    if ((int)t <= 0 || (int)t >= 360) {
      if (t < -0.1) {
        t += 360.f;
      } else if ((int)t >= 360) {
        t -= 360.f;
      } else {
        t =  0;
      }
    }
    return t;
  }

  float toDeg(float t) {
    return normalizeHeadingRad(t) * 180.f / M_PI ;
  }

  float toRad(float t) {
    return normalizeHeadingRad(t / 180.f * M_PI);
  }
}
}