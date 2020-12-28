#include "Helper.h"
#include <sstream>

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

  std::string writePath2String(const std::vector<bool>& forward, 
                               const std::vector<std::vector<Vector2D>>& path) {
    std::stringstream ss;
    auto d = forward.cbegin();
    for (auto p = path.cbegin(); p != path.cend(); ++d, ++p) {
      if (*d)
        ss << "F";
      else
        ss << "B";
      for(auto i = p->cbegin(); i < p->cend(); ++i) {
        ss << ";" << "B," << i->x << "," << i->y << "," << 0;
      }
      ss << "\n";
    }
    std::cout << ss.str() << std::endl;

    return ss.str();
  }
}
}