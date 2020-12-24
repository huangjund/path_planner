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

  std::string writePath2String(const std::vector<std::pair<double, double>>& path) {
    std::stringstream ss;
    ss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" << "<Paths>\n";
    ss << "<Path No=\"1\" Detail=\"F";
    for(auto point : path) {
      ss << ";" << "B," << point.first << "," << point.second << "," << 0;
    }
    ss << "\" />\n</Paths>";

    return ss.str();
  }
}
}