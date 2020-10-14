#ifndef _HYBRID_A_STAR_VALIDITY_CHECKER_H
#define _HYBRID_A_STAR_VALIDITY_CHECKER_H

#include <memory>
#include <functional>
#include <utility>

#include "../common/collision/clsDetection.h"
#include "common/statespace/SE2State.h"

namespace HybridAStar
{
class ValidityChecker{
  public:
    ValidityChecker(const std::function<bool(const Common::SE2State*)>&);
    virtual bool isValid(const Common::SE2State *state) const {
      return checker_(state);
    }
  private:
    std::function<bool(const Common::SE2State*)> checker_;
};

class MotionChecker{
  private:
    std::function<bool(const Common::SE2State*, const Common::SE2State*)> checker_;
  public:
    MotionChecker(const std::function<bool(const Common::SE2State*, const Common::SE2State*)>&);
    virtual bool checkMotion(const Common::SE2State* s1, 
                            const Common::SE2State* s2) const {
                              return checker_(s1, s2);
                            }
};
} // namespace HybridAStar




#endif