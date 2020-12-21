#ifndef _HYBRID_A_STAR_VALIDITY_CHECKER_H
#define _HYBRID_A_STAR_VALIDITY_CHECKER_H

#include <memory>
#include <functional>
#include <utility>

#include "common/statespace/SE2State.h"

namespace HybridAStar
{
class ValidityChecker{
  private:
    std::function<bool(const Common::State*)> checker_;
  public:
    ValidityChecker(const std::function<bool(const Common::State*)>&);
    virtual bool isValid(const Common::State* state) const {
      return checker_(state);
    }
};

class MotionChecker{
  private:
    std::function<bool(const Common::State*, const Common::State*)> checker_;
  public:
    MotionChecker(const std::function<bool(const Common::State*, const Common::State*)>&);
    virtual bool checkMotion(const Common::State* s1, 
                             const Common::State* s2) const {
      return checker_(s1, s2);
    }
};
} // namespace HybridAStar

#endif