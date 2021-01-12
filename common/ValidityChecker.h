#ifndef _HYBRID_A_STAR_VALIDITY_CHECKER_H
#define _HYBRID_A_STAR_VALIDITY_CHECKER_H

#include <memory>
#include <functional>
#include <utility>

#include "../common/collision/clsDetection.h"
#include "common/statespace/SE2State.h"

namespace HybridAStar
{
  /**
   * @brief check if the state collide
   * 
   */
class ValidityChecker{
  private:
    std::function<bool(const Common::SE2State*)> checker_;
  public:
    ValidityChecker(const std::function<bool(const Common::SE2State*)>&);
    virtual bool isValid(const Common::SE2State *state) const {
      return checker_(state);
    }
};

/**
 * @brief check if the motion between two states collide
 * 
 */
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