#include "ValidityChecker.h"

namespace HybridAStar {
ValidityChecker::ValidityChecker(const std::function<bool(const Common::State*)>& fn) : 
                                checker_(std::move(fn)) {}

MotionChecker::MotionChecker(const std::function<bool(const Common::State*, const Common::State*)>& fn) :
              checker_(std::move(fn)) {}

} // namespace HybridAStar