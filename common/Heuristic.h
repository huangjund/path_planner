#ifndef _HYBRID_A_STAR_HEURISTIC_H_
#define _HYBRID_A_STAR_HEURISTIC_H_

namespace HybridAStar{
namespace Common{
  class Heuristic
  {
  private:
    
  public:
    Heuristic() = default;
    virtual ~Heuristic() = default;

    Heuristic(const Heuristic &) = delete;
    Heuristic &operator=(const Heuristic &) = delete;

    virtual double getDistance() = 0;
  };
} // namespace Common
} // namespace HybridAStar

#endif