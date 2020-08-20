#pragma once

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
  };
} // namespace Common
} // namespace HybridAStar
