#ifndef _HYBRID_A_STAR_MAP_H_
#define _HYBRID_A_STAR_MAP_H_

namespace HybridAStar {
namespace Common {

  /**
   * @brief a piece of white map
   * 
   * @tparam T double or int
   */
  template <class T>
  class Map {
   public:
    Map() = default;
    explicit Map(T length, T width);
    virtual ~Map() = default;

    Map(const Map<T>& m);
    Map<T>& operator=(const Map<T>& rhs);

    void setMap(T length, T width) { width_ = length; height_ = width;}
    T getWid() const { return width_; }
    T& getMutableWid() { return width_; }
    T getHeight() const { return height_; }
    T& getMutableHeight() {return height_; }

   protected:
    T width_{0};
    T height_{0};
  };

} // namespace Common
} // namespace HybridAStar

#endif