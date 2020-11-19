#ifndef _HYBRID_A_STAR_KD_TREE_H
#define _HYBRID_A_STAR_KD_TREE_H

/*
 * Reference to the following theory:
 *  https://en.wikipedia.org/wiki/K-d_tree
 *
 */

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <map>
#include <iostream>
#include <iterator>

#include "common/statespace/SE2State.h"

namespace HybridAStar {
// Point class used in RRTx
template <std::size_t N>
class Point {
 public:
  Point() = default;
  ~Point() = default;

  Point(const Point<N>& p) { *this = p;}
  
  Point& operator=(const Point<N>& p) {
    for (size_t i = 0; i < N; i++) coords[i] = p.coords[i];
    g = p.g; lmc = p.lmc;
    prtTreePositive = p.prtTreePositive;
    cldTreeNeg = p.cldTreeNeg;
    nOrgNeg = p.nOrgNeg;
    nOrgPositive = p.nOrgPositive;
    nRunPositive = p.nRunPositive;
    nRunNeg = p.nRunNeg;
    return *this;
  }
  // Types representing iterators that can traverse and optionally modify the elements of the Point.
  typedef double* iterator;
  typedef const double* const_iterator;

  // Returns N, the dimension of the point.
  std::size_t size() const;

  // Queries or retrieves the value of the point at a particular point. The index is assumed to be in-range.
  double& operator[](std::size_t index);
  double operator[](std::size_t index) const;

  // Returns iterators delineating the full range of elements in the Point.
  iterator begin();
  iterator end();
  const_iterator begin() const;
  const_iterator end() const;

  double& getMutableG() {return g;}
  const double getG() const {return g;}
  double& getMutableLMC() {return lmc;}
  const double getLMC() const {return lmc;}
  void setCost(const double& g, const double& lmc) {getMutableG() = g; getMutableLMC() = lmc;}

  std::shared_ptr<Point<N>> prtTreePositive;  // parent tree positive
  std::vector<std::shared_ptr<Point<N>>> cldTreeNeg; // Child tree negative
  std::vector<std::shared_ptr<Point<N>>> nOrgPositive;  // original neighbors positive
  std::vector<std::shared_ptr<Point<N>>> nOrgNeg; // original neighbors negative
  std::vector<std::shared_ptr<Point<N>>> nRunPositive; // running neighbors positive
  std::vector<std::shared_ptr<Point<N>>> nRunNeg; // running neighbors negative
 private:
  double coords[N];
  double g;
  double lmc;
};


// Returns the Euclidean distance between two points.
template <std::size_t N>
double Distance(const Point<N>& one, const Point<N>& two);


// Returns whether two points are equal / not equal
template <std::size_t N>
bool operator==(const Point<N>& one, const Point<N>& two);

template <std::size_t N>
bool operator!=(const Point<N>& one, const Point<N>& two);


/**
 * An implementation of the bounded priority queue abstraction.
 * A bounded priority queue is in many ways like a regular priority
 * queue.  It stores a collection of elements tagged with a real-
 * valued priority, and allows for access to the element whose
 * priority is the smallest.  However, unlike a regular priority
 * queue, the number of elements in a bounded priority queue has
 * a hard limit that is specified in the constructor.  Whenever an
 * element is added to the bounded priority queue such that the
 * size exceeds the maximum, the element with the highest priority
 * value will be ejected from the bounded priority queue.  In this
 * sense, a bounded priority queue is like a high score table for
 * a video game that stores a fixed number of elements and deletes
 * the least-important entry whenever a new value is inserted.
 *
 * When creating a bounded priority queue, you must specify the
 * maximum number of elements to store in the queue as an argument
 * to the constructor.  For example:
 *
 * BoundedPQueue<int> bpq(15); // Holds up to fifteen values.
 *
 * The maximum size of the bounded priority queue can be obtained
 * using the maxSize() function, as in
 *
 * size_t k = bpq.maxSize();
 *
 * Beyond these restrictions, the bounded priority queue behaves
 * similarly to other containers.  You can query its size using
 * size() and check whether it is empty using empty().  You
 * can enqueue an element into the bounded priority queue by
 * writing
 *
 * bpq.enqueue(elem, priority);
 *
 * Note that after enqueuing the element, there is no guarantee
 * that the value will actually be in the queue.  If the queue
 * is full and the new element's priority exceeds the largest
 * priority in the container, it will not be added.
 *
 * You can dequeue elements from a bounded priority queue using
 * the dequeueMin() function, as in
 *
 * int val = bpq.dequeueMin();
 *
 * The bounded priority queue also allows you to query the min
 * and max priorities of the values in the queue.  These values
 * can be queried using the best() and worst() functions, which
 * return the smallest and largest priorities in the queue,
 * respectively.
 */
template <typename T>
class BoundedPQueue {
 public:
  // Constructor: BoundedPQueue(size_t maxSize);
  // Usage: BoundedPQueue<int> bpq(15);
  // --------------------------------------------------
  // Constructs a new, empty BoundedPQueue with
  // maximum size equal to the constructor argument.
  ///
  explicit BoundedPQueue(std::size_t maxSize);
  explicit BoundedPQueue(const std::multimap<double, T>& t);
  ~BoundedPQueue() = default;

  // void enqueue(const T& value, double priority);
  // Usage: bpq.enqueue("Hi!", 2.71828);
  // --------------------------------------------------
  // Enqueues a new element into the BoundedPQueue with
  // the specified priority. If this overflows the maximum
  // size of the queue, the element with the highest
  // priority will be deleted from the queue. Note that
  // this might be the element that was just added.
  void enqueue(const T value, double priority);

  // T dequeueMin();
  // Usage: int val = bpq.dequeueMin();
  // --------------------------------------------------
  // Returns the element from the BoundedPQueue with the
  // smallest priority value, then removes that element
  // from the queue.
  T dequeueMin();

  // size_t size() const;
  // bool empty() const;
  // Usage: while (!bpq.empty()) { ... }
  // --------------------------------------------------
  // Returns the number of elements in the queue and whether
  // the queue is empty, respectively.
  std::size_t size() const;
  bool empty() const;

  // size_t maxSize() const;
  // Usage: size_t queueSize = bpq.maxSize();
  // --------------------------------------------------
  // Returns the maximum number of elements that can be
  // stored in the queue.
  std::size_t maxSize() const;

  // double best() const;
  // double worst() const;
  // Usage: double highestPriority = bpq.worst();
  // --------------------------------------------------
  // best() returns the smallest priority of an element
  // stored in the container (i.e. the priority of the
  // element that will be dequeued first using dequeueMin).
  // worst() returns the largest priority of an element
  // stored in the container.  If an element is enqueued
  // with a priority above this value, it will automatically
  // be deleted from the queue.  Both functions return
  // numeric_limits<double>::infinity() if the queue is
  // empty.
  double best()  const;
  double worst() const;

  T& operator[](size_t n) {
    auto p = elems.begin();
    while (n-- && p!=elems.end()) {
      ++p;
    }
    return p->second;
  }

  const T& operator[](size_t n) const {
    auto p = elems.cbegin();
    while (n-- && p!=elems.cend()) {
      ++p;
    }
    return p->second;
  }

 private:
  // This class is layered on top of a multimap mapping from priorities
  // to elements with those priorities.
  std::multimap<double, T> elems;
  std::size_t maximumSize;
};

template class BoundedPQueue<std::shared_ptr<Point<2>>>;

// the ElemType should be a label of a point
// if points are used by knn algorithm, this label could be an unsigned int to verify a set
// here, we take it as a coordinate of the point
// in default: "same point", in description of the node in kdtree, means the two points 
// is absolutely same or saying they are in the same memory. So, we use the pointer to decide
// whether two points are "same point".
// reference to https://www.cnblogs.com/earendil/p/8135074.html
template <std::size_t N>
class KDTree {
 public:
  using PointsPointerVec = std::vector<std::shared_ptr<Point<N>>>;
  // Constructs an empty KDTree.
  KDTree();

  // Efficiently build a balanced KD-tree from a large set of points
  KDTree(PointsPointerVec& points);

  // Frees up all the dynamically allocated resources
  ~KDTree();

  // Deep-copies the contents of another KDTree into this one.
  KDTree(const KDTree& rhs);
  KDTree& operator=(const KDTree& rhs);

  // Returns the dimension of the points stored in this KDTree.
  std::size_t dimension() const;

  // Returns the number of elements in the kd-tree and whether the tree is empty
  std::size_t size() const;
  bool empty() const;

  // Returns whether the specified point is contained in the KDTree.
  bool contains(const std::shared_ptr<Point<N>>& pt) const;

  /*
    * Inserts the point pt into the KDTree, associating it with the specified value.
    * If the element already existed in the tree, the new value will overwrite the existing one.
    */
  void insert(const std::shared_ptr<Point<N>>& pt);

  // /*
  //   * Returns a reference to the value associated with point pt in the KDTree.
  //   * If the point does not exist, then it is added to the KDTree using the
  //   * default value of ElemType as its key.
  //   */
  // ElemType& operator[](const Point<N>& pt);

  // /*
  //   * Returns a reference to the key associated with the point pt. If the point
  //   * is not in the tree, this function throws an out_of_range exception.
  //   */
  // ElemType& at(const Point<N>& pt);
  // const ElemType& at(const Point<N>& pt) const;

  /*
    * Given a point v and an integer k, finds the k points in the KDTree
    * nearest to v and returns the most common value associated with those
    * points. In the event of a tie, one of the most frequent value will be chosen.
    */
  BoundedPQueue<std::shared_ptr<Point<N>>> kNNValue(const Point<N>& key, std::size_t k) const;

  /**
   * @brief find points within a specific radius
   * 
   * @param key 
   * @param radius 
   * @return BoundedPQueue<Point<N>> 
   */
  BoundedPQueue<std::shared_ptr<Point<N>>> kNNValue(const Point<N>& key, double radius) const;

  std::shared_ptr<Point<N>> kNNValue(const Point<N>& key, 
                                std::function<bool(const Common::SE2State* s1, 
                                                  const Common::SE2State* s2)> condition) const;

  std::shared_ptr<Point<N>> rootNode() {return root_->point;}

 private:
  struct Node {
      std::shared_ptr<Point<N>> point;
      Node *left;
      Node *right;
      int level;  // level of the node in the tree, starts at 0 for the root
      Node(const std::shared_ptr<Point<N>>& _pt, int _level):
          point(_pt), left(NULL), right(NULL), level(_level) {}
  };

  // Root node of the KD-Tree
  Node* root_;

  // Number of points in the KD-Tree
  std::size_t size_;

  /*
    * Recursively build a subtree that satisfies the KD-Tree invariant using points in [start, end)
    * At each level, we split points into two halves using the median of the points as pivot
    * The root of the subtree is at level 'currLevel'
    * O(n) time partitioning algorithm is used to locate the median element
    */
  Node* buildTree(typename PointsPointerVec::iterator start,
                  typename PointsPointerVec::iterator end, int currLevel);

  /*
    * Returns the Node that contains Point pt if it is present in subtree 'currNode'
    * Returns the Node below which pt should be inserted if pt is not in the subtree
    */
  Node* findNode(Node* currNode, const std::shared_ptr<Point<N>>& pt) const;

  // Recursive helper method for kNNValue(pt, k)
  void nearestNeighborRecurse(const Node* currNode,
                              const Point<N>& key, 
                              BoundedPQueue<std::shared_ptr<Point<N>>>& pQueue) const;

  void nearestNeighborRecurse(const Node* currNode, 
                              const Point<N>& key, 
                              double radius,
                              std::multimap<double, std::shared_ptr<Point<N>>>& pBucket) const;

  void nearestNeighborRecurse(const Node* currNode,
                              const Point<N>& key,
                              std::shared_ptr<Point<N>>& p,
                              std::function<bool(const Common::SE2State* s1, 
                                                  const Common::SE2State* s2)>& condition) const;
  /*
    * Recursive helper method for copy constructor and assignment operator
    * Deep copies tree 'root' and returns the root of the copied tree
    */
  Node* deepcopyTree(Node* root);

  // Recursively free up all resources of subtree rooted at 'currNode'
  void freeResource(Node* currNode);
};

template class KDTree<2>;

} // namespace HybridAStar

#endif