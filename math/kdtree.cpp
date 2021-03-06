#include "kdtree.h"

namespace HybridAStar {
template <std::size_t N>
KDTree<N>::KDTree() :
    root_(NULL), size_(0) { }

template <std::size_t N>
typename KDTree<N>::Node* KDTree<N>::deepcopyTree(typename KDTree<N>::Node* root) {
    if (root == NULL) return NULL;
    Node* newRoot = new Node(*root);
    newRoot->left = deepcopyTree(root->left);
    newRoot->right = deepcopyTree(root->right);
    return newRoot;
}

template <std::size_t N>
typename KDTree<N>::Node* KDTree<N>::buildTree(typename PointsPointerVec::iterator start,
                                                typename PointsPointerVec::iterator end, int currLevel) {
    if (start >= end) return NULL; // empty tree

    int axis = currLevel % N; // the axis to split on
    auto cmp = [axis](const std::shared_ptr<Point<N>>& p1, 
                      const std::shared_ptr<Point<N>>& p2) {
        return (*p1)[axis] < (*p2)[axis];
    };
    std::size_t len = end - start;
    auto mid = start + len / 2;
    std::nth_element(start, mid, end, cmp); // linear time partition

    // move left (if needed) so that all the equal points are to the right
    // The tree will still be balanced as long as there aren't many points that are equal along each axis
    while (mid > start && (mid - 1)[axis] == mid[axis]) {
        --mid;
    }

    Node* newNode = new Node(*mid, currLevel);
    newNode->left = buildTree(start, mid, currLevel + 1);
    newNode->right = buildTree(mid + 1, end, currLevel + 1);
    return newNode;
}

template <std::size_t N>
KDTree<N>::KDTree(PointsPointerVec& points) {
    root_ = buildTree(points.begin(), points.end(), 0);
    size_ = points.size();
}

template <std::size_t N>
KDTree<N>::KDTree(const KDTree& rhs) {
    root_ = deepcopyTree(rhs.root_);
    size_ = rhs.size_;
}

template <std::size_t N>
KDTree<N>& KDTree<N>::operator=(const KDTree& rhs) {
    if (this != &rhs) { // make sure we don't self-assign
        freeResource(root_);
        root_ = deepcopyTree(rhs.root_);
        size_ = rhs.size_;
    }
    return *this;
}

template <std::size_t N>
void KDTree<N>::freeResource(typename KDTree<N>::Node* currNode) {
    if (currNode == NULL) return;
    freeResource(currNode->left);
    freeResource(currNode->right);
    delete currNode;
}

template <std::size_t N>
KDTree<N>::~KDTree() {
    freeResource(root_);
}

template <std::size_t N>
std::size_t KDTree<N>::dimension() const {
    return N;
}

template <std::size_t N>
std::size_t KDTree<N>::size() const {
    return size_;
}

template <std::size_t N>
bool KDTree<N>::empty() const {
    return size_ == 0;
}

template <std::size_t N>
typename KDTree<N>::Node* KDTree<N>::findNode(typename KDTree<N>::Node* currNode,
                                              const std::shared_ptr<Point<N>>& pt) const {
    if (currNode == NULL || currNode->point == pt) return currNode;

    const auto& currPoint = currNode->point;
    int currLevel = currNode->level;
    if ((*pt)[currLevel%N] < (*currPoint)[currLevel%N]) { // recurse to the left side
        return currNode->left == NULL ? currNode : findNode(currNode->left, pt);
    } else { // recurse to the right side
        return currNode->right == NULL ? currNode : findNode(currNode->right, pt);
    }
}

template <std::size_t N>
bool KDTree<N>::contains(const std::shared_ptr<Point<N>>& pt) const {
    auto node = findNode(root_, pt);
    return node != NULL && node->point == pt;
}

template <std::size_t N>
void KDTree<N>::insert(const std::shared_ptr<Point<N>>& pt) {
    auto targetNode = findNode(root_, pt);
    if (targetNode == NULL) { // this means the tree is empty
        root_ = new Node(pt, 0);
        size_ = 1;
    } else {
        if (targetNode->point == pt) { // pt is already in the tree, simply update its value
            std::cout << "point already in kd tree" << std::endl;
        } else { // construct a new node and insert it to the right place (child of targetNode)
            int currLevel = targetNode->level;
            Node* newNode = new Node(pt, currLevel + 1);
            if ((*pt)[currLevel%N] < (*targetNode->point)[currLevel%N]) {
                targetNode->left = newNode;
            } else {
                targetNode->right = newNode;
            }
            ++size_;
        }
    }
}

// template <std::size_t N>
// const ElemType& KDTree<N>::at(const Point<N>& pt) const {
//     auto node = findNode(root_, pt);
//     if (node == NULL || node->point != pt) {
//         throw std::out_of_range("Point not found in the KD-Tree");
//     } else {
//         return node->value;
//     }
// }

// template <std::size_t N>
// ElemType& KDTree<N>::at(const Point<N>& pt) {
//     const KDTree<N>& constThis = *this;
//     return const_cast<ElemType&>(constThis.at(pt));
// }

// template <std::size_t N>
// ElemType& KDTree<N>::operator[](const Point<N>& pt) {
//     auto node = findNode(root_, pt);
//     if (node != NULL && node->point == pt) { // pt is already in the tree
//         return node->value;
//     } else { // insert pt with default ElemType value, and return reference to the new ElemType
//         insert(pt);
//         if (node == NULL) return root_->value; // the new node is the root
//         else return (node->left != NULL && node->left->point == pt) ? node->left->value: node->right->value;
//     }
// }


// completed
template <std::size_t N>
void KDTree<N>::nearestNeighborRecurse(const typename KDTree<N>::Node* currNode,
                                        const Point<N>& key, 
                                        BoundedPQueue<std::shared_ptr<Point<N>>>& pQueue) const {
    if (currNode == NULL) return;
    const auto& currPoint = currNode->point;

    // Add the current point to the BPQ if it is closer to 'key' that some point in the BPQ
    // @param: value, priority
    // TODO: this sentence should not be here
    pQueue.enqueue(currNode->point, Distance(*currPoint, key));

    // Recursively search the half of the tree that contains Point 'key'
    int currLevel = currNode->level;
    bool isLeftTree;
    if (key[currLevel%N] < (*currPoint)[currLevel%N]) {
        nearestNeighborRecurse(currNode->left, key, pQueue);
        isLeftTree = true;
    } else {
        nearestNeighborRecurse(currNode->right, key, pQueue);
        isLeftTree = false;
    }

    if (pQueue.size() < pQueue.maxSize() || fabs(key[currLevel%N] - (*currPoint)[currLevel%N]) < pQueue.worst()) {
        // Recursively search the other half of the tree if necessary
        if (isLeftTree) nearestNeighborRecurse(currNode->right, key, pQueue);
        else nearestNeighborRecurse(currNode->left, key, pQueue);
    }
}

template <std::size_t N>
BoundedPQueue<std::shared_ptr<Point<N>>> KDTree<N>::kNNValue(const Point<N>& key, std::size_t k) const {
    BoundedPQueue<std::shared_ptr<Point<N>>> pQueue(k); // BPQ with maximum size k
    if (empty()) return BoundedPQueue<std::shared_ptr<Point<N>>>(0); // default return value if KD-tree is empty

    // Recursively search the KD-tree with pruning
    nearestNeighborRecurse(root_, key, pQueue);

    return pQueue;
}


// completed
template <std::size_t N>
void KDTree<N>::nearestNeighborRecurse(const Node* currNode, 
                                        const Point<N>& key, 
                                        double radius,
                                        std::multimap<double, std::shared_ptr<Point<N>>>& pBucket) const {
  if (currNode == NULL) return;
  const auto& currPoint = currNode->point;

  auto dis = Distance(*currPoint, key);
  if (dis < radius) 
    pBucket.insert(std::make_pair(dis, currPoint));

  int currLevel = currNode->level;
  bool isLeftTree;
  if (key[currLevel%N] < (*currPoint)[currLevel%N]) {
    nearestNeighborRecurse(currNode->left, key, radius, pBucket);
    isLeftTree = true;
  } else {
    nearestNeighborRecurse(currNode->right, key, radius, pBucket);
    isLeftTree = false;
  }

  if (fabs(key[currLevel%N] - (*currPoint)[currLevel%N]) < radius) {
    if (isLeftTree) nearestNeighborRecurse(currNode->right, key, radius, pBucket);
    else nearestNeighborRecurse(currNode->left, key, radius, pBucket);
  }
}

// incomplete
// searching for points within radius r
template <std::size_t N>
BoundedPQueue<std::shared_ptr<Point<N>>> KDTree<N>::kNNValue(const Point<N>& key, double radius) const {
    std::multimap<double, std::shared_ptr<Point<N>>> bucket;   // create an ordered map as a temp container
    if (empty()) return BoundedPQueue<std::shared_ptr<Point<N>>>(0);    // default return value if KD-tree is empty

    nearestNeighborRecurse(root_, key, radius, bucket);

    return BoundedPQueue<std::shared_ptr<Point<N>>>(bucket);
}

template <std::size_t N>
void KDTree<N>::nearestNeighborRecurse(const Node* currNode,
                                    const Point<N>& key,
                                    std::shared_ptr<Point<N>>& p,
                                    std::function<bool(const Common::SE2State* s1, 
                                                        const Common::SE2State* s2)>& condition) const {
    if (currNode == NULL) return;
    const auto& currPoint = currNode->point;

    bool isLeftTree;
    int currLevel = currNode->level;
    if (key[currLevel%N] < (*currPoint)[currLevel%N]) {
        nearestNeighborRecurse(currNode->left, key, p, condition);
        isLeftTree = true;
    }else {
        nearestNeighborRecurse(currNode->right, key, p, condition);
        isLeftTree = false;
    }

    static double maxRadius = Distance(key, *currPoint);

    // assume there must be a point which can connect with key
    auto v0 = std::make_unique<Common::SE2State>(key[0],key[1],0);
    auto u0 = std::make_unique<Common::SE2State>((*currPoint)[0],(*currPoint)[1],0);
    if (condition(v0.get(), u0.get())) {
        if (p == NULL) {
            p = currNode->point;
        }
        if (fabs((*p)[currLevel%N] - (*currPoint)[currLevel%N]) < Distance(*p,key)) {
            if (isLeftTree) nearestNeighborRecurse(currNode->right, key, p, condition);
            else nearestNeighborRecurse(currNode->left, key, p, condition);
        }
    } else if (p == NULL){  // if p is still not found, search within an extending circle
        if (fabs((*p)[currLevel%N] - (*currPoint)[currLevel%N]) < maxRadius) {
            if (isLeftTree) nearestNeighborRecurse(currNode->right, key, p, condition);
            else nearestNeighborRecurse(currNode->left, key, p, condition);
        }
        maxRadius += 0.2;
    }
}

template <std::size_t N>
std::shared_ptr<Point<N>> KDTree<N>::kNNValue(const Point<N>& key, 
                            std::function<bool(const Common::SE2State* s1, 
                                            const Common::SE2State* s2)> condition) const {
    std::shared_ptr<Point<N>> p;
    if (empty()) return p;

    nearestNeighborRecurse(root_, key, p, condition);

    return p;
}
// class Point

template <std::size_t N>
std::size_t Point<N>::size() const {
    return N;
}

template <std::size_t N>
double& Point<N>::operator[] (std::size_t index) {
    return coords[index];
}

template <std::size_t N>
double Point<N>::operator[] (std::size_t index) const {
    return coords[index];
}

template <std::size_t N>
typename Point<N>::iterator Point<N>::begin() {
    return coords;
}

template <std::size_t N>
typename Point<N>::const_iterator Point<N>::begin() const {
    return coords;
}

template <std::size_t N>
typename Point<N>::iterator Point<N>::end() {
    return begin() + size();
}

template <std::size_t N>
typename Point<N>::const_iterator Point<N>::end() const {
    return begin() + size();
}

template <std::size_t N>
double Distance(const Point<N>& one, const Point<N>& two) {
    double result = 0.0;
    for (std::size_t i = 0; i < N; ++i)
        result += (one[i] - two[i]) * (one[i] - two[i]);
    return std::sqrt(result);
}

template <std::size_t N>
bool operator==(const Point<N>& one, const Point<N>& two) {
    for (const auto i = one.begin(), j = two.begin(); i != one.end(); ++i, ++j) {
        if (!(std::abs(*i - *j) < 1e-8)) {
            return false;
        }
    }
    return true;
}

template <std::size_t N>
bool operator!=(const Point<N>& one, const Point<N>& two) {
    return !(one == two);
}


/** BoundedPQueue class implementation details */

template <typename T>
BoundedPQueue<T>::BoundedPQueue(std::size_t maxSize) {
    maximumSize = maxSize;
}

template <typename T>
BoundedPQueue<T>::BoundedPQueue(const std::multimap<double, T>& t) : elems(t) {
    maximumSize = elems.size();
}

// enqueue adds the element to the map, then deletes the last element of the
// map if there size exceeds the maximum size.
template <typename T>
void BoundedPQueue<T>::enqueue(const T value, double priority) {
    // Add the element to the collection.
    elems.insert(std::make_pair(priority, value));

    // If there are too many elements in the queue, drop off the last one.
    if (size() > maxSize()) {
        typename std::multimap<double, T>::iterator last = elems.end();
        --last; // Now points to highest-priority element
        elems.erase(last);
    }
}

// dequeueMin copies the lowest element of the map (the one pointed at by
// begin()) and then removes it.
template <typename T>
T BoundedPQueue<T>::dequeueMin() {
    // Copy the best value.
    T result = elems.begin()->second;

    // Remove it from the map.
    elems.erase(elems.begin());
    
    return result;
}

// size() and empty() call directly down to the underlying map.
template <typename T>
std::size_t BoundedPQueue<T>::size() const {
    return elems.size();
}

template <typename T>
bool BoundedPQueue<T>::empty() const {
    return elems.empty();
}

// maxSize just returns the appropriate data member.
template <typename T>
std::size_t BoundedPQueue<T>::maxSize() const {
    return maximumSize;
}

// The best() and worst() functions check if the queue is empty,
// and if so return infinity.
template <typename T>
double BoundedPQueue<T>::best() const {
    return empty()? std::numeric_limits<double>::infinity() : elems.begin()->first;
}

template <typename T>
double BoundedPQueue<T>::worst() const {
    return empty()? std::numeric_limits<double>::infinity() : elems.rbegin()->first;
}

} // namespace HybridAStar
