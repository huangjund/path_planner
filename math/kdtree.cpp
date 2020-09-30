#include "kdtree.h"

namespace HybridAStar {
template <std::size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree() :
    root_(NULL), size_(0) { }

template <std::size_t N, typename ElemType>
typename KDTree<N, ElemType>::Node* KDTree<N, ElemType>::deepcopyTree(typename KDTree<N, ElemType>::Node* root) {
    if (root == NULL) return NULL;
    Node* newRoot = new Node(*root);
    newRoot->left = deepcopyTree(root->left);
    newRoot->right = deepcopyTree(root->right);
    return newRoot;
}

template <std::size_t N, typename ElemType>
typename KDTree<N, ElemType>::Node* KDTree<N, ElemType>::buildTree(typename std::vector<std::pair<Point<N>, ElemType>>::iterator start,
                                                                   typename std::vector<std::pair<Point<N>, ElemType>>::iterator end, int currLevel) {
    if (start >= end) return NULL; // empty tree

    int axis = currLevel % N; // the axis to split on
    auto cmp = [axis](const std::pair<Point<N>, ElemType>& p1, const std::pair<Point<N>, ElemType>& p2) {
        return p1.first[axis] < p2.first[axis];
    };
    std::size_t len = end - start;
    auto mid = start + len / 2;
    std::nth_element(start, mid, end, cmp); // linear time partition

    // move left (if needed) so that all the equal points are to the right
    // The tree will still be balanced as long as there aren't many points that are equal along each axis
    while (mid > start && (mid - 1)->first[axis] == mid->first[axis]) {
        --mid;
    }

    Node* newNode = new Node(mid->first, currLevel, mid->second);
    newNode->left = buildTree(start, mid, currLevel + 1);
    newNode->right = buildTree(mid + 1, end, currLevel + 1);
    return newNode;
}

template <std::size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree(std::vector<std::pair<Point<N>, ElemType>>& points) {
    root_ = buildTree(points.begin(), points.end(), 0);
    size_ = points.size();
}

template <std::size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree(const KDTree& rhs) {
    root_ = deepcopyTree(rhs.root_);
    size_ = rhs.size_;
}

template <std::size_t N, typename ElemType>
KDTree<N, ElemType>& KDTree<N, ElemType>::operator=(const KDTree& rhs) {
    if (this != &rhs) { // make sure we don't self-assign
        freeResource(root_);
        root_ = deepcopyTree(rhs.root_);
        size_ = rhs.size_;
    }
    return *this;
}

template <std::size_t N, typename ElemType>
void KDTree<N, ElemType>::freeResource(typename KDTree<N, ElemType>::Node* currNode) {
    if (currNode == NULL) return;
    freeResource(currNode->left);
    freeResource(currNode->right);
    delete currNode;
}

template <std::size_t N, typename ElemType>
KDTree<N, ElemType>::~KDTree() {
    freeResource(root_);
}

template <std::size_t N, typename ElemType>
std::size_t KDTree<N, ElemType>::dimension() const {
    return N;
}

template <std::size_t N, typename ElemType>
std::size_t KDTree<N, ElemType>::size() const {
    return size_;
}

template <std::size_t N, typename ElemType>
bool KDTree<N, ElemType>::empty() const {
    return size_ == 0;
}

template <std::size_t N, typename ElemType>
typename KDTree<N, ElemType>::Node* KDTree<N, ElemType>::findNode(typename KDTree<N, ElemType>::Node* currNode, const Point<N>& pt) const {
    if (currNode == NULL || currNode->point == pt) return currNode;

    const Point<N>& currPoint = currNode->point;
    int currLevel = currNode->level;
    if (pt[currLevel%N] < currPoint[currLevel%N]) { // recurse to the left side
        return currNode->left == NULL ? currNode : findNode(currNode->left, pt);
    } else { // recurse to the right side
        return currNode->right == NULL ? currNode : findNode(currNode->right, pt);
    }
}

template <std::size_t N, typename ElemType>
bool KDTree<N, ElemType>::contains(const Point<N>& pt) const {
    auto node = findNode(root_, pt);
    return node != NULL && node->point == pt;
}

template <std::size_t N, typename ElemType>
void KDTree<N, ElemType>::insert(const Point<N>& pt, const ElemType& value) {
    auto targetNode = findNode(root_, pt);
    if (targetNode == NULL) { // this means the tree is empty
        root_ = new Node(pt, 0, value);
        size_ = 1;
    } else {
        if (targetNode->point == pt) { // pt is already in the tree, simply update its value
            targetNode->value = value;
        } else { // construct a new node and insert it to the right place (child of targetNode)
            int currLevel = targetNode->level;
            Node* newNode = new Node(pt, currLevel + 1, value);
            if (pt[currLevel%N] < targetNode->point[currLevel%N]) {
                targetNode->left = newNode;
            } else {
                targetNode->right = newNode;
            }
            ++size_;
        }
    }
}

template <std::size_t N, typename ElemType>
const ElemType& KDTree<N, ElemType>::at(const Point<N>& pt) const {
    auto node = findNode(root_, pt);
    if (node == NULL || node->point != pt) {
        throw std::out_of_range("Point not found in the KD-Tree");
    } else {
        return node->value;
    }
}

template <std::size_t N, typename ElemType>
ElemType& KDTree<N, ElemType>::at(const Point<N>& pt) {
    const KDTree<N, ElemType>& constThis = *this;
    return const_cast<ElemType&>(constThis.at(pt));
}

template <std::size_t N, typename ElemType>
ElemType& KDTree<N, ElemType>::operator[](const Point<N>& pt) {
    auto node = findNode(root_, pt);
    if (node != NULL && node->point == pt) { // pt is already in the tree
        return node->value;
    } else { // insert pt with default ElemType value, and return reference to the new ElemType
        insert(pt);
        if (node == NULL) return root_->value; // the new node is the root
        else return (node->left != NULL && node->left->point == pt) ? node->left->value: node->right->value;
    }
}

template <std::size_t N, typename ElemType>
void KDTree<N, ElemType>::nearestNeighborRecurse(const typename KDTree<N, ElemType>::Node* currNode,
                                                const Point<N>& key, 
                                                BoundedPQueue<ElemType>& pQueue) const {
    if (currNode == NULL) return;
    const Point<N>& currPoint = currNode->point;

    // Add the current point to the BPQ if it is closer to 'key' that some point in the BPQ
    // @param: value, priority
    pQueue.enqueue(currNode->value, Distance(currPoint, key));

    // Recursively search the half of the tree that contains Point 'key'
    int currLevel = currNode->level;
    bool isLeftTree;
    if (key[currLevel%N] < currPoint[currLevel%N]) {
        nearestNeighborRecurse(currNode->left, key, pQueue);
        isLeftTree = true;
    } else {
        nearestNeighborRecurse(currNode->right, key, pQueue);
        isLeftTree = false;
    }

    if (pQueue.size() < pQueue.maxSize() || fabs(key[currLevel%N] - currPoint[currLevel%N]) < pQueue.worst()) {
        // Recursively search the other half of the tree if necessary
        if (isLeftTree) nearestNeighborRecurse(currNode->right, key, pQueue);
        else nearestNeighborRecurse(currNode->left, key, pQueue);
    }
}

template <std::size_t N, typename ElemType>
BoundedPQueue<ElemType> KDTree<N, ElemType>::kNNValue(const Point<N>& key, std::size_t k) const {
    BoundedPQueue<ElemType> pQueue(k); // BPQ with maximum size k
    if (empty()) return BoundedPQueue<ElemType>(0); // default return value if KD-tree is empty

    // Recursively search the KD-tree with pruning
    nearestNeighborRecurse(root_, key, pQueue);

    return pQueue;
}

template <std::size_t N, typename ElemType>
void KDTree<N, ElemType>::nearestNeighborRecurse(const Node* currNode, 
                                                const Point<N>& key, 
                                                double radius,
                                                std::unordered_map<double, ElemType>& pBucket) const {
  if (currNode == NULL) return;
  const Point<N>& currPoint = currNode->point;

  auto dis = Distance(currPoint, key);
  if (dis < radius) 
    pBucket[dis] = currNode->value;

  int currLevel = currNode->level;
  bool isLeftTree;
  if (key[currLevel%N] < currPoint[currLevel%N]) {
    nearestNeighborRecurse(currNode->left, key, radius, pBucket);
    isLeftTree = true;
  } else {
    nearestNeighborRecurse(currNode->right, key, radius, pBucket);
    isLeftTree = false;
  }

  if (fabs(key[currLevel%N] - currPoint[currLevel%N]) < radius) {
    if (isLeftTree) nearestNeighborRecurse(currNode->right, key, radius, pBucket);
    else nearestNeighborRecurse(currNode->left, key, radius, pBucket);
  }
}

template <std::size_t N, typename ElemType>
BoundedPQueue<ElemType> KDTree<N, ElemType>::kNNValue(const Point<N>& key, double radius) const {
  std::unordered_map<double, ElemType> bucket;   // create an unordered multimap as a temp container
  if (empty()) return BoundedPQueue<ElemType>(0);


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
    return std::equal(one.begin(), one.end(), two.begin());
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

// enqueue adds the element to the map, then deletes the last element of the
// map if there size exceeds the maximum size.
template <typename T>
void BoundedPQueue<T>::enqueue(const T& value, double priority) {
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
