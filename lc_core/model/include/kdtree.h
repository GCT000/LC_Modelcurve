/**
 * @file    kdtree.h
 * @brief   kd-tree implementation
 * @author  Yipeng Zhao
 * @date    2024/09
 * @ref     https://github.com/gaoxiang12/slam_in_autonomous_driving/blob/master/src/ch5/kdtree.cc
 */

#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <queue>
#include <opencv2/core/types.hpp>

namespace lc_core
{

typedef std::vector<int> IndexVec;
typedef cv::Point2d PointType;
typedef std::vector<PointType> PointVec;

/// @brief  KDTree struct
struct KDTreeNode {
    int id_ = -1;
    int point_idx_ = 0;
    int axis_index_ = 0;
    float split_value_ = 0.0f;
    KDTreeNode* left_ = nullptr;
    KDTreeNode* right_ = nullptr;

    bool isLeaf() const {
        return left_ == nullptr && right_ == nullptr;
    }
};

/// @brief  Node and distance, used for knn search
struct NodeAndDistance {
    NodeAndDistance(KDTreeNode* node, float dis2) : node_(node), distance2_(dis2) {}
    KDTreeNode* node_ = nullptr;
    float distance2_ = 0;  // squared distance, used for comparison

    bool operator<(const NodeAndDistance& other) const { return distance2_ < other.distance2_; }
};

class KdTree {
public:
    /// @brief  Constructor
    explicit KdTree() = default;
    
    /// @brief  Destructor
    ~KdTree() {
        clear();
    }

    /// @brief  Clear the tree
    void clear();

    /// @brief  Build the tree
    bool build(const std::vector<PointType>& points);

    /// @brief  Get the closest point
    bool getClosestPoint(const PointType& target, std::vector<int>& closest_idx, int k = 5);
    
    /// @brief  Get the closest points using multi-threading
    bool getClosestPointMT(const PointVec& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);

    /// @brief  Return nodes' size
    size_t size() const {
        return size_;
    }

private:
    /// @brief  Insert points into the tree
    void insert(const IndexVec& points, KDTreeNode* node);

    /// @brief  Calculate the split axis and threshold
    bool findSplitAxisAndThreshold(const IndexVec& points, int& axis, float& threshold, IndexVec& left, IndexVec& right);

    /// @brief  Reset
    void reset();

    /// @brief  Calculate the distance between two points
    static inline double calculateDistance(const PointType& p1, const PointType& p2) {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /// @brief  Knn search
    void knnSearch(const PointType& target, KDTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /// @brief  Compute distance for leaf node
    void computeDisForLeaf(const PointType& pt, KDTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /// @brief  Whether the node needs expand
    bool needExpand(const PointType& pt, KDTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const;

private:
    int k_ = 5;                                  
    std::shared_ptr<KDTreeNode> root_ = nullptr;
    std::vector<PointType> cloud_;                // input points
    std::unordered_map<int, KDTreeNode*> nodes_;  // for bookkeeping

    size_t size_ = 0;       // leaf nodes
    int tree_node_id_ = 0;  // assign id for kdtree node
};

} // namespace lc_core
#endif