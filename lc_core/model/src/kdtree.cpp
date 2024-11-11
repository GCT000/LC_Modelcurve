#include "kdtree.h"
#include <execution>
#include <glog/logging.h>

using namespace lc_core;

bool KdTree::build(const std::vector<PointType>& points) {
    if (points.empty()) {
        return false;
    }

    cloud_.clear();
    cloud_.reserve(points.size());
    for (const auto& point : points) {
        cloud_.push_back(point);
    }

    // Clear the tree
    clear();
    // Reset the tree
    reset();

    // Generate indices
    IndexVec indices(points.size());
    std::generate(indices.begin(), indices.end(), [n = 0]() mutable { return n++; });
    // Insert indices into the tree
    insert(indices, root_.get());

    return true;
}

void KdTree::insert(const IndexVec& points, KDTreeNode* node) {
    nodes_.insert(std::make_pair(node->id_, node));

    if (points.empty()) {
        return;
    }

    if (points.size() == 1) {
        size_++;
        node->point_idx_ = points[0];
        return;
    }

    IndexVec left, right;
    // Find the split axis and threshold
    if (!findSplitAxisAndThreshold(points, node->axis_index_, node->split_value_, left, right)) {
        ++size_;
        node->point_idx_ = points[0];
        return;
    }

    // Create left and right child
    const auto create_if_not_empty = [&node, this](KDTreeNode *&new_node, const IndexVec &index) {
        if (!index.empty()) {
            new_node = new KDTreeNode;
            new_node->id_ = tree_node_id_++;
            insert(index, new_node);
        }
    };

    create_if_not_empty(node->left_, left);
    create_if_not_empty(node->right_, right);
}

bool KdTree::getClosestPoint(const PointType& target, std::vector<int>& closest_idx, int k) {
    if (k > size_) {
        LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
        return false;
    }
    k_ = k;

    std::priority_queue<NodeAndDistance> knn_result;
    knnSearch(target, root_.get(), knn_result);

    // Sort and return results
    closest_idx.resize(knn_result.size());
    for (int i = closest_idx.size() - 1; i >= 0; --i) {
        // Insert in reverse order
        closest_idx[i] = knn_result.top().node_->point_idx_;
        knn_result.pop();
    }
    return true;
}

// TODO: The points' number is not large, so the need for parallelization is not obvious
[[maybe_unused]]bool KdTree::getClosestPointMT(const PointVec &cloud, std::vector<std::pair<size_t, size_t>> &matches, int k) {
    matches.resize(cloud.size() * k);

    // index
    std::vector<int> index(cloud.size());
    for (int i = 0; i < cloud.size(); ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &cloud, &matches, &k](int idx) {
        std::vector<int> closest_idx;
        getClosestPoint(cloud[idx], closest_idx, k);
        for (int i = 0; i < k; ++i) {
            matches[idx * k + i].second = idx;
            if (i < closest_idx.size()) {
                matches[idx * k + i].first = closest_idx[i];
            } else {
                matches[idx * k + i].first = -1;
            }
        }
    });

    return true;
}

void KdTree::knnSearch(const PointType &pt, KDTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const {
    if (node->isLeaf()) {
        // If it is a leaf node, check if it can be inserted
        computeDisForLeaf(pt, node, knn_result);
        return;
    }

    // Check which side (left or right) pt falls on, and prioritize searching that subtree
    // Then check if the other subtree needs to be searched
    KDTreeNode *this_side, *that_side;
    double value = node->axis_index_ == 0 ? pt.x : pt.y;
    if (value < node->split_value_) {
        this_side = node->left_;
        that_side = node->right_;
    } else {
        this_side = node->right_;
        that_side = node->left_;
    }

    knnSearch(pt, this_side, knn_result);
    if (needExpand(pt, node, knn_result)) {
        knnSearch(pt, that_side, knn_result);
    }
}

bool KdTree::needExpand(const PointType &pt, KDTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const {
    if (knn_result.size() < k_) {
        return true;
    }

    double value = node->axis_index_ == 0 ? pt.x : pt.y;
    float d = value - node->split_value_;
    if ((d * d) < knn_result.top().distance2_) {
        return true;
    } else {
        return false;
    }
}

void KdTree::computeDisForLeaf(const PointType &pt, KDTreeNode *node,
                               std::priority_queue<NodeAndDistance> &knn_result) const {
    // Compare with the result queue, insert if better than the farthest distance
    float dis2 = calculateDistance(pt, cloud_[node->point_idx_]);
    if (knn_result.size() < k_) {
        // results less than k
        knn_result.emplace(node, dis2);
    } else {
        // results equal to k, compare current with max_dis_iter
        if (dis2 < knn_result.top().distance2_) {
            knn_result.emplace(node, dis2);
            knn_result.pop();
        }
    }
}

bool KdTree::findSplitAxisAndThreshold(const IndexVec &point_idx, int &axis, float &th, IndexVec &left, IndexVec &right) {
    // calculate variance
    double var_x = 0, var_y = 0;
    double mean_x = 0, mean_y = 0;
    
    // calculate mean
    for (const auto& idx : point_idx) {
        mean_x += cloud_[idx].x;
        mean_y += cloud_[idx].y;
    }
    mean_x /= point_idx.size();
    mean_y /= point_idx.size();
    
    // calculate variance
    for (const auto& idx : point_idx) {
        var_x += (cloud_[idx].x - mean_x) * (cloud_[idx].x - mean_x);
        var_y += (cloud_[idx].y - mean_y) * (cloud_[idx].y - mean_y);
    }
    var_x /= point_idx.size();
    var_y /= point_idx.size();
    
    // choose the axis with larger variance
    axis = (var_x > var_y) ? 0 : 1;
    
    // calculate threshold (using median)
    std::vector<double> values;
    values.reserve(point_idx.size());
    for (const auto& idx : point_idx) {
        values.push_back(axis == 0 ? cloud_[idx].x : cloud_[idx].y);
    }
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    th = values[values.size() / 2];
    
    // split points into left and right
    for (const auto& idx : point_idx) {
        if ((axis == 0 ? cloud_[idx].x : cloud_[idx].y) < th) {
            left.push_back(idx);
        } else {
            right.push_back(idx);
        }
    }

    if (point_idx.size() > 1 && (left.empty() || right.empty())) {
        return false;
    }

    return true;
}

void KdTree::reset() {
    tree_node_id_ = 0;
    root_.reset(new KDTreeNode());
    root_->id_ = tree_node_id_++;
    size_ = 0;
}

void KdTree::clear() {
    for (const auto &np : nodes_) {
        if (np.second != root_.get()) {
            delete np.second;
        }
    }

    nodes_.clear();
    root_ = nullptr;
    size_ = 0;
    tree_node_id_ = 0;
}