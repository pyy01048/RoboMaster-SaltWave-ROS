#include "sensor_fusion/graph_optimizer.hpp"
#include <cmath>

namespace sensor_fusion {

GraphOptimizer::GraphOptimizer()
    : max_iterations_(20)
    , convergence_threshold_(1e-6)
    , loop_closure_enabled_(true) {
}

void GraphOptimizer::setMaxIterations(int iter) {
    max_iterations_ = std::max(1, iter);
}

void GraphOptimizer::setConvergenceThreshold(double thresh) {
    convergence_threshold_ = std::max(1e-10, thresh);
}

void GraphOptimizer::enableLoopClosure(bool enable) {
    loop_closure_enabled_ = enable;
}

void GraphOptimizer::addNode(const GraphNode& node) {
    std::lock_guard<std::mutex> lock(mutex_);
    nodes_[node.id] = node;
}

void GraphOptimizer::addEdge(const GraphEdge& edge) {
    std::lock_guard<std::mutex> lock(mutex_);
    edges_.push_back(edge);
}

Eigen::Matrix<double, 6, 1> GraphOptimizer::computeResidual(
    const GraphEdge& edge, 
    const GraphNode& from, 
    const GraphNode& to) {
    
    Eigen::Matrix<double, 6, 1> residual;
    
    Eigen::Matrix3d R_from = from.orientation.toRotationMatrix();
    Eigen::Vector3d predicted_position = from.position + R_from * edge.relative_position;
    
    residual.head<3>() = to.position - predicted_position;
    
    Eigen::Quaterniond q_predicted = from.orientation * edge.relative_orientation;
    Eigen::Quaterniond q_error = to.orientation * q_predicted.inverse();
    
    if (q_error.w() < 0) {
        q_error = Eigen::Quaterniond(-q_error.w(), -q_error.x(), -q_error.y(), -q_error.z());
    }
    
    residual.tail<3>() = 2.0 * q_error.vec();
    
    return residual;
}

Eigen::Matrix<double, 6, 6> GraphOptimizer::computeJacobian(
    const GraphEdge& edge, 
    const GraphNode& from, 
    const GraphNode& to) {
    
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
    
    Eigen::Matrix3d R_from = from.orientation.toRotationMatrix();
    J.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    J.block<3, 3>(0, 3) = -R_from * Eigen::Matrix3d::Identity();
    
    return J;
}

bool GraphOptimizer::detectLoopClosure(const GraphNode& new_node) {
    if (!loop_closure_enabled_ || nodes_.size() < 10) {
        return false;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    int closest_id = -1;
    
    for (const auto& [id, node] : nodes_) {
        if (id == new_node.id) continue;
        
        double dist = (node.position - new_node.position).norm();
        double time_diff = std::abs(node.timestamp - new_node.timestamp);
        
        if (dist < 1.0 && time_diff > 30.0 && dist < min_distance) {
            min_distance = dist;
            closest_id = id;
        }
    }
    
    return closest_id >= 0;
}

void GraphOptimizer::addLoopClosureConstraint(int from_id, int to_id) {
    if (nodes_.find(from_id) == nodes_.end() || nodes_.find(to_id) == nodes_.end()) {
        return;
    }
    
    const auto& from = nodes_[from_id];
    const auto& to = nodes_[to_id];
    
    GraphEdge loop_edge;
    loop_edge.from_id = from_id;
    loop_edge.to_id = to_id;
    loop_edge.relative_position = from.orientation.inverse() * (to.position - from.position);
    loop_edge.relative_orientation = from.orientation.inverse() * to.orientation;
    loop_edge.information_matrix = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
    
    edges_.push_back(loop_edge);
}

double GraphOptimizer::computeTotalError() {
    double total_error = 0.0;
    
    for (const auto& edge : edges_) {
        if (nodes_.find(edge.from_id) == nodes_.end() || 
            nodes_.find(edge.to_id) == nodes_.end()) {
            continue;
        }
        
        auto residual = computeResidual(edge, nodes_[edge.from_id], nodes_[edge.to_id]);
        total_error += residual.transpose() * edge.information_matrix * residual;
    }
    
    return total_error;
}

void GraphOptimizer::optimize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (nodes_.empty() || edges_.empty()) {
        return;
    }
    
    double prev_error = computeTotalError();
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        Eigen::MatrixXd H_total = Eigen::MatrixXd::Zero(nodes_.size() * 6, nodes_.size() * 6);
        Eigen::VectorXd b_total = Eigen::VectorXd::Zero(nodes_.size() * 6);
        
        for (const auto& edge : edges_) {
            if (nodes_.find(edge.from_id) == nodes_.end() || 
                nodes_.find(edge.to_id) == nodes_.end()) {
                continue;
            }
            
            auto& from = nodes_[edge.from_id];
            auto& to = nodes_[edge.to_id];
            
            auto residual = computeResidual(edge, from, to);
            auto J = computeJacobian(edge, from, to);
            
            Eigen::Matrix<double, 6, 6> H = J.transpose() * edge.information_matrix * J;
            Eigen::Matrix<double, 6, 1> b = J.transpose() * edge.information_matrix * residual;
            
            H_total.block<6, 6>(from.id * 6, from.id * 6) += H;
            b_total.segment<6>(from.id * 6) += b;
        }
        
        H_total.diagonal().array() += 1e-6;
        
        Eigen::VectorXd dx = H_total.ldlt().solve(b_total);
        
        for (auto& [id, node] : nodes_) {
            Eigen::Matrix<double, 6, 1> delta = dx.segment<6>(id * 6);
            node.position += delta.head<3>();
            
            Eigen::Quaterniond dq;
            dq.w() = 1.0;
            dq.vec() = 0.5 * delta.tail<3>();
            dq.normalize();
            node.orientation = node.orientation * dq;
            node.orientation.normalize();
        }
        
        double current_error = computeTotalError();
        if (std::abs(prev_error - current_error) < convergence_threshold_) {
            break;
        }
        prev_error = current_error;
    }
}

GraphNode GraphOptimizer::getOptimizedPose(int node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = nodes_.find(node_id);
    if (it != nodes_.end()) {
        return it->second;
    }
    
    return GraphNode();
}

std::vector<GraphNode> GraphOptimizer::getAllOptimizedPoses() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<GraphNode> poses;
    poses.reserve(nodes_.size());
    
    for (const auto& [id, node] : nodes_) {
        poses.push_back(node);
    }
    
    return poses;
}

}
