#pragma once

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <mutex>

namespace sensor_fusion {

struct GraphNode {
    int id;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double timestamp;
};

struct GraphEdge {
    int from_id;
    int to_id;
    Eigen::Vector3d relative_position;
    Eigen::Quaterniond relative_orientation;
    Eigen::Matrix<double, 6, 6> information_matrix;
};

class GraphOptimizer {
public:
    GraphOptimizer();
    ~GraphOptimizer() = default;

    void addNode(const GraphNode& node);
    void addEdge(const GraphEdge& edge);
    void optimize();
    
    GraphNode getOptimizedPose(int node_id) const;
    std::vector<GraphNode> getAllOptimizedPoses() const;
    
    void setMaxIterations(int iter);
    void setConvergenceThreshold(double thresh);
    void enableLoopClosure(bool enable);

private:
    std::map<int, GraphNode> nodes_;
    std::vector<GraphEdge> edges_;
    
    int max_iterations_;
    double convergence_threshold_;
    bool loop_closure_enabled_;
    
    mutable std::mutex mutex_;
    
    Eigen::Matrix<double, 6, 6> computeJacobian(const GraphEdge& edge, const GraphNode& from, const GraphNode& to);
    Eigen::Matrix<double, 6, 1> computeResidual(const GraphEdge& edge, const GraphNode& from, const GraphNode& to);
    
    bool detectLoopClosure(const GraphNode& new_node);
    void addLoopClosureConstraint(int from_id, int to_id);
    
    double computeTotalError();
};

}
