#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include "model.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>

class RouteModel : public Model {

public:
  class Node : public Model::Node {
  public:
    // Add public Node variables and methods here.

    Node() {}
    Node(int idx, RouteModel *search_model, Model::Node node)
        : Model::Node(node), parent_model(search_model), index(idx) {}

    Node *parent = nullptr;
    float h_value = std::numeric_limits<float>::max();
    float g_value = std::numeric_limits<float>::max();
    bool visited = false;
    bool explored = false;
    std::vector<Node *> neighbors;
    float distance(const Node &other_node) const {
      return std::sqrt((other_node.x - x) * (other_node.x - x) +
                       (other_node.y - y) * (other_node.y - y));
    };
    void FindNeighbors();

  private:
    // Add private Node variables and methods here.
    int index;
    RouteModel *parent_model = nullptr;
    RouteModel::Node *FindNeighbor(std::vector<int> node_indices);
  };

  // Add public RouteModel variables and methods here.
  RouteModel(const std::vector<std::byte> &xml);
  std::vector<Node> path; // This variable will eventually store the path that
                          // is found by the A* search.
  std::vector<Node> &SNodes() { return m_Nodes; }
  const auto &GetNodeToRoadMap() const { return node_to_road; }
  Node &FindClosestNode(float x, float y);

private:
  // Add private RouteModel variables and methods here.
  std::vector<Node> m_Nodes;
  std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
  void CreateNodeToRoadHashmap();
};

#endif
