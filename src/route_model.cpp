#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  int counter = 0;
  for (const Model::Node &node : Nodes()) {
    m_Nodes.push_back(Node(counter++, this, node));
  }
  CreateNodeToRoadHashmap();
}

/*
Ways(): vector of way
Roads(): vector of road
road.way: index of road in Ways()
Ways()[road.way].nodes: vector of indices of nodes in the way
*/
void RouteModel::CreateNodeToRoadHashmap() {
  // iterate each road
  for (const Road &road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      // iterate each node in the road
      for (int node_idx : Ways()[road.way].nodes) {
        // node is not processed yet
        if (node_to_road.find(node_idx) == node_to_road.end()) {
          node_to_road[node_idx] = std::vector<const Model::Road *>();
        }
        node_to_road[node_idx].push_back(&road);
      }
    }
  }
}

// Find the closest unvisited node from a vector of node indices
RouteModel::Node *
RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
  Node *closest_node = nullptr;
  Node *other_node = nullptr;
  float dist_to_closest = std::numeric_limits<float>::max();
  for (int node_index : node_indices) {
    other_node = &parent_model->SNodes()[node_index];
    if (!other_node->visited) {
      float dist_to_other = RouteModel::Node::distance(*other_node);
      if (dist_to_other != 0.0 && dist_to_other < dist_to_closest) {
        closest_node = other_node;
        dist_to_closest = dist_to_other;
      }
    }
  }
  return closest_node;
};
void RouteModel::Node::FindNeighbors() {
  for (const Road *road : parent_model->node_to_road[index]) {
    const std::vector<int> &node_indices =
        parent_model->Ways()[road->way].nodes;
    RouteModel::Node *neighbor_node =
        RouteModel::Node::FindNeighbor(node_indices);
    if (neighbor_node) {
      neighbors.push_back(neighbor_node);
    }
  }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
  RouteModel::Node node_input;
  node_input.x = x;
  node_input.y = y;
  float min_dist = std::numeric_limits<float>::max();
  int closest_idx = -1;
  for (const Road &road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_idx : Ways()[road.way].nodes) {
        float current_dist = node_input.distance(SNodes()[node_idx]);
        if (current_dist < min_dist) {
          min_dist = current_dist;
          closest_idx = node_idx;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}