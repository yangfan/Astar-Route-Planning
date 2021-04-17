#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;
  start_node = &(m_Model.FindClosestNode(start_x, start_y));
  start_node->g_value = 0;
  end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(const RouteModel::Node *current_node) {
  std::vector<RouteModel::Node> path_found;
  route_distance = 0.0f;
  while (current_node->parent) {
    path_found.emplace_back(*current_node);
    route_distance += current_node->distance(*(current_node->parent));
    current_node = current_node->parent;
  }
  path_found.emplace_back(*current_node); // include the starting node
  route_distance *= m_Model.MetricScale();
  std::reverse(path_found.begin(), path_found.end());
  return path_found;
}

void RoutePlanner::AStarSearch() {
  start_node->visited = true;
  open_list.emplace_back(start_node);
  RouteModel::Node *current_node = nullptr;
  while (open_list.size()) {
    current_node = NextNode();
    if (current_node->distance(*end_node) == 0) {
      m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
      return;
    } else {
      AddNeighbors(current_node);
    }
  }
  std::cout << "No path is found!\n";
}

float RoutePlanner::CalculateHValue(const RouteModel::Node *current_node) {
  return current_node->distance(*end_node);
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(),
            [](const auto &a, const auto &b) -> bool {
              return a->g_value + a->h_value < b->g_value + b->h_value;
            });
  RouteModel::Node *node_lowest = open_list.front();
  node_lowest->visited = true;
  open_list.erase(open_list.begin());
  return node_lowest;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (RouteModel::Node *neighbor : current_node->neighbors) {
    float neighbor_g =
        current_node->g_value + current_node->distance(*neighbor);
    if (neighbor_g < (neighbor->g_value)) {
      neighbor->parent = current_node;
      neighbor->g_value = neighbor_g;
    }
    if (!neighbor->explored) {
      neighbor->explored = true;
      open_list.emplace_back(neighbor);
      neighbor->h_value = CalculateHValue(neighbor);
    }
  }
}