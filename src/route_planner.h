#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include "route_model.h"
#include <iostream>
#include <string>
#include <vector>

class RoutePlanner {
public:
  RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x,
               float end_y);
  // Add public variables or methods declarations here.
  float GetDistance() const { return route_distance; };
  void AStarSearch();
  std::vector<RouteModel::Node>
  ConstructFinalPath(const RouteModel::Node *current_node);
  float CalculateHValue(const RouteModel::Node *current_node);
  RouteModel::Node *NextNode();
  void AddNeighbors(RouteModel::Node *current_node);

private:
  // Add private variables or methods declarations here.
  RouteModel &m_Model;
  RouteModel::Node *start_node;
  RouteModel::Node *end_node;
  std::vector<RouteModel::Node *> open_list;
  float route_distance;
};

#endif