#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto *node : current_node->neighbors)
    {
        node->parent = current_node;
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->h_value = this->CalculateHValue(node);
        node->visited = true;
        this->open_list.push_back(node);
    }
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    std::cout << "Path Found Size :" << path_found.size() << std::endl;
    while (current_node->parent != nullptr)
    {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    path_found.push_back(*this->start_node);
    distance *= m_Model.MetricScale();
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}

bool Compare(RouteModel::Node const *node_1, RouteModel::Node const *node_2)
{
    return (node_1->g_value + node_1->h_value) > (node_2->g_value + node_2->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), Compare);
    RouteModel::Node *node = this->open_list.back();
    this->open_list.pop_back();
    return node;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    this->start_node->visited = true;
    this->open_list.push_back(start_node);
    while(this->open_list.size()>0){
        current_node = this->NextNode();
        if (current_node->distance(*this->end_node) == 0)
        {
            this->m_Model.path = this->ConstructFinalPath(current_node);
            break;
        }
        else {
            this->AddNeighbors(current_node);
        }
    }
    std::cout << start_node->x << std::endl;
}