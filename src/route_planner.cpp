#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Finds and stores the closest nodes to the starting and ending coordinates 
    // into RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Calculates the heuristic value.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Expands the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node *neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}

// Sorts open _list by the sum of the g and h values and returns a pointer to the node
// with the current shortest estimated distance (lowest sum) while also removing this
// node from the open_list.
RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), [](RouteModel::Node *&node1, RouteModel::Node *&node2)
    {
        return (node1->h_value + node1->g_value) > (node2->h_value + node2->g_value);
    });
    RouteModel::Node *lowest_sum_node = open_list.back();
    open_list.pop_back();
    return lowest_sum_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Iteratively calculates distance and path by following the chain of parent nodes.
    while(current_node->parent != nullptr) {
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.emplace_back(*start_node); // Adds the start node to the final path.

    // Reverses the path_found vector so the beginning to the end of the vector
    // represents the start to the end of the path.
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm
void RoutePlanner::AStarSearch() {
    // Initialize start_node to be added to open_list
    RouteModel::Node *current_node = start_node;
    current_node->g_value = 0;
    current_node->h_value = CalculateHValue(current_node);
    current_node->visited = true;
    open_list.emplace_back(start_node);

    // Continues while path is not found or it has explored all accessible nodes
    while(!open_list.empty()) {
        current_node = NextNode(); // Sorts the open_list and returns the next closest node

        // The end_node is reached with the shortest path and stored in m_Model.path . This path is then shown on the map.
        if(current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // If the end_node is not yet reached
        AddNeighbors(current_node); // Add unvisited neighbor nodes of the current_node to open_list.
    }
}