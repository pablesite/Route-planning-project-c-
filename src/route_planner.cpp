#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.  
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y); 
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors(); //FindNeighbords()
    /*I'm not sure about this structure: "Data Type* &instance. 
    Am I creating references on pointers? Makes sense? */
    for(RouteModel::Node* &neighbord : current_node->neighbors){                            //For each node in current_node.neighbors
        neighbord->parent = current_node;                                                   //Set the parent
        neighbord->h_value = CalculateHValue(neighbord);                                    //Set h value (CalculateHValue)
        neighbord->g_value = current_node->g_value + current_node->distance(*neighbord);    //Set g value
        neighbord->visited = true;                                                          //Set the node's visited attribute to true
        open_list.emplace_back(neighbord);                                                  //Add neighbord to open_list
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

/*According to the definition of sort function, it should retur true when f_first is BIGGER than f_second. 
Thus, the shortest distance will be placed in the last posititon of the vector*/
bool comp(RouteModel::Node *first, RouteModel::Node *second){
    float f_first = first->g_value + first->h_value;
    float f_second = second->g_value + second->h_value;
    return f_first>f_second; 
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), comp);    //Sort the open_list according to the sum of the h value and g value.
    RouteModel::Node* next_node = open_list.back();         //Create a pointer to the node in the list with the lowest sum
    open_list.pop_back();                                   //Remove that node from the open_list.
    return next_node;                                       //Return the pointer.
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

    // TODO: Implement your solution here.
    while(current_node->parent != nullptr) {                        //Iteratively follow the chain of parents of nodes until the starting node is found
        path_found.emplace_back(*current_node);                     
        distance += current_node->distance(*current_node->parent);  //Adding the distance from the node to its parent to the distance variable 
        current_node = current_node->parent;                        //Moving to the next node in the chain
    } 
    path_found.emplace_back(*current_node);                         //This will be the first node.

    std::reverse(path_found.begin(), path_found.end());             //The start node should be the first element of the vector, the end node should be the last element

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    
    // Initialize start_node.
    start_node->g_value = 0;
    start_node->h_value = CalculateHValue(start_node);
    start_node->visited = true;
    open_list.emplace_back(start_node);
    
    while(open_list.size()>0){
        current_node = NextNode();                          //Sort the open_list and return the next node

        if(current_node == end_node){                       //When the search has reached the end_node return the final path that was found
            m_Model.path = ConstructFinalPath(end_node); 
            break;
        }

        AddNeighbors(current_node);                         //Add all of the neighbors of the current node (start_node)) to the open_list
    }

}