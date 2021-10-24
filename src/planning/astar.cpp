#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <algorithm>

double h_cost(Node* from, Node* goal) {
    int dx = std::abs(goal->cell.x - from->cell.x);
    int dy = std::abs(goal->cell.y - from->cell.y);
    double cost = 0;
    if (dx >= dy) cost = std::sqrt(2) * dy + (dx - dy);
    else cost = std::sqrt(2) * dx + (dy - dx);
    return cost;
}

double g_cost(Node* to, Node* parent){
    int dx = std::abs(to->cell.x - parent->cell.x);
    int dy = std::abs(to->cell.y - parent->cell.y);
    if(dx == 1 && dy == 1) return parent->g_cost + std::sqrt(2);
    else return parent->g_cost + 1;
}


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    int map_width = distances.widthInCells();
    int map_height = distances.heightInCells();
    Node start_node(start.x, start.y);
    Node goal_node(goal.x, goal.y);
    PriorityQueue open_list;
    NodeList closed_list;
    NodeList searched_list;
    
    Node* current_node = open_list.pop();
    while(!(*current_node==goal_node)){
        closed_list.put(current_node);
        expand_node(current_node, distances, params, closed_list, searched_list, open_list, goal_node);
        current_node = open_list.pop();
    }

    robot_path_t path;
    path.utime = start.utime;
    extract_pose_path(current_node, path, start_node, path.utime);
    //path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;
}

void expand_node(Node* node, ObstacleDistanceGrid& distances, const SearchParams& params, NodeList& closed_list, NodeList& searched_list, PriorityQueue open_list, Node& goal_node){
    const int xDeltas[8] = {1, 1, 0, 1, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, 0, -1};
    for(int i=0; i<8; i++){
        cell_t cell(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
        Node* neighbor;
        if(searched_list.is_member(cell))
            neighbor = searched_list.get(cell);
        else
            neighbor = new Node(cell.x, cell.y);

        if (!closed_list.is_member(neighbor->cell) && neighbor->is_in_map(distances) && !neighbor->is_obstacle(distances)) {
            if (!searched_list.is_member(neighbor->cell)) {
                neighbor->g_cost = g_cost(neighbor, node);
                neighbor->h_cost = h_cost(neighbor, &goal_node);
                neighbor->parent = node;
                open_list.push(neighbor);
                searched_list.put(neighbor);
            }
            else if (neighbor->g_cost > g_cost(node, neighbor)) {
                neighbor->g_cost = g_cost(node, neighbor);
                neighbor->parent = node;
                open_list.push(neighbor);
            }
        }
        

    }
}

void extract_node_path(Node* node, robot_path_t& path, Node& start_node, int64_t utime){
    Node* current_node = node;
    while(!(*current_node == start_node)){
        pose_xyt_t pose;
        pose.utime = utime;
        pose.x = current_node->cell.x;
        pose.y = current_node->cell.y;
        pose.theta = 0.0;
        path.path.push_back(pose);
        current_node = current_node->parent;
    }
    std::reverse(path.path.begin(), path.path.end());
}