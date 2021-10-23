#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    // int map_width = distances.widthInCells();
    // int map_height = distances.heightInCells();
    // Node start_node(start.x, start.y);
    // Node goal_node(goal.x, goal.y);
    // PriorityQueue open_list;
    // NodeList closed_list;
    // NodeList searched_list;
    
    // Node* current_node = open_list.pop();
    // while(!(*current_node==goal_node)){
    //     break;
    // }

    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;
}
