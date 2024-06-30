// #include <pluginlib/class_list_macros.h>
// #include "global_planner.h"

// PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

// namespace global_planner {

// GlobalPlanner::GlobalPlanner (){

// }

// GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
//     initialize(name, costmap_ros);
// }

// void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

// }

// bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

//         plan.push_back(start);
//         plan.push_back(goal);

//         return true;
//     }
// };

// #include <pluginlib/class_list_macros.h>
// #include "global_planner.h"

// PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

// namespace global_planner
// {

//     GlobalPlanner::GlobalPlanner() : costmap_ros_(nullptr), costmap_(nullptr), initialized_(false) {}

//     GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(nullptr), costmap_(nullptr), initialized_(false)
//     {
//         initialize(name, costmap_ros);
//     }

//     void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
//     {
//         if (!initialized_)
//         {
//             costmap_ros_ = costmap_ros;
//             costmap_ = costmap_ros_->getCostmap();
//             initialized_ = true;
//         }
//     }

//     bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
//                                  std::vector<geometry_msgs::PoseStamped> &plan)
//     {
//         if (!initialized_)
//         {
//             ROS_ERROR("GlobalPlanner has not been initialized, please call initialize() before using this planner");
//             return false;
//         }

//         plan.clear(); // Clear the existing plan

//         unsigned int start_x, start_y, goal_x, goal_y;
//         if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
//             !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
//         {
//             ROS_ERROR("The start or goal is out of the map bounds");
//             return false;
//         }

//         struct Node
//         {
//             unsigned int x, y;
//             double g, h;
//             Node *parent;
//             double f() const { return g + h; }
//         };

//         auto manhattan_heuristic = [](unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) -> double
//         {
//             return static_cast<double>(std::abs(static_cast<int>(x1) - static_cast<int>(x2)) + std::abs(static_cast<int>(y1) - static_cast<int>(y2)));
//         };

//         struct NodeComparator
//         {
//             bool operator()(const Node *lhs, const Node *rhs) const
//             {
//                 return lhs->f() > rhs->f();
//             }
//         };

//         std::priority_queue<Node *, std::vector<Node *>, NodeComparator> open_list;
//         // priority_queue open_list()

//         std::vector<std::vector<bool>> closed_list(costmap_->getSizeInCellsX(), std::vector<bool>(costmap_->getSizeInCellsY(), false)); // visited nodes list

//         Node *start_node = new Node{start_x, start_y, 0.0, manhattan_heuristic(start_x, start_y, goal_x, goal_y), nullptr}; // set values for start_node

//         open_list.push(start_node);

//         // const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
//         // const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

//         const int dx[4] = {1, -1, 0, 0};
//         const int dy[4] = {0, 0, 1, -1};

//         int count = 0; // Test1

//         while (!open_list.empty())
//         {

//             Node *current = open_list.top();
//             open_list.pop();

//             count++; // Test1
//             // ROS_INFO(" count of loop is (%d)",count);
//             ROS_INFO(" start x:(%d) , y: (%d)", start_x, start_y);
//             ROS_INFO(" goals x:(%d) , y: (%d)", goal_x, goal_y);

//             if (current->x == goal_x && current->y == goal_y)
//             { // If we reached the goal, trace back parents to make a path plan
//                 Node *path_node = current;
//                 while (path_node != nullptr)
//                 {
//                     geometry_msgs::PoseStamped pose;
//                     double wx, wy;
//                     costmap_->mapToWorld(path_node->x, path_node->y, wx, wy);
//                     pose.pose.position.x = wx;
//                     pose.pose.position.y = wy;
//                     pose.pose.orientation = start.pose.orientation;
//                     plan.push_back(pose);
//                     path_node = path_node->parent;
//                 }

//                 std::reverse(plan.begin(), plan.end()); // reverse it

//                 // Clean up allocated nodes
//                 while (!open_list.empty())
//                 {
//                     delete open_list.top();
//                     open_list.pop();
//                 }

//                 delete current;
//                 return true;
//             }
//             // else

//             closed_list[current->x][current->y] = true; // mark as visited
//             ROS_INFO("coordinates are (%d) and  (%d) and f:(%lf)", current->x, current->y, current->f());

//             for (int i = 0; i < 4; ++i)
//             {

//                 unsigned int nx = current->x + dx[i];
//                 unsigned int ny = current->y + dy[i];

//                 if (nx >= costmap_->getSizeInCellsX() || ny >= costmap_->getSizeInCellsY() || closed_list[nx][ny])
//                 {
//                     continue; // if point is visited or out of map
//                 }

//                 unsigned char cost = costmap_->getCost(nx, ny);
//                 if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost >= 253)
//                 {
//                     continue; // if there is a obstacle in nbd
//                 }

//                 // double g = current->g + ((i >= 4) ? 1.414 : 1.0) * cost; // Diagonal cost factor
//                 double g = current->g + cost; // Diagonal cost factor
//                 double h = manhattan_heuristic(nx, ny, goal_x, goal_y);
//                 Node *neighbor = new Node{nx, ny, g, h, current};
//                 open_list.push(neighbor);
//             }
//             delete current;
//         }

//         ROS_INFO(" count of loop is (%d)", count);

//         // Clean up in case no path was found
//         while (!open_list.empty())
//         {
//             delete open_list.top();
//             open_list.pop();
//         }
//         return false;
//     }

// } // namespace global_planner


















































// working code starts

#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

// Includes the necessary pluginlib macros for registering the planner as a plugin, and includes the header file for this planner.

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)
// Registers the GlobalPlanner class as a plugin, making it available to be used by the ROS navigation stack.

namespace global_planner
{

    GlobalPlanner::GlobalPlanner() : costmap_(nullptr), initialized_(false) {}
    // Default constructor sets the value of private class variables

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : costmap_(nullptr), initialized_(false)
    {
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ = costmap_ros->getCostmap(); // This is the method of costmap_2D class
            global_frame_ = costmap_ros->getGlobalFrameID();
            initialized_ = true;
        }
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("GlobalPlanner has not been initialized");
            return false;
        }

        plan.clear(); // clears any previous path

        unsigned int start_x, start_y, goal_x, goal_y;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
            !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
        {
            ROS_WARN("The start or goal is out of the costmap bounds");
            return false;
        }
        // Converts the start and goal positions from world coordinates to map coordinates. If either conversion fails, logs a warning and returns false

        std::vector<std::vector<double>> distances(costmap_->getSizeInCellsX(), std::vector<double>(costmap_->getSizeInCellsY(), std::numeric_limits<double>::infinity()));

        std::vector<std::vector<geometry_msgs::Point>> predecessors(costmap_->getSizeInCellsX(), std::vector<geometry_msgs::Point>(costmap_->getSizeInCellsY(), geometry_msgs::Point()));

        /*
         Initializes two 2D vectors :

            distances : Stores the shortest distance to each cell,
                       initialized to infinity.predecessors : Stores the predecessor of each cell,
                     initialized to default geometry_msgs::Point.

        */

        std::queue<std::pair<unsigned int, unsigned int>> q;

        distances[start_x][start_y] = 0;
        q.push({start_x, start_y});

        const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
        const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

        while (!q.empty())
        {
            auto current = q.front();
            q.pop();
            unsigned int x = current.first;
            unsigned int y = current.second;

            for (int i = 0; i < 8; ++i)
            {
                unsigned int nx = x + dx[i];
                unsigned int ny = y + dy[i];
                if (nx >= costmap_->getSizeInCellsX() || ny >= costmap_->getSizeInCellsY())
                    continue;

                // if (costmap_->getCost(nx, ny) <= 0)
                // {
                //     continue; // Skip neighbors that are obstacles (negative cost)
                // }

                double new_cost = distances[x][y] + costmap_->getCost(nx, ny);
                // double new_cost = distances[x][y] + 1;
                if (new_cost < distances[nx][ny])
                {
                    distances[nx][ny] = new_cost;
                    predecessors[nx][ny].x = x;
                    predecessors[nx][ny].y = y;
                    q.push({nx, ny});
                }
            }
        }

        if (distances[goal_x][goal_y] == std::numeric_limits<double>::infinity())
        {
            ROS_WARN("No valid path found");
            return false;
        }

        std::vector<geometry_msgs::PoseStamped> reverse_plan;
        unsigned int cx = goal_x;
        unsigned int cy = goal_y;

        while (cx != start_x || cy != start_y)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = global_frame_;
            double wx, wy;
            costmap_->mapToWorld(cx, cy, wx, wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            reverse_plan.push_back(pose);

            geometry_msgs::Point pred = predecessors[cx][cy];
            cx = static_cast<unsigned int>(pred.x);
            cy = static_cast<unsigned int>(pred.y);
        }

        geometry_msgs::PoseStamped start_pose = start;
        start_pose.header.stamp = ros::Time::now();
        reverse_plan.push_back(start_pose);

        plan.assign(reverse_plan.rbegin(), reverse_plan.rend());
        return true;
    }

}; // namespace global_planner

// working code ends

































// #include <pluginlib/class_list_macros.h>
// #include "global_planner.h"

// // Includes the necessary pluginlib macros for registering the planner as a plugin, and includes the header file for this planner.

// PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)
// // Registers the GlobalPlanner class as a plugin, making it available to be used by the ROS navigation stack.

// namespace global_planner
// {

//     // Reconstruct path function
//     void reconstruct_path(Node current, std::vector<std::vector<geometry_msgs::Point>> &predecessors)
//     {
//         // std::vector<Node> total_path = {current};
//         Node *p = &current;
//         Node *q = p->parent;

//         while (p->parent != nullptr)
//         {
//             // current = cameFrom[current];
//             // total_path.push_back(current);

//             predecessors[p->x][p->y].x = q->x;
//             predecessors[p->x][p->y].y = q->y;

//             p = p->parent;
//             q = q->parent;
//         }
//         return;
//         // std::reverse(total_path.begin(), total_path.end());

//         // return total_path; // We get reversed path
//     }

//     double heuristic_cost_estimate(const Node &start, const Node &goal)
//     {
//         // Example: Manhattan distance
//         return (abs(start.x - goal.x) + abs(start.y - goal.y));
//     }

//     // Define your neighbors function
//     std::vector<Node> neighbors(const Node &node)
//     {
//         // Example: 4-connectivity (up, down, left, right)
//         std::vector<Node> result;
//         Node neighbour = node;

//         neighbour.x++;
//         result.emplace_back(neighbour);
//         neighbour.x--;

//         neighbour.x--;
//         result.emplace_back(neighbour);
//         neighbour.x++;

//         neighbour.y++;
//         result.emplace_back(neighbour);
//         neighbour.y--;

//         neighbour.y--;
//         result.emplace_back(neighbour);
//         neighbour.y++;

//         // result.emplace_back(node.x - 1, node.y);
//         // result.emplace_back(node.x, node.y + 1);
//         // result.emplace_back(node.x, node.y - 1);
//         return result;
//     }

//     // std::vector<Node> GlobalPlanner::a_star(Node start_, Node &goal)

//     bool GlobalPlanner::a_star(Node start_, Node &goal, std::vector<std::vector<geometry_msgs::Point>> &predecessors)
//     {
//         Node start = start_;
//         std::set<Node> closedSet;
//         std::set<Node> openSet = {start};

//         // As hash function is not defined for Node
//         // std::unordered_map<Node, Node> parent;
//         // std::unordered_map<Node, double> gScore;
//         // std::unordered_map<Node, double> fScore;

//         // gScore[start] = 0;
//         start.g = 0;
//         // fScore[start] = heuristic_cost_estimate(start, goal);
//         start.h = heuristic_cost_estimate(start, goal);

//         while (!openSet.empty())
//         {
//             // Node current = *std::min_element(openSet.begin(), openSet.end(), [&](Node &a, Node &b)
//             //                                  { return a.f() < b.f(); });

//             Node current = *openSet.begin();

//             if (current.x == goal.x && current.y == goal.y)
//             {

//                 return true;
//                 // return reconstruct_path(cameFrom, current);
//             }

//             // else
//             openSet.erase(current);

//             closedSet.insert(current);

//             for (Node &neighbor : neighbors(current))
//             {
//                 if (closedSet.find(neighbor) != closedSet.end()) // neighbour is in ClosedSet
//                 {
//                     continue;
//                 }
//                 if (neighbor.x >= costmap_->getSizeInCellsX() || neighbor.y >= costmap_->getSizeInCellsY())
//                     continue;

//                 if (openSet.find(neighbor) == openSet.end()) // not in openSet
//                 {
//                     openSet.insert(neighbor);
//                 }

//                 // double tentative_gScore = current.g + 1;
//                 double tentative_gScore = current.g + costmap_->getCost(neighbor.x, neighbor.y);

//                 if (tentative_gScore >= neighbor.g)
//                 {
//                     continue;
//                 }

//                 // cameFrom[neighbor] = current;
//                 neighbor.parent = &current;
//                 // gScore[neighbor] = tentative_gScore;
//                 neighbor.g = tentative_gScore;
//                 neighbor.h = heuristic_cost_estimate(neighbor, goal);
//             }
//         }

//         return false; // Return empty path if failure
//     }

//     GlobalPlanner::GlobalPlanner() : costmap_(nullptr), initialized_(false) {}
//     // Default constructor sets the value of private class variables

//     GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : costmap_(nullptr), initialized_(false)
//     {
//         initialize(name, costmap_ros);
//     }

//     void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) // Initialize function
//     {
//         if (!initialized_)
//         {
//             costmap_ = costmap_ros->getCostmap(); // This is the method of costmap_2D class
//             global_frame_ = costmap_ros->getGlobalFrameID();
//             initialized_ = true;
//         }
//     }

//     bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
//     {
//         if (!initialized_)
//         {
//             ROS_ERROR("GlobalPlanner has not been initialized");
//             return false;
//         }

//         plan.clear(); // clears any previous path

//         unsigned int start_x, start_y, goal_x, goal_y;
//         if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
//             !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
//         {
//             ROS_WARN("The start or goal is out of the costmap bounds");
//             return false;
//         }
//         // Converts the start and goal positions from world coordinates to map coordinates. If either conversion fails, logs a warning and returns false

//         // std::vector<std::vector<double>> distances(costmap_->getSizeInCellsX(), std::vector<double>(costmap_->getSizeInCellsY(), std::numeric_limits<double>::infinity()));

//         std::vector<std::vector<geometry_msgs::Point>> predecessors(costmap_->getSizeInCellsX(), std::vector<geometry_msgs::Point>(costmap_->getSizeInCellsY(), geometry_msgs::Point()));

//         /*
//          Initializes two 2D vectors :

//             distances : Stores the shortest distance to each cell,
//                        initialized to infinity.predecessors : Stores the predecessor of each cell,
//                      initialized to default geometry_msgs::Point.

//         */

//         // std::queue<std::pair<unsigned int, unsigned int>> q;

//         // distances[start_x][start_y] = 0;
//         // q.push({start_x, start_y});

//         // const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
//         // const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

//         // const int dx[4] = {-1, 1, 0, 0};
//         // const int dy[4] = {0, 0, -1, 1};

//         Node start1;
//         start1.x = start_x;
//         start1.y = start_y;

//         Node goal1;
//         goal1.x = goal_x;
//         goal1.y = goal_y;

//         bool A_star = a_star(start1, goal1, predecessors);

//         if (A_star == false)
//         {
//             ROS_WARN("No valid path found");
//             return false;
//         }

//         // while (!q.empty())
//         // {
//         //     auto current = q.front();
//         //     q.pop();
//         //     unsigned int x = current.first;
//         //     unsigned int y = current.second;

//         //     for (int i = 0; i < 4; ++i)
//         //     {
//         //         unsigned int nx = x + dx[i];
//         //         unsigned int ny = y + dy[i];
//         //         if (nx >= costmap_->getSizeInCellsX() || ny >= costmap_->getSizeInCellsY())
//         //             continue;

//         //         double new_cost = distances[x][y] + costmap_->getCost(nx, ny);
//         //         if (new_cost < distances[nx][ny])
//         //         {
//         //             distances[nx][ny] = new_cost;
//         //             predecessors[nx][ny].x = x;
//         //             predecessors[nx][ny].y = y;
//         //             q.push({nx, ny});
//         //         }
//         //     }
//         // }

//         // if (distances[goal_x][goal_y] == std::numeric_limits<double>::infinity())
//         // {
//         //     ROS_WARN("No valid path found");
//         //     return false;
//         // }

//         std::vector<geometry_msgs::PoseStamped> reverse_plan;
//         unsigned int cx = goal_x;
//         unsigned int cy = goal_y;
//         // Node goal2 = goal1;

//         while (cx != start_x || cy != start_y)
//         {
//             geometry_msgs::PoseStamped pose;
//             pose.header.stamp = ros::Time::now();
//             pose.header.frame_id = global_frame_;
//             double wx, wy;

//             costmap_->mapToWorld(cx, cy, wx, wy);
//             pose.pose.position.x = wx;
//             pose.pose.position.y = wy;
//             pose.pose.position.z = 0.0;
//             pose.pose.orientation.w = 1.0;
//             reverse_plan.push_back(pose);

//             geometry_msgs::Point pred = predecessors[cx][cy];
//             cx = static_cast<unsigned int>(pred.x);
//             cy = static_cast<unsigned int>(pred.y);
//         }

//         geometry_msgs::PoseStamped start_pose = start;
//         start_pose.header.stamp = ros::Time::now();
//         reverse_plan.push_back(start_pose);

//         plan.assign(reverse_plan.rbegin(), reverse_plan.rend());
//         return true;
//     }

// }; // namespace global_planner

// ChatGPT updated code

// #include <pluginlib/class_list_macros.h>
// #include "global_planner.h"

// PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

// namespace global_planner
// {
//     double heuristic_cost_estimate(const Node &start, const Node &goal)
//     {
//         return (abs(start.x - goal.x) + abs(start.y - goal.y));
//     }

//     std::vector<Node> GlobalPlanner::neighbors(const Node &node, costmap_2d::Costmap2D *costmap)
//     {
//         std::vector<Node> result;
//         std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
//         for (auto &dir : directions)
//         {
//             int nx = node.x + dir.first;
//             int ny = node.y + dir.second;
//             // Check if neighbor corresponds to an obstacle
//             if (costmap_->getCost(nx, ny) <= 0) {
//                 continue; // Skip neighbors that are obstacles (negative cost)
//             }
//             if (nx >= 0 && ny >= 0 && nx < costmap->getSizeInCellsX() && ny < costmap->getSizeInCellsY())
//             {
//                 result.emplace_back(nx, ny);
//             }
//         }
//         return result;
//     }
//     // OBST
//     void GlobalPlanner::reconstruct_path(Node current, std::vector<std::vector<std::pair<int, int>>> &predecessors)
//     {
//         // while (current.parent != nullptr)
//         // {

//         //     // Check costmap for collision at current cell
//         //     if (costmap_->getCost(current.x, current.y) <= 0)
//         //     {
//         //         ROS_WARN("Collision detected while reconstructing path!");
//         //         // Handle collision (e.g., replan or adjust path)
//         //         break;
//         //     }

//         //     predecessors[current.x][current.y] = {current.parent->x, current.parent->y};
//         //     current = *current.parent;
//         // }

//         while (current.parent != nullptr)
//         {
//             // Check costmap for collision at current cell
//             if (costmap_->getCost(current.x, current.y) <= 0)
//             {
//                 ROS_WARN("Collision detected while reconstructing path!");
//                 // Handle collision (e.g., replan or adjust path)
//                 break;
//             }
//             predecessors[current.x][current.y] = {current.parent->x, current.parent->y};
//             current = *current.parent;
//         }

//         return;
//     }

//     bool GlobalPlanner::a_star(Node start, Node &goal, std::vector<std::vector<std::pair<int, int>>> &predecessors)
//     {
//         auto cmp = [](const Node &a, const Node &b)
//         { return a.f() > b.f(); };
//         std::set<Node, decltype(cmp)> openSet(cmp);

//         std::set<std::pair<int, int>> closedSet;

//         start.g = 0;
//         start.h = heuristic_cost_estimate(start, goal);
//         openSet.insert(start);

//         while (!openSet.empty())
//         {
//             Node current = *openSet.begin();
//             openSet.erase(openSet.begin());

//             if (current == goal)
//             {
//                 reconstruct_path(current, predecessors);
//                 return true;
//             }

//             closedSet.insert({current.x, current.y});

//             for (Node &neighbor : neighbors(current, costmap_))
//             {
//                 if (closedSet.find({neighbor.x, neighbor.y}) != closedSet.end())
//                 {
//                     continue;
//                 }

//                 // double tentative_gScore = current.g + costmap_->getCost(neighbor.x, neighbor.y);
//                 double tentative_gScore = current.g + 1;

//                 if (tentative_gScore < neighbor.g || openSet.find(neighbor) == openSet.end())
//                 {
//                     neighbor.parent = new Node(current);
//                     neighbor.g = tentative_gScore;
//                     neighbor.h = heuristic_cost_estimate(neighbor, goal);

//                     openSet.insert(neighbor);
//                 }
//             }
//         }

//         return false;
//     }

//     GlobalPlanner::GlobalPlanner() : costmap_(nullptr), initialized_(false) {}

//     GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : costmap_(nullptr), initialized_(false)
//     {
//         initialize(name, costmap_ros);
//     }

//     void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
//     {
//         if (!initialized_)
//         {
//             costmap_ = costmap_ros->getCostmap();
//             global_frame_ = costmap_ros->getGlobalFrameID();
//             initialized_ = true;
//         }
//     }

//     bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
//     {
//         if (!initialized_)
//         {
//             ROS_ERROR("GlobalPlanner has not been initialized");
//             return false;
//         }

//         plan.clear();

//         unsigned int start_x, start_y, goal_x, goal_y;
//         if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
//             !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
//         {
//             ROS_WARN("The start or goal is out of the costmap bounds");
//             return false;
//         }

//         std::vector<std::vector<std::pair<int, int>>> predecessors(costmap_->getSizeInCellsX(), std::vector<std::pair<int, int>>(costmap_->getSizeInCellsY(), {-1, -1}));

//         Node start_node(start_x, start_y);
//         Node goal_node(goal_x, goal_y);

//         bool success = a_star(start_node, goal_node, predecessors);

//         if (!success)
//         {
//             ROS_WARN("No valid path found");
//             return false;
//         }

//         std::vector<geometry_msgs::PoseStamped> reverse_plan;
//         unsigned int cx = goal_x, cy = goal_y;

//         while (cx != start_x || cy != start_y)
//         {
//             geometry_msgs::PoseStamped pose;
//             pose.header.stamp = ros::Time::now();
//             pose.header.frame_id = global_frame_;
//             double wx, wy;
//             costmap_->mapToWorld(cx, cy, wx, wy);
//             pose.pose.position.x = wx;
//             pose.pose.position.y = wy;
//             pose.pose.position.z = 0.0;
//             pose.pose.orientation.w = 1.0;
//             reverse_plan.push_back(pose);

//             auto pred = predecessors[cx][cy];
//             cx = pred.first;
//             cy = pred.second;
//         }

//         geometry_msgs::PoseStamped start_pose = start;
//         start_pose.header.stamp = ros::Time::now();
//         reverse_plan.push_back(start_pose);

//         plan.assign(reverse_plan.rbegin(), reverse_plan.rend());
//         return true;
//     }
// }
