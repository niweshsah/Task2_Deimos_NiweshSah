// #include <ros/ros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>

// using std::string;

// #ifndef GLOBAL_PLANNER_CPP
// #define GLOBAL_PLANNER_CPP

// namespace global_planner {

//     class GlobalPlanner : public nav_core::BaseGlobalPlanner {
//         public:

//         GlobalPlanner();
//         GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

//         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
//         bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
//         std::vector<geometry_msgs::PoseStamped>& plan);
//     };
// };
// #endif
























// #ifndef GLOBAL_PLANNER_CPP
// #define GLOBAL_PLANNER_CPP

// #include <ros/ros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>
// #include <vector>
// #include <queue>
// #include <algorithm>

// namespace global_planner {

// class GlobalPlanner : public nav_core::BaseGlobalPlanner {
// public:
//     GlobalPlanner();
//     GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

//     void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
//     bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
//                   std::vector<geometry_msgs::PoseStamped>& plan);

// private:
//     costmap_2d::Costmap2DROS* costmap_ros_;
//     costmap_2d::Costmap2D* costmap_;
//     bool initialized_;
// };

// }  // namespace global_planner

// #endif  // GLOBAL_PLANNER_CPP























// working code starts

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>
#include <queue>
#include <limits>

using std::string;

namespace global_planner
{ // Defines a namespace global_planner to avoid name conflicts with other parts of the program or other libraries.

    class GlobalPlanner : public nav_core::BaseGlobalPlanner
    // nav_core::BaseGlobalPlanner is the class in ros from which we are doing public inheritance
    {
    public:
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        // initializes the planner with a name and a costmap

        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        //   Generates a plan from 'start' to 'goal' , storing the resulting path in 'plan'

    private:
        costmap_2d::Costmap2D *costmap_;
        bool initialized_;
        std::string global_frame_;
    };
};

#endif

// working code ends




























// #ifndef GLOBAL_PLANNER_CPP
// #define GLOBAL_PLANNER_CPP

// #include <ros/ros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>
// #include <vector>
// #include <queue>
// #include <limits>
// #include <unordered_map>
// #include <set>

// using std::string;

// namespace global_planner
// { // Defines a namespace global_planner to avoid name conflicts with other parts of the program or other libraries.

//     struct Node
//     {
//         int x;
//         int y;
//         double g;
//         double h;
//         Node *parent;

//         double f;
        

//         Node()
//         {
//             x = 0;
//             y = 0;
//             g = 0.0;
//             h = 0.0;
//             f = 0.0;
//             parent = nullptr;
//         }

//         Node(int x_, int y_, double g_ = 0, double h_ = 0, Node *p_ = nullptr)
//         {
//             x = x_;
//             y = y_;
//             g = g_;
//             h = h_;
//             f = g_ + h_;
//             parent = p_;
//         }

//         bool operator<( const Node &other) const
//         {
//             return f < other.f; // Compare nodes based on their f value
//         }

//         bool operator==(const Node &other) const
//         {
//             return x == other.x && y == other.y;
//         }
//     };

//     class GlobalPlanner : public nav_core::BaseGlobalPlanner
//     // nav_core::BaseGlobalPlanner is the class in ros from which we are doing public inheritance
//     {
//     public:
//         // bool a_star(Node start_, Node &goal, std::vector<std::vector<geometry_msgs::Point>> &predecessors);
//         GlobalPlanner();
//         GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

//         void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
//         // initializes the planner with a name and a costmap

//         bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
//                       std::vector<geometry_msgs::PoseStamped> &plan);
//         //   Generates a plan from 'start' to 'goal' , storing the resulting path in 'plan'

//     private:
//         costmap_2d::Costmap2D *costmap_;
//         bool initialized_;
//         std::string global_frame_;
//         // std::vector<Node> a_star(Node , Node &);
//         bool a_star(Node, Node &, std::vector<std::vector<geometry_msgs::Point>> &predecessors);
//     };
// };

// #endif

























// GPT code





// #ifndef GLOBAL_PLANNER_CPP
// #define GLOBAL_PLANNER_CPP

// #include <ros/ros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>
// #include <vector>
// #include <queue>
// #include <limits>
// #include <unordered_map>
// #include <set>
// #include <algorithm>

// using std::string;

// namespace global_planner
// {
//     struct Node
//     {
//         int x;
//         int y;
//         double g;
//         double h;
//         Node *parent;

//         Node(int x_, int y_, double g_ = 0, double h_ = 0, Node *p_ = nullptr)
//             : x(x_), y(y_), g(g_), h(h_), parent(p_) {}

//         double f() const
//         {
//             return g + h;
//         }

//         bool operator<(const Node &other) const
//         {
//             return f() < other.f();
//         }

//         bool operator==(const Node &other) const
//         {
//             return x == other.x && y == other.y;
//         }

//         bool operator!=(const Node &other) const
//         {
//             return !(*this == other);
//         }
//     };

//     class GlobalPlanner : public nav_core::BaseGlobalPlanner
//     {
//     public:
//         GlobalPlanner();
//         GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

//         void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
//         bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

//     private:
//         costmap_2d::Costmap2D *costmap_;
//         bool initialized_;
//         std::string global_frame_;
//         bool a_star(Node start, Node &goal, std::vector<std::vector<std::pair<int, int>>> &predecessors);
//         void reconstruct_path(Node current, std::vector<std::vector<std::pair<int, int>>> &predecessors);
//         std::vector<Node> neighbors(const Node &node, costmap_2d::Costmap2D *costmap);
//     };
// };

// #endif
