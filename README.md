## Prerequisites

Before you can run the node, make sure you have the following installed:
- [ROS (Robot Operating System)](http://wiki.ros.org/ROS/Installation)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)



**NOTE:** We are using ROS Noetic

## Installation

1.  **Make workspace (e.g. catkin_ws)**

     ```bash
    cd ~
    mkdir catkin_ws
    cd catkin_ws/
    
    ```

2.  **Clone the Repository**

    ```bash
    git clone https://github.com/niweshsah/Task2_Deimos_NiweshSah.git
    ```

3. **Build the Package**

    Ensure you are in the workspace root directory (e.g., `catkin_ws`).

    ```bash
    catkin init
    catkin build
    ```
4. **Sourcing and setting the environment variables**:

     Type " nano ~/.bashrc " in terminal and add following lines at the end:

     ```bash
    source /opt/ros/noetic/setup.bash
     export TURTLEBOT3_MODEL=burger
     source ~/catkin_ws/devel/setup.bash
     ```

     Save the file and open the terminal:
     ```bash
    source ~/.bashrc
    ```
        
6.  **Move the map.yaml at home directory**
    
    ```bash
    mv ~/catkin_ws/map.yaml ~/
    ```

## Launching Autonomous Navigation Nodes Using Custom Global Planner

To run the node, use the following command:

```bash
rosrun main_launch python_start_launches.py 
```
This will start 4 terminals:
1. Rosmaster
2. Gazebo world simulation with turtlebot3
3. RVIZ
4. Teleoperation node

## Initial Pose Estimation 

1. Click the 2D Pose Estimate button in the RViz menu.
![image](images_github/2d_pose_button.png)
2. Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
3. Repeat step 1 and 2 until the LDS sensor data is overlayed on the saved map.
4. Use keyboard teleoperation node to precisely locate the robot on the map.
5. Move the robot back and forth a bit to collect the surrounding environment information and narrow down the estimated location of the TurtleBot3 on the map which is displayed with tiny green arrows.

<p float="left">
  <img src="images_github/tb3_amcl_particle_01.png" width="200" />
  <img src="images_github/tb3_amcl_particle_02.png" width="200" />
</p>

6. Terminate the keyboard teleoperation node by entering Ctrl + C to the teleop node terminal in order to prevent different cmd_vel values are published from multiple nodes during Navigation.

## Set Navigation Goal
1. Click the 2D Nav Goal button in the RViz menu.
![image](images_github/2d_nav_goal_button.png)
2. Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.
.  This green arrow is a marker that can specify the destination of the robot.
.  The root of the arrow is x, y coordinate of the destination, and the angle θ is determined by the orientation of the arrow.
.  As soon as x, y, θ are set, TurtleBot3 will start moving to the destination immediately.



## Explanation of the Global Planner Algorithm

### Modified Breadth-First Search (BFS) for Path Planning

The modified BFS algorithm used for the global planner is designed to efficiently find the shortest path from the start to the goal position in a grid-based map. Here's a step-by-step explanation:

1. **Initialization**:
   - Initialize a queue and add the start node to it.
   - Mark the start node as visited.

2. **Exploration**:
   - While the queue is not empty, remove the front node from the queue.
   - For each neighbor of the current node:
     - If the neighbor is the goal node, terminate the search and reconstruct the path.
     - If the neighbor has not been visited and is not an obstacle, mark it as visited and add it to the queue.

3. **Path Reconstruction**:
   - Backtrack from the goal node to the start node to reconstruct the path using the parent pointers stored during the exploration phase.

### Comparison with Dijkstra's Algorithm

| Feature                      | Modified BFS                                   | Dijkstra's Algorithm                          |
|------------------------------|------------------------------------------------|-----------------------------------------------|
| **Type**                     | Uninformed search algorithm                    | Informed search algorithm                     |
| **Cost Consideration**       | Assumes uniform cost (all edges have the same cost) | Considers the actual cost of edges            |
| **Data Structure**           | Queue (FIFO)                                   | Priority queue (min-heap)                     |
| **Time Complexity**          | O(V + E)                                       | O((V + E) log V)                              |
| **Space Complexity**         | O(V)                                           | O(V)                                          |
| **Path Optimality**          | Guarantees shortest path for uniform cost grids| Guarantees shortest path for any cost grid    |
| **Usage Scenario**           | Suitable for grids with uniform cost           | Suitable for grids with varying edge costs    |

**Modified BFS** is efficient and straightforward for uniform cost grids, making it suitable for many robotic navigation tasks. It explores all nodes at the present depth level before moving on to nodes at the next depth level, ensuring the shortest path is found in an unweighted grid.

**Dijkstra's Algorithm** is more versatile as it accounts for varying edge costs, making it ideal for environments where the cost of movement between nodes differs. It uses a priority queue to explore nodes in the order of their cumulative cost from the start node.

## Creating a Custom Global Planner from Scratch

1. **Creating new package**

    Delete the existing global_planner package and make a new one using following command:
     ```bash
     cd ~/catkin_ws/src/
     rm -rf global_planner
     catkin_create_pkg global_planer roscpp nav_core nav_msgs tf std_msgs sensor_msgs geometry msgs 
    ```

2. **Build the workspace**

    ```bash
     cd ~/catkin_ws/
     catkin build
    ```

3. **Creating new folder**

    ```bash
     cd ~/catkin_ws/src/global_planner/
     mkdir plugins
    ```

     It is recommended that you put your global planner C++ codes inside this folder.

4. **Writing C++ code for Global Planner**

     It is recommend that you save the file as "global_planner.cpp".
   
    4.1. **Include necessary headers in your source file:**
   
     ```bash
     #include <nav_core/base_global_planner.h>
     #include <pluginlib/class_list_macros.h>
     #include <ros/ros.h>
     ```
     4.2. **Define your class that inherits from nav_core::BaseGlobalPlanner:**

    ```bash
    class MyGlobalPlanner : public nav_core::BaseGlobalPlanner {
     public:
         MyGlobalPlanner();
         MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
         bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, 
                  std::vector<geometry_msgs::PoseStamped>& plan) override;
          };

     ```

   4.3. **Implement the constructor and methods:**
     ```bash
   MyGlobalPlanner::MyGlobalPlanner() {}

     MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
     }

     void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
         // Initialization code here
     }

     bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
         // Planning algorithm here (e.g., A*, BFS)
         return true;
     }
   ```

     4.4. **Make the file executable**

   ```bash
     cd ~/catkin_ws/src/global_planner/plugins
     chmod +x global_planner.cpp
    ```
   
5. **Update CMakeLists.txt**

     Add the following to your CMakeLists.txt to export the plugin:
      ```bash
    install(TARGETS global_planner
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
     )
    ```
      Update your CMakeLists.txt to build the global planner:
   ```bash
    add_library(global_planner plugins/global_planner.cpp)
     add_dependencies(global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
     target_link_libraries(global_planner ${catkin_LIBRARIES}) 
    ```
     
6. **Make plugin.xml for Global Planner**

     Open terminal and write:
   
     ```bash
    cd ~/catkin_ws/src/global_planner/
    touch global_planner_plugin.xml
    ```
     Modify this plugin.xml with the following code:
   
     ```bash
    <library path="lib/libglobal_planner">
    <class name="global_planner/GlobalPlanner"
           type="global_planner::GlobalPlanner"
           base_class_type="nav_core::BaseGlobalPlanner">
        <description>
            My custom global planner implementation.
        </description>
         </class>
     </library>
    ```

7. **Update the package.xml file**

   Add the following line at end of package.xml inside the package tag:
   
     ```bash
       <export>
         <!-- Other tools can request additional information be placed here -->
         <nav_core plugin="${prefix}/global_planner_plugin.xml" />
       </export>
    ```

     This links the package.xml to our plugin.xml.

8. **Build the Workspace**

     ```bash
       cd ~/catkin_ws/
       catkin build
    ```
9. **Updating the move_base.launch**

     Open the terminal and type:
    ```bash
       roscd turtlebot3_navigation/
       cd launch/
    ```
     Now update the move_base.launch file by adding the paramater for our Global Planner
    
     ```bash
      <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
         <param name="base_global_planner" value="global_planner/GlobalPlanner" />
         <!-- If you don't put this line, turtlebot will use the default global planner -->
    
         <!-- Other parameters  -->
      </node>
    ```

10. **Running Autonomous Navigation**

     Follow the steps given above to do the autonomous navigation using your custom global planner.


## Demo Video

Refer to this [video](https://youtu.be/y3TqNdaqOh4).

**Recommended:** Watch this at 2x.

## Resources Used

1. https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
2. https://gazebosim.org/docs
3. https://wiki.ros.org/Documentation
