## How to Task_1 (Mapping):

The following steps explain how to launch and execute the mapping process.

1) Start the ROS master node:  
Open a terminal inside your Docker environment (or native ROS1 workspace) and run:  
"roscore"  
This will initialize the ROS computation graph and enable node communication.  
Tip: use "tmux" or a similar terminal multiplexer to handle multiple parallel terminal windows inside the same session.

2) Launch the mapping node:  
In a new terminal window, execute:  
"roslaunch second_project task_1.launch"  
This will:
- Launch the required ROS nodes for SLAM and visualization
- Start RViz with a predefined configuration
- Launch gmapping for real-time map construction based on laser scan and odometry data

3) Play the dataset (bag file):  
In a third terminal, run:  
"rosbag play --clock robotics2.bag"  
This will provide the robot's sensor data (velocity, laser scans, motion data, etc.) necessary for the mapping process.

4) Save the generated map:  
Once the map is complete, you can save it by running:  
"rosrun map_server map_saver -f map"  
Alternatively, you can use the pre-saved map "map.png" available inside the "maps" folder of the second_project package.

---

## How to Task_2 (Navigation):

The following steps explain how to launch and execute the autonomous navigation process.

1) Start the ROS master node:  
Open a terminal inside your Docker environment (or native ROS1 workspace) and run:  
"roscore"  
This will initialize the ROS computation graph and enable node communication.  
Tip: use "tmux" or a similar terminal multiplexer to handle multiple parallel terminal windows inside the same session.

2) Launch the navigation node:  
In a new terminal window, execute:  
"roslaunch second_project task_2.launch"  
This will:
- Launch the required ROS nodes for navigation and visualization
- Start RViz with a predefined configuration
- Launch Stage to simulate the robot moving in a virtual environment towards a defined goal
