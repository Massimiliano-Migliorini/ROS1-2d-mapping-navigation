## Coordinate Frames and Assumptions

Theoretical assumption: in both nodes, the Odom reference frame corresponds to the static position of the robot, whereas the local reference frames linked to the car — precisely vehicle and gps — have the x-axis pointing in the direction of the car's heading.

Consequences of the convention:
1) To integrate odometry in Node 1, we selected "theta_init = 0", in order to overlap the Odom and Vehicle frames at the starting pose and remain consistent with the odometry integration formulas.
2) To compute odometry from GPS data, after converting the "(latitude, longitude, altitude)" measurements to ENU coordinates, it was necessary to apply a fixed rotation from ENU to Odom equal to the car’s static orientation at the initial time.

---

## How to Run the Code

The following steps explain how to properly launch and execute the project:

1) Start the ROS master node:  
Open a terminal inside your Docker environment (or native ROS1 workspace) and run:  
"roscore"  
This will initialize the ROS computation graph and enable node communication.  
Tip: use "tmux" or a similar terminal multiplexer to handle multiple parallel terminal windows inside the same session.

2) Launch the main ROS node with visualization:  
In a new terminal window, execute:  
"roslaunch first_project launch.launch"  
This will:  
- Start both the required ROS nodes  
- Load all necessary parameters  
- Automatically launch "rviz" with the predefined configuration

3) Play the dataset (bag file):  
In a third terminal, run the following command to replay the recorded data from the self-driving vehicle:  
"rosbag play --clock project.bag"  
This will publish all required topics (velocity, steering, GPS data, etc.) into the ROS environment, allowing the nodes to process and visualize the data in real time.




          