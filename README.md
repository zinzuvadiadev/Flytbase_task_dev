# TurtleSim ROS2 Operations

This project implements a TurtleSim navigation system using ROS2 and PID controllers for smooth waypoint tracking.

## Setup Instructions

### Step 1: Create a ROS2 Workspace
```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Clone the Repository
```sh
git clone <repository-url>
```

If there are any issues for cloning the repository, you can download the zip file and extract the contents into the src folder.

### Step 3: Navigate Back to Workspace Root
```sh
cd ~/ros2_ws
```

### Step 4: Build the Package
```sh
colcon build
```

### Clean Build if Facing Issues
If there are any issues while building, clean and rebuild:
```sh
sudo rm -rf log install build
colcon build
```

## Running Instructions for Goal:1 

### 1. Launching the Turtlesim
```sh
ros2 run turtlesim turtlesim_node
```

### 2. Launching the Turtle Controller
Open another terminal to launch the node
```sh
source install/setup.bash
ros2 run task_1 task_1
```

### 3. Setting Waypoints
Run the node and follow the instructions to enter waypoints for the turtle to follow.

![image](https://github.com/user-attachments/assets/11060e11-d869-4751-834b-96a7a37d8fff)


## Running Instructions for Goal:2 

### 1. Launching the Turtlesim
```sh
ros2 run turtlesim turtlesim_node
```
### 2. Launch the grid controller

Open a new Terminal and run the following commands
```sh
source install/setup.bash
ros2 run task_2 task_2
```
You should see the turtle navigating through a certain set of predefined waypoints to make a grid like pattern like this.

![image](https://github.com/user-attachments/assets/b82672e3-092c-4c99-a140-393848070f09)


## Running Instructions for Goal:3 

### 1. Launching the Turtlesim
```sh
ros2 run turtlesim turtlesim_node
```
### 2. Launch the node for Goal:3

Open a new Terminal and run the following commands.
```sh
source install/setup.bash
ros2 run task_3 task_3
```

### 3. Launch the node for Goal:3

Open a new Terminal and run the following commands for launching a node that will allow you to give custom radius for the circle.
```sh
source install/setup.bash
ros2 run task_3 custom_radius_publisher
```
You can now change the radius of the controller using this node like this.

Launching the node task_3 will also start Publishing real pose of turtle as well as the noisy pose of the turtle every 5 sec on /rt_real_pose and /rt_noisy_pose you can check these pose by running following commands

```sh
ros2 topic echo /rt_real_pose # For real pose
ros2 topic echo /rt_noisy_pose # For noisy pose
```

Circle with custom radius:

![image](https://github.com/user-attachments/assets/cb350b78-d10d-44b3-8aa9-ddd557a83727)

Data published on /rt_real_pose and /rt_noisy_pose:

![image](https://github.com/user-attachments/assets/7e433079-90a1-4e5b-b2ba-9403dd676e0c)


## Running Instructions for Goal:4 

### 1. Launching the Turtlesim
```sh
ros2 run turtlesim turtlesim_node
```
### 2. Launch the node for Goal:3

Open a new Terminal and run the following commands.
```sh
source install/setup.bash
ros2 run task_3 task_3
```

### 3. Launch the node for Goal:3

Open a new Terminal and run the following commands for launching a node that will allow you to give custom radius for the circle.
```sh
source install/setup.bash
ros2 run task_3 custom_radius_publisher
```
Make sure that the radius of the circle is greater than 35 units which will allow the Robber Turtle to gain some distance.

### 4. Launch the node for Goal:4

Open a new Terminal and run the following commands for launching a node that will chase the Robber Turtle(RT) from Goal:3.
```sh
source install/setup.bash
ros2 run task_4 police_turtle
```
The Police Turtle(PT) will start chasing RT on the basis of pose published every 5 secs on /rt_real_pose after 10 secs.

![image](https://github.com/user-attachments/assets/cd04a7e6-0968-4a6f-8c32-b753385b2a2b)


## Running Instructions for Goal:5

### 1. Launching the Turtlesim
```sh
ros2 run turtlesim turtlesim_node
```
### 2. Launch the node for Goal:3

Open a new Terminal and run the following commands.
```sh
source install/setup.bash
ros2 run task_3 task_3
```

### 3. Launch the node for Goal:3

Open a new Terminal and run the following commands for launching a node that will allow you to give custom radius for the circle.
```sh
source install/setup.bash
ros2 run task_3 custom_radius_publisher
```
Make sure that the radius of the circle is greater than 35 units which will allow the Robber Turtle to gain some distance.

### 4. Launch the node for Goal:5

Open a new Terminal and run the following commands for launching a node that will chase the Robber Turtle(RT) from Goal:3.
```sh
source install/setup.bash
ros2 run task_5 task_5
```
The Nerfed Police Turtle(PT), with half of the speed of RT will start chasing RT after 10 secs.

![image](https://github.com/user-attachments/assets/3013c496-4d4d-4167-8c6d-112331bf7f0b)


## Running Instructions for Goal:6

### 1. Launching the Turtlesim
```sh
ros2 run turtlesim turtlesim_node
```
### 2. Launch the node for Goal:3

Open a new Terminal and run the following commands.
```sh
source install/setup.bash
ros2 run task_3 task_3
```

### 3. Launch the node for Goal:3

Open a new Terminal and run the following commands for launching a node that will allow you to give custom radius for the circle.
```sh
source install/setup.bash
ros2 run task_3 custom_radius_publisher
```
Make sure that the radius of the circle is greater than 35 units which will allow the Robber Turtle to gain some distance.

### 4. Launch the node for Goal:6

Open a new Terminal and run the following commands for launching a node that will chase the Robber Turtle(RT) from Goal:3.
```sh
source install/setup.bash
ros2 run task_6 task_6
```
The Police Turtle(PT), will start chasing RT on basis of the gausian noise pose published every 5 secs on /rt_noisy_pose after 10 secs.

## Notes
- Ensure ROS2 is properly sourced before running any commands.
- Modify PID gains in the script for different performance tuning.

