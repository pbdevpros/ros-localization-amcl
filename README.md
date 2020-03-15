## ros-tracker-bot

Simulation of robot which reads in images from a camera, scans for a white object and moves towards the object.

### Setup for Packages
```bash
$ mkdir catkin_ws
$ cd catkin_ws
$ git clone <project> src
$ cd src
$ catkin_init_workspace
```

### Building Packages
```bash
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch ros_robot world.launch # launches gazebo and Rvitz enviroment
```

Open a separate terminal
```bash
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch # launches the image processing and actuation
```

#### ros_robot pkg
ROS dependencies: message_generation

#### ball_chaser pkg
ROS dependencies: roscpp std_msgs message_generation
