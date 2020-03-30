## ros-localization-amcl

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
$ cd src
$ roslaunch ros_robot amcl.launch # launch amcl localization node
```

Open a separate terminal
```bash
$ source devel/setup.bash
$ cd src
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py # launch teleop keyboard control for robot
```

### map generation
In order to create the global map used by the robot, the pgm_map_creator package was used. 
This has a dependency on  libignition-math2-dev and protobuf-compiler to compile the map creator.
Use the following to install these libraries.
```bash
$ sudo apt-get install libignition-math2-dev protobuf-compiler
```

The pgm repo should then be installed:
```bash
$ cd src
$ git clone https://github.com/udacity/pgm_map_creator.git
```

A map is created from a world file by placing into the ```pgm_map_creator/world``` folder and including the line
```<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>``` in the world file.

To create the map, then run:
```bash
$ gzserver src/pgm_map_creator/world/<YOUR GAZEBO WORLD FILE>
$ roslaunch pgm_map_creator request_publisher.launch
```
and copy it into the src/ros_robot/maps/ folder. A yaml file is created, in the same maps/ folder, with the associated parameters used from the ```request_publisher.launch``` file, from the ```pgm_map_creator``` package.


