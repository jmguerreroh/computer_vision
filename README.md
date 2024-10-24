# Computer Vision Examples

![distro](https://img.shields.io/badge/Ubuntu%2024-Nobley%20Numbat-green)
![distro](https://img.shields.io/badge/ROS2-Rolling-blue)
[![rolling](https://github.com/jmguerreroh/computer_vision/actions/workflows/master.yaml/badge.svg?branch=rolling)](https://github.com/jmguerreroh/computer_vision/actions/workflows/master.yaml)

This project contains code examples created in Visual Studio Code for Computer Vision using C++ & OpenCV & Point Cloud Library (PCL) in ROS 2. These examples are created for the Computer Vision Subject of Robotics Software Engineering Degree at URJC.

~~This package is recommended to use with the [TIAGO](https://github.com/jmguerreroh/tiago_simulator) simulator.~~ *Under migration

# Installation 

You need to have previously installed ROS 2. Please follow this [guide](https://docs.ros.org/en/jazzy/Installation.html) if you don't have it.
```bash
source /opt/ros/jazzy/setup.bash
```

Clone the repository to your workspace:
```bash
mkdir -p ~/cv_ws/src
cd ~/cv_ws/src/
git clone https://github.com/jmguerreroh/computer_vision.git
cd ~/cv_ws/
rosdep install --from-paths src --ignore-src -r -y
```

# Building project

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
``` 
# Run

Execute:
```bash
ros2 launch computer_vision cv.launch.py
```
If you want to use your own robot, in the launcher, change the topic names to match the robot topics.

## FAQs:

* /usr/bin/ld shows libraries conflicts between two versions:

Probably you have installed and built your own OpenCV version, rename your local folder:
```bash
mv /usr/local/lib/cmake/opencv4 /usr/local/lib/cmake/oldopencv4
```

## About

This project was made by [José Miguel Guerrero], Associate Professor at [Universidad Rey Juan Carlos].

Copyright &copy; 2024.

[![Twitter](https://img.shields.io/badge/follow-@jm__guerrero-green.svg)](https://twitter.com/jm__guerrero)

## License

This work is licensed under the terms of the [MIT license](https://opensource.org/license/mit).

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero
