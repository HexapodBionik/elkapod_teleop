# Elkapod teleoperation repository
![ROS2 distro](https://img.shields.io/badge/ros--version-humble-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Python Version](https://img.shields.io/badge/python-3.10-g.svg)

## Installation
1. Create a workspace and clone packages into it
```bash
mkdir -p elkapod_teleop/src/
git clone https://github.com/HexapodBionik/elkapod_teleop.git elkapod_teleop/src/
```
2. Move into `src/` folder and download all additional packages using [vcstool](http://wiki.ros.org/vcstool)
```bash
cd elkapod_teleop/src/
vcs import . < repos.yaml
```