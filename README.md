# Heimdal-Drone-project
A drone project in which I work with ROS2 (Jazzy) and Gazebo (Harmonic V8.9.0) simulator.

Current issue:

The current issue in this project is that the simulations /clock never starts, which crashes the simulation. The next problem is that mavros crashes. This could be due to: 1. Mavros starts multiple times, starts multiple nodes, or starts nodes on multiple channels (it shouldn't though). Or, My labtop is a potato computer, powered by lemons, leaving it very little CPU power (this is likely one case, but not THE case).

Dependencies:

ROS2 Jazzy

Gazebo Harmonic

Ubuntu 24.04 (recommended)

RTABMAP, obtainable from here: https://github.com/introlab/rtabmap

And the ros2 version: https://github.com/introlab/rtabmap_ros

ArduPilot

Mavros2
