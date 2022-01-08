# Dependencies

## roscpp

roscpp is a C++ implementation of ROS. It provides a client library that enables C++ programmers to quickly interface with ROS Topics, Services, and Parameters. roscpp is the most widely used ROS client library and is designed to be the high-performance library for ROS.

## ur_robot_driver

The new driver for Universal Robots UR3, UR5 and UR10 robots with CB3 controllers and the e-series.

## geometry2

A metapackage to bring in the default packages second generation Transform Library in ros, tf2.

## diagnostics

The diagnostics system is designed to collect information from hardware drivers and robot hardware to users and operators for analysis, troubleshooting, and logging. The diagnostics stack contains tools for collecting, publishing, analyzing and viewing diagnostics data. 

## rqt

rqt is a Qt-based framework for GUI development for ROS. 

## urdf

This package contains a C++ parser for the Unified Robot Description Format (URDF), which is an XML format for representing a robot model.

## moveit

Meta package that contains all essential package of MoveIt!.

## nodelet_core

The nodelet package is designed to provide a way to run multiple algorithms in the same process with zero copy transport between algorithms. This package provides both the nodelet base class needed for implementing a nodelet, as well as the NodeletLoader class used for instantiating nodelets.

## actionlib

The actionlib stack provides a standardized interface for interfacing with preemptable tasks. Examples of this include moving the base to a target location, performing a laser scan and returning the resulting point cloud, detecting the handle of a door, etc.

## ros_core

A metapackage to aggregate the packages required to use publish / subscribe, services, launch files, and other core ROS concepts.

## common_msgs

common_msgs contains messages that are widely used by other ROS packages. These includes messages for actions (actionlib_msgs), diagnostics (diagnostic_msgs), geometric primitives (geometry_msgs), robot navigation (nav_msgs), and common sensors (sensor_msgs), such as laser range finders, cameras, point clouds.
