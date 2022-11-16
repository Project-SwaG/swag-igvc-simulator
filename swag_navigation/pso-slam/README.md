# Particle Swarm Optimisation-SLAM
===========

 This ROS package provides an enhanced implementation of PSO (Particle Swarm Optimisation). It uses Normal Distribution Transform to model the environment.

This package has been tested with ROS Melodic on Ubuntu 18.04 and works without errors.

This project is devided to two parts, the `libndtpso_slam` which is a C++ library that implements the proposed PSO method, and in the other part, the ROS node `ndtpso_slam_node` which provides the ROS interface to this library.
