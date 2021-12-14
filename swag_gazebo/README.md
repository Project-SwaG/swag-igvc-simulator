# Package information

This package contains 4 different node scripts

## 1. Control: 
This node is responsible for sharing acurate control commands to simulated robot.

## 2. Ground_truth: 
This node is responsible for sharing odometry values ie, tf transformation between odom frame to base_footprint[using this info we can easily figure out how far robot has been moved from starting position]. Also it provide tf transformation between odom and UTM[gps coordinate frame], this can be used for converting GPS lattitude & longitude values into posestamped[position & orientation] message format.

## 3. Ground_truth_to_rpy: 
This node is responsible for converting quaternion angle values into rpy angles.

## 4. Sim_color_detector: 
This node is responsible for converting top, right, left camera images into fused lane detector image.



Launch files:
There are multiple launch files available but we just focus on 3 main launch files 

1. **swag_control.launch**: This file calls control node by feeding all required motor tuned pararms.

2. **sim_detector.launch**: This file calls sim_detector node and loads all camera required parameters.

3. **simulation.launch**: This is the meta launch file responsible calling gazebo world, robot model, ground_truth node, ground_truth_to_rpy node, controls node and sim_detector node and synchronizes all nodes effeciently.