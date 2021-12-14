# Package information

Create pacakge using below command line:
```bash
catkin_create_pkg swag_description rospy roscpp std_msgs
```

Once it is created place URDF and Launch folder generated using solidworks software inside this package

1. This pacakge carries physical robot urdf files
2. This package include all simulated environmental world files for testing in gazebo.
3. Having all models using for testing(ex: barrels, lanes, ramps, etc.)
4. Having meshing(visual appearance) files of robot.
5. Having textures files using for simulation world modifications.


Run below command to bring your swag model on empty world gazebo environment:
```bash
roslaunch swag_description gazebo.launch
```

To move the robots according a custom velocity, use the below commands and give velocties of x,y and z directions: 
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
  ```


(or)
Use telop node to manuever swag robot using keyboard:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```