# Package Information
This package contains navigation stack inforamtion,

## **localisation.launch**:
This launches nodes related to robot localisation application. Below is the details of those nodes:

   **navsat_transform_node** : Converts the gps coordinates (longitude and latitude) information into a world coordinate frame.

   **ekf_localization_node**: Fuses the different odometry sources and gives filtered odometry. This node reads ekf_localization_node_params.yaml file and loads respective covarience matrices, oodometry source(imu, wheel, gps) topics which helps process Extended Kalman Filter algorithm.

   **quaternion_to_rpy**: Converts quaternion format to rpy format. This can be used for localisation applicatiom.

   **wheel_odometry**: It reads simulated motor encoders and share wheel odometry(travelled position) information.

 *Note*: In realtime just rechange topics in ekf_localization_node_params.yaml file for imu, wheel, gps data publishing topics and keep remaining params,  matrices information as same.

**differential_drive.launch**    (only for simulation)
    This node is related to simulated motor controls application. It subscribes the command velocity signals from move_baseFelx stack and share into robot motors as command to follow desired trajectory.

**Navigation_client.launch**:
    This script subscribes movebase_simple/goal topic(rviz 2D Nav Goal) and uses odom-->UTM tf frames transformations and finally sends valid target pose format to /move_to_waypoint topic. This topic internally shares target pose info into move_base_flex via navigation_server script.
                    
**set_waypoint_file_path.launch**                
    This script reads gps waypoints(longitude and lattitude) format .csv file and publishes target waypoints to navigation_server or move_base_flex synchronously via navigation_client node. [In Navigation_client.launch file set read file argument as true, it provides the waypoints .csv file path accurately]

**navigation_server.launch**:
    This node subscribes move_to_waypoint topic and sends into move_base_flex stack using move_base_flex/get_path/goal. This node also publishes move_to_waypoint/current_goalpose, which can used to inspect current target goal given to move_base_flex stack.

**mbf_navigation.launch**
    This launches mbf_costmap_nav contains the costmap navigation server implementation of Move Base Flex. It uses loads costmap based static map as an input for perform autonomous navigation. It provides the Actions for planning, controlling and recovering. At the time of start MBF loads all defined plugins. Therefor, it loads all plugins which are defined in the lists *planners*, *controllers* and *recovery_behaviors.
    ie, it loads 
    global_costmap_params.yaml & local_costmap_params.yaml  --> defines size of local map, robot dimmension and loads plugins helps add detection layer on top of local map. Also helps to add detected lidar obstacle information on map.
    teb_local_planner_params.yaml --> robot controller plugins params
    planners.yaml --> loads planner plugins
    recovery.yaml --> loads recovery_behaviors plugins

**navigation_simulation.launch** --> load all the above launch files.

