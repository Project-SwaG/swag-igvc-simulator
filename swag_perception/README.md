# Package Description

This package contains lot many things about various kinds of filters and implementation. Later this output used for obstacle detection and avoidance usecases.

Our aim is to use only pointcloud_filter script from this meta package. Rest all packages have only been used for testing purpose. These have been included in repo for any research purpose of the viewer.

**Note:** Slop filters and elevation mapping nodes are not working as expected since these are having dependency with ANYbotics/elevation_mapping package. So we only utilise pointcloud_filter node from this package.

## Pointcloud_filter: This node subscribes with /velodyne_points topic coming from velodyne lidar and apply PCL filters techniques like 
  **back_filter**-> filters out points behind the lidar,
  **radius_filter**-> filters out points that are further than some radius_squared
  **ground_filter** -> performs some naive ground filtering based on the z-value of the points and remove ground plane,
  **raycast_filter** -> computes the free points by performing a raycasting for each occupied lidar point.

This package publishes the obstacle occupied points on "lidar/occupied" topic, obstacle free points on: "lidar/free", filtered points on: "lidar/filtered" topics. Also publishes /ground and /nonground topics to segement points on ground and points on above ground.
