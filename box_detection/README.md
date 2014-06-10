# Box Detection
## Goal
Detect the position and orientation of a carton box on a flat surface.
The box is approximately 60x10x10cm and of normal brown-greyish carton.

## Requirements
  * ROS: tested with `Groovy` and `Hydro`
  * catkin workspace
  * libraries: PCL, Eigen, pcl_conversions
  * depth camera: openni compatible
      - Microsoft Kinect, ASUS Xtion/PrimeSense

## Usage
Launch the box detection service including the openni camera driver:
```sh
roslaunch box_detection box_detection.launch
```

Call the service to detect boxes and get the pose, it will return 
  * `string box_id`
  * `geometry_msgs/Pose pose`

```sh
rosservice call /box_detection/detect_boxes
```

This service call will also publish a new PlanningScene object (the box) with the box model, position, and orientation.

If there is no provided `base_link` in your current environment, you can use this static transformation to transform the camera data. Make sure to set correct values in the launch-file for your individual camera mounting. 
The `powerball` package stack should provide a valid `base_link` transformation once the robot arm is connected and properly initialised.

```sh
roslaunch box_detection static_transformation.launch
```

## Algorithm
The following steps are performed each time the service for box detection is called:

  * getting the point cloud from the depth camera
  * convert the point cloud to be PCL-compatible
  * compute the best-fitting plane
      - lying in the XY-plane 
      - with normal vector in Z-axis
  * remove outliers from the found plane
      - extract plane from remaining point cloud
  * down sample the point cloud containing the plane
      - only keep points to fill a 1x1cm voxel grid
  * transform the point cloud from `camera_link` to `base_link`
  * create clusters of points within the cloud
  * for each cluster perform:
      - compute convex hull over all points in cluster
      - compute the area of the convex hull
      - check the area to be within the threshold
          + yes: box found, stop further processing
          + no: keep on checking the other clusters
  * if no box is found until now:
      - use remaining point cloud (outliers) to start over

For each found box compute the box pose:
  * position (center point)
      - get min-point and max-point of the cluster point cloud
      - compute vector between these two
      - use the middle of the vector
  * orientation
      - compute a PCA
      - get largest Eigenvalue and Eigenvector
      - compute quaternions between largest Eigenvector and `UnitX`

## Caveats
PCL uses the qhull library for convex hull computation. This piece of software does not comply with any rational sense of output control and just spits out internal computation results to `stdout`. Workaround: Pipe everything through a filter with `grep` to only display your own output (prepend it with a common prefix like `[BOX_DETECTION]`).
