# window_detector
This ROS package contains a node of the same name which detects windows

## How it works

The node subscribes to depth image and depth image camera info through image_transport API, only requiring the name
of the topic where the image is published, because the camera info topic is always published on the same namespace 
as the image topic.
For example a camera can publish the following topics:

- `/camera/color/image_raw`
- `/camera/color/camera_info`

Both of them under the namespace `/camera/color`.

`image_transport` API allows to subscribe to both topics only specifying the camera image topic name, through a call to the `image_transport` function `subscribeCamera`.

In this case, the topic to which, the node subscribes by default is `depth_camera/image_raw` which can be remapped to anything else when the node is called (using ROS remapping arguments).

### Image processing

Once the image is received, it is converted from 32-bit float encoding (in meters) to a grayscale image of 8 bits.
The converted image is passed to the function cv::goodFeaturesToTrack which tries to extract four corners from that image.
If the number of corners extracted is equal to 4, a window might be present in the image, so we continue the algorithm.
The function returns the pixel coordinates of each of the corners, stored in a vector.

We must sort the corners before proceeding with the next steps. The ordering is done by the bubbleSort function.
It orders the corners vector according to the following sequence: upper_left corner, lower left_corner, upper_right corner, lower_right corner.

Once ordered, we can get the 3D coordinates of each of the corners, using the camera model parameters.

Then, the window size is checked, if not match the expected dimensions, it is skipped.

If the dimensions are correct, corners are published as markers in the `corners_markers` topic, with the frame name **uav_1**.

Later on, the center and the orientation of the window with respect to the camera reference frame are calculated (It must be taken into account the conventions about the camera reference frame and the window reference frame, described below).

Finally, with the window center and orientation calculated we can publish some arrow markers through the topic `window_axis` and a tf2 transform through `tf`.

### Camera frame convention

In ROS, the camera reference frame axis are oriented as follows:

- z axis points from the optical center towards the scene
- x axis points to the right of the optical center
- y axis points down from the optical center

As shown in the following image:

![alt text](https://docs.nvidia.com/isaac/isaac/_images/coord-frame-camera-frame.png "Camera frame depiction")

### Window frame convention

I decided to use a reference frame for the window similar to the one used for the camera, with the following axis (if we see the window from the outside)

- z axis points towards the opposite side from which the window is seen
- x axis points towards the left
- y axis points to the ground

The center of the reference frame is the center of the window

### Subscribed topics

The node subscribes to:
- `depth_camera/image_raw` with type `sensor_msgs::Image`
- `depth_camera/camera_info` with type `sensor_msgs::CameraInfo`

### Published topics

The node publishes to:
- `corners_markers` with type `visualization_msgs::Marker`
- `window_axis` with type `visualization_msgs::Marker`

## How to run it

For running it, with a depth camera publishing images through the topic /camera/depth/image_raw you need to execute:

`rosrun window_detector window_detector depth_camera/image_raw:=/camera/depth/image_raw`

Then, to visualize the output you must run Rviz, configured to use uav_1 as fixed frame. Subscribe to the corners markers topic and windows axis topic.
