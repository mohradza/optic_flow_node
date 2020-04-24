# Optic Flow

Computes tangetial optic flow from an image distorted by a parabolic mirror. Uses Lucas-Kanade pyramidal algorithm.

## Installation

You need OpenCV and Eigen3 libraries.

The driver used to interface with a USB camera is [usb_cam](https://github.com/ros-drivers/usb_cam).

## Usage

A sample launch file to run offline optic flow on recorded usb cam images:

```bash
roslaunch optic_flow_node simple_oflow.launch
```
To simple run the node:

```bash
rosrun optic_flow_node wide_field_optic_flow_node
```

## Wide Optic Flow Node
### Subscribed topics
* **/usb_cam/image_raw** ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))

### Published topics
* **/u_flow** ([std_msgs/Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html)): horizontal component of the averaged optic flow

* **/v_flow**  ([std_msgs/Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html)): vertical component of the averaged optic flow

* **/tang_optic_flow** ([std_msgs/Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html)): tangential averaged optic flow

* **/tang_optic_flow/filtered** ([std_msgs/Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html)): low-pass filtered tangential optic flow

### Parameters
#### Parameters
~ *image_center_x* (int, default: 325): pixel column location of the center.

~ *image_center_y* (int, default: 220): pixel row location of the center.

~ *inner_ring_radius* (double,default: 150.0): size of the first circle.

~ *num_ring_points* (int, default: 30): number of points in one ring.

~ *num_rings* (int, default: 1): number of concentric rings.

~ *ring_dr* (int, default: 5): radial distance between the different rings.

~ *blur_size* (int, default: 5): number of pixels blur (requires *~if_blur* set to true). Must be odd number or it will error out.

#### Filter
~ *alpha* (double, default: 0.7): low pass filter coefficient. 0 does not take new measuremnts into account, 1 does not filter.

#### Lucas-Kanade
~ *pyr_window_size* (int, default: 30): size of the search window at each pyramid level.

~ *pixel_scale* (double, default: 150.0):


#### Switches
~ *if_blur* (bool, default: false): whether to add Gaussian Blur to the image.

~ *enable_debug* (bool, default: true): if enabled, displays the image with resulting flow and publishes **/u_flow** and **/v_flow**.
