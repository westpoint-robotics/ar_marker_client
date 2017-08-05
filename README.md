# ar_marker_client
Code for processing data from an AR marker detection node.

## Installation
Install [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) package which is used for marker detection:
```
$ sudo apt-get install ros-kinetic-ar-track-alvar
```
In addition, install yaml-cpp library:
```
$ sudo apt-get install libyaml-cpp-dev
```
## Parameters
The parameters of the *multi_marker_tracker_node* node are set in [this yaml file](https://github.com/westpoint-robotics/ar_marker_client/blob/master/cfg/multi_marker_tracker.yaml). Some of the parameters are obsolete and are not used in the current version of the code. These parameters should be deleted in the future version (TODO). The parameters used in the current version are as follows:
  * marker_ids  A list of marker ids to track. The first id in this list determines the marker used as the origin of the base marker coordinate frame. The poses of all other markers are determined with respect to the base coordinate frame, as well as the pose of the vehicle being localized.
  * use_optitrack_align Boolean variable which determines if the base coordinate frame is translated and aligned with the optitrack coordinate frame. If set to true, the node collects the samples of relative transform between the optitrack and base marker coordinate frame and computes average transform. This transform is then used to compute the vehicle pose in the optitrack coordinate frame.
  * optitrack_data_vector_length  The number of samples of relative transform betweeen the optitrack and base marker coordinate frame needed to compute the average transform.
  * marker_transform_sample_num The number of samples of relative transform between two detected markers needed to compute the average transform between the markers. After the average transform is computed, a new marker is added to the list of known markers.
  * average_filter_size The number of samples of detected marker pose used to compute average filter over these samples. For each marker, a list of recent samples is stored and the average pose is computed. The most recent sample is discarded if its z position deviates from the average z position by some threshold.
  * average_filter_threshold The threshold used in the average filter of each detected marker.
  * pose_filter_size The number of samples of the vehicle pose used to compute average filter over these samples. Similar to average marker filter, with difference that this filter is applied on the final vehicle pose, and not on the pose of each individual marker.
  * pose_filter_threshold The threshold for the average pose filter.
  * filter_const The filter constant of the first order low pass filter used to reduce the noise in the final vehicle pose. Increase this parameter to smooth the measurement. However, as an increase also produces measurement delay, this parameter should be tuned to find the compromise between satisfactory smoothness and delay.

## Subscribed topics
  * ar_pose_marker ([ar_track_alvar_msgs::AlvarMarkers](http://docs.ros.org/jade/api/ar_track_alvar_msgs/html/msg/AlvarMarkers.html)) - the list of poses of detected markers
  * imu ([sensor_msgs::Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) - contains the imu data of the aerial vehicle being localized. This data is used to compensate the roll and pitch angles of the vehicle. Remap this topic to correspond to the imu topic of your flight controller unit.
  * optitrack/pose ([geometry_msgs::TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html)) - the pose of the vehicle in the optitrack frame. Used to align the optitrack and base marker coordinate frame.
 
## Published topics
  * ar_tracker/pose ([geometry_msgs::PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)) - the unfiltered vehicle pose. This presents the measurement not filtered by the average pose filter nor the first order low pass filter.
  * ar_tracker/pose_f ([geometry_msgs::PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)) - the filtered vehicle pose. This presents the measurement filtered by the average pose filter and the first order low pass filter.
  * ar_tracker/pose_tf ([geometry_msgs::TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html)) - the same as previous, just packed in the different message type required by [multi-sensor fusion node](https://github.com/westpoint-robotics/ethzasl_msf)
  * ar_tracker/marker0 ([geometry_msgs::PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) - the pose of the base marker in the camera frame (used for debugging purposes)
  
 ## Running code
 To launch only multi-marker tracking node, use [this launch file](https://github.com/westpoint-robotics/ar_marker_client/blob/master/launch/multi_marker_tracker.launch). Make sure to remap the imu topic containing imu data of the flight controller unit being used.
 
 Another launch file, which runs the camera node, ar_track_alvar node and multi-marker tracking node is given [here](https://github.com/westpoint-robotics/ar_marker_client/blob/master/launch/ar_alvar_camera_multi_marker_tracking.launch). Adjust the launch parameters important for ar_track_alvar node, namely, the size of each individual marker, the camera topic, the camera info topic and the frequency of measurements. The description of all ar_track_alvar node parameters is given on its  [wiki page](http://wiki.ros.org/ar_track_alvar).
 
 If you are using a [multi-sensor fusion node](https://github.com/westpoint-robotics/ethzasl_msf) alongside this package, use [this launch file](https://github.com/westpoint-robotics/ar_marker_client/blob/master/launch/ar_alvar_camera_multi_marker_tracking_msf.launch) to run everything together.
    
