# people_tracking
This ROS package can be used to track humans in the surroundings.

The launch file sort_tracking_astra.launch can be used to launch the process if ORBBEC astra camera is used.
```
roslaunch sort_tracking sort_tracking.launch
```

The launch file sort_tracking.launch can be used to launch the process if RealSense camera is used.
```
roslaunch sort_tracking sort_tracking_astra.launch
```
Note that tracking will start only if the corresponding ros topic **/people_tracker/person_tracker_measurement** is subscribed to.
