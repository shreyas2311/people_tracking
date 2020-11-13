# people_tracking
## Building
Download the repository into the src folder. Run the below command to build the code.
```
catkin_make
```

## Launching the process
This ROS package can be used to track humans in the surroundings. Portions of the code for subscribing to image topics derived from the ros people package. The SORT algorithm is used to track humans. On top of 2D measurements, a 3D Kalman Filter is implemented to obtain depth information of humans in the environment.

The launch file sort_tracking_astra.launch can be used to launch the process if ORBBEC astra camera is used.
```
roslaunch sort_tracking sort_tracking.launch
```

The launch file sort_tracking.launch can be used to launch the process if RealSense camera is used.
```
roslaunch sort_tracking sort_tracking_astra.launch
```
Note that tracking will start only if the corresponding ros topic **/people_tracker/person_tracker_measurement** is subscribed to.
