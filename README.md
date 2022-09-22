# EKF-CAL

Extended Kalman Filter - Calibration and Localization

## Build
The EKF-CAL package has the following dependencies:
- OpenCV
- Doxygen
- Google Test

Building can be done simply with the following command:

```
colcon build --symlink-install --packages-select ekf_cal
```

For full CMake support, the following expected variables are environment:
- PLANTUML_INSTALL_DIR
- ROS_DISTRO

## Documentation
Documentation can be generated using the following command:
```
doxygen .doxyfile
```

## Launch
For an example of a filter node launch file, see `launch/multi-imu-launch.py`

In particular, note the configuration file `config/multi-imu.yaml` 

The configuration file specifies which sensor topics should to use and the initialization values.
