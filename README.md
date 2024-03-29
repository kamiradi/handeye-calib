# handeye-calib
---

This package calibrates the realsense camera to the wrist of the panda arm. The idea is to have a basic calibration pipeline down. Follow the steps detailed below

First, run the panda controller from the panda_ultrasound workspace within `~/Documents`.

```
roslaunch franka_example_controllesr_copy cartesian_impedance_eample_controller.launch
```

Next, run the camera and apriltag nodes necessary for the calibration

```
roslaunch handeye_calib camera_for_calib.launch
```

In another terminal window run the following script
```
python scripts/calibration.py
```
