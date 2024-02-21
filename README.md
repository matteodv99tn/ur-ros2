# Universal Robot ROS2 collection

Repository containing personal launch files for Universal Robots manipulators.

## Moveit note
As discussed [here](https://github.com/ros-planning/moveit2/issues/1782), it is possible that launching moveit might result with errors in loading OMPL libraries.
To solve this issue prepend ``LC_NUMERIC=en_US.UTF-8`` to the launch command call, i.e.
```
LC_NUMERIC=en_US.UTF-8 ros2 launch magician_ur moveit.launch.py
```