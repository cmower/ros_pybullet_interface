# ros_pybullet_interface

Install:
1. Create catkin workspace, and `cd` into `src/`
1. `$ git clone git@github.com:cmower/ros_pybullet_interface.git`
1. `$ cd ros_pybullet_interface`
1. `$ rosdep update ; rosdep install --from-paths ./ -iry`
1. Build workspace run test, `$ roslaunch ros_pybullet_interface example.launch` (*note*, you'll need to modify the `urdf` attribute in `configs/kuka.yaml`)

Stream joint states (`sensor_msgs/JointState`) to
```
    /ros_pybullet_interface/joint_state/target
```

Joint states (`sensor_msgs/JointState`) from PyBullet are streamed to
```
    /ros_pybullet_interface/joint_state/current
```

End-effector poses (`geometry_msgs/PoseStamped`) from PyBullet are streamed to
```
    /ros_pybullet_interface/end_effector/current
```
