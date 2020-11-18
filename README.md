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

The end-effector transform in the world coordinate system is broadcast as a `tf` using the `tf2` package.


# ros_rbdl_IK_interace

RBDL need to be installed: (instructions according to https://github.com/rbdl/rbdl)
1. Create folder, and `cd` into 
2. `$ git clone https://github.com/rbdl/rbdl`
3. mkdir build, and cd build/ 
3,5.  NOTE:  Eigen3 linear algebra library should be installed, probably is installed already, but check!  
4. cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX="local path" ../ 
5. Configure cmake:
    5i.   ccmake .
    5ii.  find RBDL_BUILD_PYTHON_WRAPPER  and set it to ON 
    5iii. find RBDL_USE_ROS_URDF_LIBRARY  and set it to OFF
    5iv.  press c e g ... etc (to configure)
5. make 
6. make install 
