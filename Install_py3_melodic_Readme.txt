for tf2_ros, tf_conversions, PyKDL....

mkdir -p ~/catkin_ws/src;
cd ~/catkin_ws
catkin build
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool set -y src/geometry --git https://github.com/ros/geometry -v 1.12.1
wstool set -y src/orocos_KDL --git https://github.com/orocos/orocos_kinematics_dynamics  -v 1.3.2
wstool up
rosdep install --from-paths src --ignore-src -y -r

catkin build --force-cmake --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source devel/setup.bash
