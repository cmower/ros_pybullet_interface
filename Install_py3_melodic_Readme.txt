for tf2_ros, tf_conversions, PyKDL....

# the following packages should be installed
pip3 install rospkg
pip3 install sip
sudo apt-get install python3-sip-dev

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

Issue is:
Packages were compiled for python2.
Recompile packages for python3 (melodic)
info found: https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/
and here https://github.com/ros/geometry2/issues/259
-------
In case you get something like this after catkin build :

Exception ignored in: <function BaseEventLoop.__del__ at 0x7fa8971964d0>
Traceback (most recent call last):
  File "/usr/local/lib/python3.7/dist-packages/trollius/base_events.py", line 395, in __del__
  File "/usr/local/lib/python3.7/dist-packages/trollius/unix_events.py", line 65, in close
  File "/usr/local/lib/python3.7/dist-packages/trollius/unix_events.py", line 166, in remove_signal_handler
  File "/usr/lib/python3.7/signal.py", line 47, in signal
TypeError: signal handler must be signal.SIG_IGN, signal.SIG_DFL, or a callable object

Then do:
pip3 install --user git+https://github.com/catkin/catkin_tools.git

some explation of the issue can be found here: https://github.com/catkin/catkin_tools/issues/558
and here https://answers.ros.org/question/353113/catkin-build-in-ubuntu-2004-noetic/
and here https://github.com/orocos/orocos_kinematics_dynamics/issues/96
