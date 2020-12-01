# ros_pybullet_interface


# Install

1. Create catkin workspace, and `cd` into `src/`
1. `$ git clone git@github.com:cmower/ros_pybullet_interface.git`
1. `$ cd ros_pybullet_interface`
1. `$ rosdep update ; rosdep install --from-paths ./ -iry`
1. Build workspace run test, `$ roslaunch ros_pybullet_interface example.launch` (*note*, you'll need to modify the `urdf` attribute in `configs/kuka.yaml`)

# Usage

Stream target joint states (`sensor_msgs/JointState`) to
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

wherever you run the roslaunch ros_pybullet_interface exampleIK.launch, before you launch the following path needs to be sourced:
export PYTHONPATH=$PYTHONPATH:"local path" + /rbdl/build/python


# Style guide

In the first instance, the following should be used as reference for style and how we should structure a ROS node. Regarding strings, try to always use double quotes `"`.

```python
import os
import sys

GLOBAL_VARIABLE_NAME = 0

def methodName(variable_name):
    pass

class ClassName:

    CLASS_CONSTANT = 0

    def __init__(self):
        rospy.init_node(name, disable_signals=True)
        self.variable_name = 0
        
    def publicMethod(self):
        pass
        
    def setupSomePublisher(self):
        self.pub = rospy.Publisher(TOPIC, MessageType, queue_size=10)
        
    def setupSubscriber(self):
        msg = rospy.wait_for_message(TOPIC, MessageType)
        self.__readSomething(msg)
        self.sub = rospy.Subscriber(TOPIC, MessageType, self.__readSomething)

    def setupTimer(self):
        self.timer = rospy.Timer(rospy.Duration(DT), self.__update_timer)

    def spin(self):
        try:
            rospy.spin()
        except rospy.ROSException as err:
            self.shutdown(err)
        
    def shutdown(self, reason=''):
        self.timer.shutdown()
        self.sub.unregister()
        rospy.signal_shutdown(reason)

    def __privateMethod(self):
        pass

    def __update_timer(self, event):
        if not self.in_finish_state:
            pass
        else:
            self.shutdown()
        
    def __readSomething(self, msg):
        self.something = msg
    
if __name__ == "__main__":
    node = ClassName()
    node.setupPublisher()
    node.setupSubscriber()
    node.start()
    node.spin()
```

Keep the class method `shutdown` public, since sometimes you may not want to specify `disable_signals=True` in `rospy.init_node` and then `shutdown` needs to be passed to `rospy.on_shutdown` (*note*, in this case you don't need to call `rospy.signal_shutdown`). If in doubt, refer to the [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guide.

# Install SDL and ros-keyboard

To run the teleoperation example you need to install [SDL](http://www.libsdl.org/index.php) and [ros-keyboard](https://github.com/lrse/ros-keyboard). Follow these instructions.

*Install SDL*
```
#install sdl2
sudo apt install libsdl2-dev libsdl2-2.0-0 -y;

#install sdl image
sudo apt install libjpeg-dev libwebp-dev libtiff5-dev libsdl2-image-dev libsdl2-image-2.0-0 -y;

#install sdl mixer
sudo apt install libmikmod-dev libfishsound1-dev libsmpeg-dev liboggz2-dev libflac-dev libfluidsynth-dev libsdl2-mixer-dev libsdl2-mixer-2.0-0 -y;

#install sdl true type fonts
sudo apt install libfreetype6-dev libsdl2-ttf-dev libsdl2-ttf-2.0-0 -y;
```

*Install ros-keyboard*
1. `cd /path/to/catkin_ws/src`
1. `git clone git@github.com:lrse/ros-keyboard.git`
1. `catkin build`
