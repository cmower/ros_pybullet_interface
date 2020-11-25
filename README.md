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
        rospy.Subscriber(TOPIC, MessageType, self.__readSomething)
        
    def start(self):
        rospy.Timer(rospy.Duration(DT), self.__mainLoop)
      
    def __privateMethod(self):
        pass
        
    def __readSomething(self, msg):
        self.something = msg
        
    def __mainLoop(self, event):
        if not self.in_finish_state:
            pass
        else:
            self.shutdown()
        
    def spin(self):
        rospy.spin()
        
    def shutdown(self, reason=''):
        rospy.signal_shutdown(reason)
    
if __name__ == "__main__":
   node = ClassName()
   node.setupPublisher()
   node.setupSubscriber()
   node.start()
   try:
       node.spin()
   except rospy.ROSException as err:
       node.shutdown(err)
```

If in doubt, refer to the [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guide.

