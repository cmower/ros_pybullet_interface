import rospy

"""

ROS API

The reason for this class is to minimize effort when parsing the
ros_pybullet_interface to ROS2.

When porting, consider notes in:
https://github.com/cmower/ros_pybullet_interface/issues/102

"""

class RosNode:

    def init_node(self, *args, **kwargs):
        rospy.init_node(*args, **kwargs)

    def set_param(self, *args, **kwargs):
        return rospy.set_param(*args, **kwargs)

    def get_param(self, *args, **kwargs):
        return rospy.get_param(*args, **kwargs)

    def Publisher(self,  *args, **kwargs):
        return rospy.Publisher(*args, **kwargs)

    def Subscriber(self, *args, **kwargs):
        return rospy.Subscriber(*args, **kwargs)

    def Service(self, *args, **kwargs):
        return rospy.Service(*args, **kwargs)

    def Timer(self, *args, **kwargs):
        return rospy.Timer(*args, **kwargs)

    def Duration(self, *args, **kwargs):
        return rospy.Duration(*args, **kwargs)

    def logdebug(self, *args, **kwargs):
        rospy.logdebug(*args, **kwargs)

    def loginfo(self, *args, **kwargs):
        rospy.loginfo(*args, **kwargs)

    def logwarn(self, *args, **kwargs):
        rospy.logwarn(*args, **kwargs)

    def logerr(self, *args, **kwargs):
        rospy.logerr(*args, **kwargs)

    def logfatal(self, *args, **kwargs):
        rospy.logfatal(*args, **kwargs)

    def spin(self):
        rospy.spin()
