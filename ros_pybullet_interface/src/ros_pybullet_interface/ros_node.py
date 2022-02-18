import rospy

"""

ROS API

The reason for this class is to minimize effort when parsing the
ros_pybullet_interface to ROS2.

A first draft for the RosNode implemented for ROS2 is at the end.

When porting, consider notes in:
https://github.com/cmower/ros_pybullet_interface/issues/102

"""

class RosNode:

    def __init__(self, *args, **kwargs):
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



# import rclpy
# import inspect
# from rclpy.node import Node
# from functools import partial


# class RosNode(Node):

#     def __init__(self, *args, **kwargs):
#         node_name = args[0]
#         super().__init__(node_name)
#         self.logger = self.get_logger()

#     def set_param(self, *args, **kwargs):
#         return rospy.set_param(*args, **kwargs)  # TODO

#     def get_param(self, *args, **kwargs):
#         return rospy.get_param(*args, **kwargs)  # TODO

#     def Publisher(self,  *args, **kwargs):
#         msg_type = args[1]
#         topic = args[0]
#         qos_profile = kwargs['queue_size']
#         return self.create_publisher(msg_type, topic, qos_profile)

#     def Subscriber(self, *args, **kwargs):
#         msg_type = args[1]
#         topic = args[0]
#         callback = args[2]
#         if 'callback_args' in kwargs:
#             partial_args = [callback]
#             callback_args_param_str = list(inspect.signature(callback).parameters.keys())[1]
#             partial_kwargs = {callback_args_param_str: kwargs['callback_args']}
#             callback_use = partial(*partial_args, **partial_kwargs)
#         else:
#             callback_use = callback
#         return self.create_subscription(msg_type, topic, callback_use)

#     def Service(self, *args, **kwargs):
#         srv_type = args[1]
#         srv_name = args[0]
#         callback = args[2]
#         return self.create_service(srv_type, srv_name, callback)

#     def Timer(self, *args, **kwargs):
#         timer_period_secs = args[0]
#         callback = args[1]
#         return self.create_timer(timer_period_secs, callback)

#     def Duration(self, *args, **kwargs):
#         return float(args[0])

#     def _handle_log_args(self, *args):
#         if len(args) > 1:
#             message = args[0] % args[1:]
#         else:
#             message = args[0]
#         return message

#     def logdebug(self, *args, **kwargs):
#         self.logger.debug(self._handle_log_args(*args))

#     def loginfo(self, *args, **kwargs):
#         self.logger.info(self._handle_log_args(*args))

#     def logwarn(self, *args, **kwargs):
#         self.logger.warning(self._handle_log_args(*args))

#     def logerr(self, *args, **kwargs):
#         self.logger.error(self._handle_log_args(*args))

#     def logfatal(self, *args, **kwargs):
#         self.logger.fatal(self._handle_log_args(*args))

#     def spin(self):
#         rclpy.spin(self)
