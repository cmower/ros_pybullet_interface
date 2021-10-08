#!/usr/bin/env python3
import rospy
import pybullet
import numpy
from scipy.interpolate import interp1d
from std_msgs.msg import Int64, Float64MultiArray
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import WrenchStamped


from ros_pybullet_interface.config import load_config
from ros_pybullet_interface.tf_interface import TfInterface
from ros_pybullet_interface.pybullet_instance import PybulletInstance
from ros_pybullet_interface.pybullet_robot import PybulletRobot, PybulletVisualRobot
from ros_pybullet_interface.pybullet_dynamic_object import PybulletDynamicObject
from ros_pybullet_interface.pybullet_collision_object import PybulletCollisionObject
from ros_pybullet_interface.pybullet_visual_object import PybulletVisualObject

from ros_pybullet_interface.srv import ManualPybullet, ManualPybulletResponse
from ros_pybullet_interface.srv import PybulletBodyUniqueIds, PybulletBodyUniqueIdsResponse
from ros_pybullet_interface.srv import PybulletRobotJointInfo, PybulletRobotJointInfoResponse
from ros_pybullet_interface.srv import CreateDynamicObject, CreateDynamicObjectResponse
from ros_pybullet_interface.srv import RunCameraBulletTime, RunCameraBulletTimeResponse

class Node:

    """ROS-Pybullet interface node."""

    def __init__(self):

        # Init ros node
        rospy.init_node('ros_pybullet_interface_node')
        ros_node_dur = rospy.Duration(1.0/float(rospy.get_param('~ros_node_freq')))

        # Setup ros status publisher
        self.status_pub = rospy.Publisher('rpbi/active', Int64, queue_size=10)

        # Setup tf interface
        self.tf = TfInterface()

        # Setup pybullet interface
        self.pb = pybullet

        # Initialize pybullet
        config = load_config(rospy.get_param('~pybullet_config'))
        self.pb_instance = PybulletInstance(config, self.pb)
        if self.pb_instance.configure_camera_from_ros:
            rospy.Subscriber('rpbi/camera', Float64MultiArray, self.pb_instance.reset_debug_visualizer_camera_callback)

        # Setup pybullet objects
        self.pb_objects = {}
        for file_name in rospy.get_param('~robot_config_file_names', []):
            #
            # Setup robots
            #

            # Load config
            config = load_config(file_name)
            name = config['name']

            # Create robot instance
            ros_api = {
                'Time.now': rospy.Time.now,
                'joint_state_publisher': rospy.Publisher(f'rpbi/{name}/joint_state', JointState, queue_size=10),
                'joint_states_to_ros_msg': self.joint_states_to_ros_msg,
            }
            robot = PybulletRobot(config, self.tf, self.pb, ros_api)

            # Setup target joint state subscriber
            rospy.Subscriber(f'rpbi/{name}/joint_state/target', JointState, robot.target_joint_state_callback)

            # Setup robot sensors
            for label, sensor_config in config.get('sensors', {}).items():
                sensor_type, sensor_label = label.split('__')
                if sensor_type == 'joint_force_torque_sensor':
                    pub = rospy.Publisher(f'rpbi/{name}/joint_force_torque_sensor/{sensor_label}', WrenchStamped, queue_size=10)
                    robot.add_joint_force_torque_sensor(
                        sensor_label,
                        sensor_config['joint_index'],
                        self.joint_force_torque_sensor_reading_to_ros_msg,
                        pub,
                    )

            # Append robot object
            self.add_pb_object(name, robot)

        for file_name in rospy.get_param('~visual_robot_config_file_names', []):
            #
            # Setup visual robots
            #

            # Load config
            config = load_config(file_name)
            name = config['name']

            # Setup robot instance
            robot = PybulletVisualRobot(config, self.tf, self.pb)

            # Setup target joint state subscriber
            rospy.Subscriber(f'rpbi/{name}/joint_state/target', JointState, robot.target_joint_state_callback)

            # Append robot object
            self.add_pb_object(name, robot)

        for file_name in rospy.get_param('~dynamic_object_config_file_names', []):
            #
            # Setup dynamic objects
            #
            obj = PybulletDynamicObject(load_config(file_name), self.tf, self.pb)
            self.add_pb_object(obj.name, obj)

        for file_name in rospy.get_param('~collision_object_config_file_names', []):
            #
            # Setup collision objects
            #
            obj = PybulletCollisionObject(load_config(file_name), self.tf, self.pb)
            self.add_pb_object(obj.name, obj)

        for file_name in rospy.get_param('~visual_object_config_file_names', []):
            #
            # Setup visual object
            #
            obj = PybulletVisualObject(load_config(file_name), self.tf, self.pb)
            self.add_pb_object(obj.name, obj)

        # Setup services
        # See service methods implemented at end of class definition
        rospy.Service('manual_pybullet_start', ManualPybullet, self.service_start_pybullet)
        rospy.Service('manual_pybullet_step', ManualPybullet, self.service_step_pybullet)
        rospy.Service('manual_pybullet_stop', ManualPybullet, self.service_stop_pybullet)
        rospy.Service('create_dynamic_object', CreateDynamicObject, self.service_create_dynamic_object)
        rospy.Service('pybullet_body_unique_ids', PybulletBodyUniqueIds, self.service_pybullet_body_unique_ids)
        rospy.Service('pybullet_robot_joint_info', PybulletRobotJointInfo, self.service_pybullet_robot_joint_info)
        rospy.Service('run_camera_bullet_time', RunCameraBulletTime, self.service_run_camera_bullet_time)

        # Create publisher for visualizer image if desired
        self.visualizer_publisher = None
        if self.pb_instance.publish_visualizer_to_ros:
            self.visualizer_pub = rospy.Publisher('rpbi/visualizer', Image, queue_size=10)
            rospy.Timer(ros_node_dur, self.publish_visualizer_to_ros)

        # Start simulation
        if self.pb_instance.enable_real_time_simulation:
            self.pb_instance.start_real_time_simulation()

        # Start nodes main loop
        rospy.Timer(ros_node_dur, self.main_loop)

        rospy.loginfo('ros_pybullet_interface node initialized')


    def add_pb_object(self, name, obj):
        assert name not in self.pb_objects.keys(), f"Object name must be unique! '{name}' already exists!"
        self.pb_objects[name] = obj


    #
    # Timer callbacks
    #

    def main_loop(self, event):
        pb_objects = self.pb_objects.copy()  # prevents errors when self.pb_objects changes size during iterations
        for pb_obj in pb_objects.values():
            pb_obj.update()
        self.status_pub.publish(Int64(data=int(self.pb_instance.active)))


    def publish_visualizer_to_ros(self, event):

        # Get image and parse as numpy array
        image = numpy.array(self.pb_instance.get_visualizer_image(), dtype=numpy.uint8).flatten()

        # Pack Image message and publish
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.width = self.pb_instance.config['camera_width']
        msg.height = self.pb_instance.config['camera_height']
        msg.encoding = 'rgba8'
        msg.step = 4*self.pb_instance.config['camera_width']
        msg.data = image.tolist()
        self.visualizer_pub.publish(msg)

    #
    # Spin method
    #

    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.pb_instance.shutdown()

    #
    # Convert to ros message methods
    #

    def joint_force_torque_sensor_reading_to_ros_msg(self, reading):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.wrench.force.x = reading[0]
        msg.wrench.force.y = reading[1]
        msg.wrench.force.z = reading[2]
        msg.wrench.torque.x = reading[3]
        msg.wrench.torque.y = reading[4]
        msg.wrench.torque.z = reading[5]
        return msg


    def joint_states_to_ros_msg(self, joint_states, active_joints, Joint):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [j.name for j in active_joints]
        msg.position = [Joint.position_from_state(state) for state in joint_states]
        msg.velocity = [Joint.velocity_from_state(state) for state in joint_states]
        msg.effort = [Joint.effort_from_state(state) for state in joint_states]
        return msg


    #
    # Define service functions
    #

    def service_start_pybullet(self, req):
        """Start pybullet real time simulation"""
        info = ''
        success = True
        try:
            self.pb_instance.start_real_time_simulation()
            rospy.loginfo('Started Pybullet real time simulation.')
        except Exception as err:
            success = False
            exception_type = type(err).__name__
            msg = str(err)
            info = f"Exception with type {exception_type} was raised with message: {msg}"
            rospy.logwarn('Failed to start Pybullet real time simulation!\nReason: %s', info)
        return ManualPybulletResponse(info=info, success=success)


    def service_step_pybullet(self, req):
        """Manually step pybullet real time simulation"""
        info = ''
        success = True
        hz = self.pb_instance.hz if req.hz <= 0 else req.hz
        rate = rospy.Rate(hz)
        try:
            self.pb_instance.step(req.num_steps, rate)
            s = 's' if req.num_steps > 1 else ''
            rospy.loginfo('Successfully stepped pybullet %d time%s.', req.num_steps, s)
        except Exception as err:
            exception_type = type(err).__name__
            msg = str(err)
            info = f"Exception with type {exception_type} was raised with message: {msg}"
            rospy.logwarn('Failed to step Pybullet!\nReason: %s', info)
        return ManualPybulletResponse(info=info, success=success)


    def service_stop_pybullet(self, req):
        """Stop pybullet real time simulation"""
        info = ''
        success = True
        try:
            self.pb_instance.stop_real_time_simulation()
            rospy.loginfo('Stopped Pybullet real time simulation.')
        except Exception as err:
            success = False
            exception_type = type(err).__name__
            msg = str(err)
            info = f"Exception with type {exception_type} was raised with message: {msg}"
            rospy.logwarn('Failed to stop Pybullet simulation!\nReason: %s', info)
        return ManualPybulletResponse(info=info, success=success)


    def service_create_dynamic_object(self, req):
        """Creates a dynamic object on-the-fly"""
        info = ''
        success = True
        try:
            # Do not overwrite objects
            name = req.name
            if name in self.pb_objects.keys():
                old_name = name
                name = name + '_' + str(rospy.Time.now().to_nsec())
                rospy.logwarn("Trying to create new dynamic object but the name is not unique!\nRequested name: %s\nNamed used: %s", old_name, name)

            # Create config
            config = dict(
                name=name,
                base_mass=req.base_mass,
                object_type=req.object_type,
                rgba_color=req.rgba_color,
                mesh_scale=req.mesh_scale,
                radius=req.radius,
                height=req.height,
                half_extends=req.half_extends,
                init_base_orient_eulerXYZ=req.init_base_orient_eulerXYZ,
                init_position=req.init_position,
                init_linear_velocity=req.init_linear_velocity,
                init_angular_velocity=req.init_angular_velocity,
                lateral_friction=req.lateral_friction,
                spinning_friction=req.spinning_friction,
                rolling_friction=req.rolling_friction,
                restitution=req.restitution,
                linear_damping=req.linear_damping,
                angular_damping=req.angular_damping,
                contact_stiffness=req.contact_stiffness,
                contact_damping=req.contact_damping,
            )

            # Create object
            obj = PybulletDynamicObject(config, self.tf, self.pb)

            # Add pb object
            self.add_pb_object(name, obj)

            rospy.loginfo('Successfully created dynamic object %s', name)

        except Exception as err:
            success = False
            exception_type = type(err).__name__
            msg = str(err)
            info = f"Exception with type {exception_type} was raised with message: {msg}"
            rospy.logwarn('Failed to create dynamic object!\nReason: %s', info)
        return CreateDynamicObjectResponse(info=info, success=success)


    def service_pybullet_body_unique_ids(self, req):
        """Return Pybullet body unique ids"""
        info = ''
        success = True
        body_unique_ids = []
        object_names = []
        try:
            pb_objects = self.pb_objects.copy()
            object_names = list(pb_objects.keys())
            for object_name in object_names:
                body_unique_ids.append(pb_objects[object_name].body_unique_id)
            rospy.loginfo('Successfully collected body unique ids')
        except Exception as err:
            exception_type = type(err).__name__
            msg = str(err)
            info = f"Exception with type {exception_type} was raised with message: {msg}"
            success = False
            rospy.logwarn('Failed to collect body unique ids!\nReason: %s', info)
        return PybulletBodyUniqueIdsResponse(object_names=object_names, body_unique_ids=body_unique_ids, info=info, success=success)


    def service_pybullet_robot_joint_info(self, req):
        """Return the link/joint indices for a given robot"""
        info = ''
        success = True
        joint_index = []
        joint_name = []
        joint_type = []
        try:
            robot = self.pb_objects[req.robot_name]
            for joint in robot.joints:
                joint_index.append(joint.index)
                joint_name.append(joint.name)
                joint_type.append(joint.type_as_string)
            ndof = robot.ndof
            ndof_active = robot.ndof_active
            rospy.loginfo('Succesfully collected joint information for robot "%s"', req.robot_name)
        except Exception as err:
            exception_type = type(err).__name__
            msg = str(err)
            info = f"Exception with type {exception_type} was raised with message: {msg}"
            rospy.logwarn('Failed to collect joint information for robot "%s"\nReason: %s', req.robot_name, info)
        return PybulletRobotJointInfoResponse(
            joint_index=joint_index,
            joint_name=joint_name,
            joint_type=joint_type,
            ndof=ndof,
            ndof_active=ndof_active,
            info=info,
            success=success,
        )

    def service_run_camera_bullet_time(self, req):

        # Setup
        info = ''
        success = True
        max_pitch = 45

        # Check camera not being configured from ROS
        if self.pb_instance.configure_camera_from_ros:
            info = 'camera being configured from ROS, cannot run bullet time'
            success = False
            rospy.logwarn('You cannot call run_camera_bullet_time service')
            return RunCameraBulletTimeResponse(info=info, success=success)


        try:
            # Extract arguments
            distance = req.distance
            target_position = req.target_position
            duration = req.duration
            initial_yaw = req.initial_yaw

            # Generate trajectory
            N = 1000
            yaw_trajectory = initial_yaw + numpy.linspace(0, 360, N)
            x_ = numpy.linspace(0, 1, N)
            pitch_trajectory = -max_pitch*numpy.exp(-3*x_**2)*numpy.cos(x_)

            # Interpolate trajectory in terms of temp parameter p
            p = numpy.linspace(0, 1, N)
            yaw_trajectory_interp = interp1d(p, yaw_trajectory, fill_value="extrapolate", kind='cubic')
            pitch_trajectory_interp = interp1d(p, pitch_trajectory, fill_value="extrapolate", kind='cubic')

            # Get current camera config
            orig_camera_config = self.pb_instance.camera_config

            # Run bullet-time
            alpha = 0.0
            rate = rospy.Rate(50)
            start_time = rospy.Time.now().to_sec()
            while alpha < 1.0:
                t = rospy.Time.now().to_sec() - start_time
                alpha = t/duration
                yaw = yaw_trajectory_interp(alpha)
                pitch = pitch_trajectory_interp(alpha)
                config = [distance, yaw, pitch, target_position[0], target_position[1], target_position[2]]
                self.pb_instance.reset_debug_visualizer_camera(config)
                rate.sleep()

            # Return camera to original position, if required
            if req.return_to_orig_camera_config_after:
                self.pb_instance.reset_debug_visualizer_camera(orig_camera_config)

            rospy.loginfo('Completed bullet-time run.')
        except Exception as err:
            success = False
            exception_type = type(err).__name__
            msg = str(err)
            info = f"Exception with type {exception_type} was raised with message: {msg}"
            rospy.logwarn('Failed to run bullet-time!\nReason: %s', info)

        return RunCameraBulletTimeResponse(info=info, success=success)

def main():
    Node().spin()


if __name__ == '__main__':
    main()
