#!/usr/bin/env python3
import os
import sys
print(sys.version)

import numpy
import rospkg
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped

sys.path.append(
    os.path.join(
        rospkg.RosPack().get_path('ros_pybullet_interface'),
        'include'
    )
)

import pybullet_interface
from utils4localPck import loadYAMLConfig



# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

FREQ = 100 # PyBullet sampling frequency
DT = 1.0/float(FREQ)
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target' # listens for joint states on this topic
CURRENT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current' # publishes joint states on this topic
WORLD_FRAME_ID = 'ros_pybullet_interface/world'
# CURRENT_END_EFFECTOR_TOPIC = 'ros_pybullet_interface/end_effector/current' # publishes end-effector poses on this topic

# ------------------------------------------------------
#
# ROS/PyBullet interface
# ------------------------------------------------------

class ROSPyBulletInterface:

    def __init__(self):

        # Setup
        self.dur = rospy.Duration(DT)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.sensor_pubs = {}
        self.tfs = {}
        self.dynamic_collision_objects = []
        self.static_collision_objects = []

        # Name of node
        self.name = rospy.get_name()
        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # Name of node
        self.name = rospy.get_name()
        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the path to this catkin ws
        self.current_dir = rospack.get_path('ros_pybullet_interface')

        # Get ros parameters
        robot_config_file_name = rospy.get_param('~robot_config')
        camera_config_file_name = rospy.get_param('~camera_config')
        collision_object_file_names = rospy.get_param('~collision_object_config_file_names', [])

        # Setup PyBullet, note publishers/subscribers are also setup internally
        # to these setup functions

        # Initialise pybullet
        pybullet_interface.initPyBullet(DT)
        self.setupPyBulletCamera(camera_config_file_name)
        self.setupPyBulletRobot(robot_config_file_name)

        for file_name in collision_object_file_names:
            self.setupPyBulletCollisionObject(file_name)

        # Main pybullet update
        rospy.Timer(self.dur, self.updatePyBullet)

    def setupPyBulletCamera(self, file_name):

        # Load camera config
        config = loadYAMLConfig(os.path.join(self.current_dir, file_name))

        # Extract data from configuration
        pybullet_interface.setupPyBulletCamera(
            config['distance'],
            config['yaw'],
            config['pitch'],
            config['target_position']
        )

    def setupPyBulletRobot(self, file_name):

        # Load robot configuration
        config = loadYAMLConfig(os.path.join(self.current_dir, file_name))

        # Setup robot
        self.robot = pybullet_interface.PyBulletRobot(os.path.join(self.current_dir, config['urdf_file_name']))
        self.robot.setBasePositionAndOrientation(
            config['base_position'],
            config['base_orient_eulerXYZ']
        )
        self.robot.setJointPositions(config['init_position'])

        # Specify target joint position
        self.target_joint_position = config['init_position']

        # Setup ros publishers
        self.joint_state_pub = rospy.Publisher(
            CURRENT_JOINT_STATE_TOPIC,
            JointState,
            queue_size=10
        )

        # Setup ros timers
        rospy.Timer(self.dur, self.publishPyBulletJointStateToROS)
        rospy.Timer(self.dur, self.publishPyBulletLinkStatesToROS)

        # Setup ros subscriber
        rospy.Subscriber(TARGET_JOINT_STATE_TOPIC, JointState, self.readTargetJointStateFromROS)

        # Setup sensors
        if 'sensors' in config:

            # Setup sensors in PyBullet and their respective ROS publishers
            for label, sensor_parameters in config['sensors'].items():

                # Setup a joint force torque sensor
                if 'joint_force_torque_sensor' in label:

                    # Extract info
                    joint_index = sensor_parameters['joint_index']
                    label = label.split('__')[1] # remove joint_force_torque_sensor__

                    # Setup sensor
                    self.robot.setupJointForceTorqueSensor(label, joint_index)

                    # Setup publisher
                    topic = f'ros_pybullet_interface/joint_force_torque_sensor/{label}'
                    self.sensor_pubs[label] = rospy.Publisher(topic, WrenchStamped, queue_size=10)

            # Setup ros timer to publish sensor readings
            rospy.Timer(self.dur, self.publishPyBulletSensorReadingsToROS)

    def setupPyBulletCollisionObject(self, file_name):

        # Load config
        config = loadYAMLConfig(os.path.join(self.current_dir, file_name))

        # Setup collision object
        obj = pybullet_interface.PyBulletCollisionObject(
            os.path.join(self.current_dir, config['file_name']),
            config['mesh_scale'],
            config['rgba_color'],
            config['base_mass'],
        )

        obj.changeDynamics(
            -1,
            config['lateral_friction'],
            config['spinning_friction'],
            config['rolling_friction'],
            config['restitution'],
            config['linear_damping'],
            config['angular_damping'],
            config['contact_stiffness'],
            config['contact_damping'],
        )

        if 'tf_frame_id' in config['link_state']:
            tf_frame_id = config['link_state']['tf_frame_id']
            self.tfs[tf_frame_id] = {'received': False, 'position': None, 'orientation': None}
            self.dynamic_collision_objects.append({
                'object': obj,
                'tf_frame_id': tf_frame_id,
            })
        else:
            obj.setBasePositionAndOrientation(
                config['link_state']['position'],
                config['link_state']['orientation_eulerXYZ']
            )
            self.static_collision_objects.append({
                'object': obj,
                'position': config['link_state']['position'],
                'orientation': config['link_state']['orientation_eulerXYZ']
            })

    def setPyBulletCollisionObjectPositionAndOrientation(self):
        for obj in self.dynamic_collision_objects:
            tf = self.tfs[obj['tf_frame_id']]
            if not tf['received']: continue
            obj['object'].setBasePositionAndOrientation(tf['position'], tf['orientation'])

    def readROSTfs(self):
        for tf_frame_id in self.tfs.keys():
            try:
                tf = self.tfBuffer.lookup_transform(WORLD_FRAME_ID, tf_frame_id, rospy.Time())
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            self.tfs[tf_frame_id]['position'] = [
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z
            ]
            self.tfs[tf_frame_id]['orientation'] = [
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w
            ]
            self.tfs[tf_frame_id]['received'] = True

    def readTargetJointStateFromROS(self, msg):
        self.target_joint_position = msg.position

    def publishPyBulletJointStateToROS(self, event):
        states = self.robot.getActiveJointStates()
        msg = JointState(
            name = [state['name'] for state in states],
            position = [state['position'] for state in states],
            velocity = [state['velocity'] for state in states],
            effort = [state['motor_torque'] for state in states],
        )
        msg.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(msg)

    def packJointForceTorqueROSMsg(self, reading):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.wrench.force.x = reading[0]
        msg.wrench.force.y = reading[1]
        msg.wrench.force.z = reading[2]
        msg.wrench.torque.x = reading[3]
        msg.wrench.torque.y = reading[4]
        msg.wrench.torque.z = reading[5]
        return msg

    def publishPyBulletSensorReadingsToROS(self, event):
        for reading in self.robot.getSensorReadings():
            if reading['type'] == 'joint_force_torque':
                msg = self.packJointForceTorqueROSMsg(reading['reading'])
                self.sensor_pubs[reading['label']].publish(msg)

    def publishPyBulletLinkStatesToROS(self, event):
        for state in self.robot.getLinkStates():

            label = state['label']
            position = state['position']
            orientation = state['orientation']

            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = WORLD_FRAME_ID
            msg.child_frame_id = 'ros_pybullet_interface/robot/'+state['label']
            msg.transform.translation.x = position[0]
            msg.transform.translation.y = position[1]
            msg.transform.translation.z = position[2]
            msg.transform.rotation.x = orientation[0]
            msg.transform.rotation.y = orientation[1]
            msg.transform.rotation.z = orientation[2]
            msg.transform.rotation.w = orientation[3] # NOTE: the ordering here may be wrong

            # Broadcast tf
            self.tfBroadcaster.sendTransform(msg)

    def updatePyBullet(self, event):
        if not pybullet_interface.isPyBulletConnected():
            rospy.signal_shutdown('PyBullet quit!')
        self.robot.commandJointPosition(self.target_joint_position)
        self.readROSTfs()
        self.setPyBulletCollisionObjectPositionAndOrientation()
        pybullet_interface.stepPyBullet()

# ------------------------------------------------------
#
# Main program
# ------------------------------------------------------

if __name__=='__main__':
    try:
        rospy.init_node('ros_pybullet_interface', anonymous=True)
        ROSPyBulletInterface()
        rospy.on_shutdown(pybullet_interface.closePyBullet)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
