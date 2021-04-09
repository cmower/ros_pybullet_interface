#!/usr/bin/env python3
import os
import sys

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped

from ros_pybullet_interface import pybullet_interface
from ros_pybullet_interface.utils import loadYAMLConfig, ROOT_DIR
from ros_pybullet_interface.srv import setObjectState, setObjectStateResponse

# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

ROS_FREQ = 100       # ROS loop sampling frequency
ROS_DT = 1.0/float(ROS_FREQ)
PYBULLET_FREQ = ROS_FREQ #256 # PyBullet simulation loop sampling frequency
PYBULLET_DT = 1.0/float(PYBULLET_FREQ)
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'  # listens for joint states on this topic
CURRENT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current'  # publishes joint states on this topic
WORLD_FRAME_ID = 'ros_pybullet_interface/world'


# ------------------------------------------------------
#
# Helper functions
# ------------------------------------------------------

def packTransformStamped(tf_frame_id, tf_child_id, p, q):
    tf = TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = tf_frame_id
    tf.child_frame_id = tf_child_id
    for i, dim in enumerate(['x', 'y', 'z']):
        setattr(tf.transform.translation, dim, p[i])
        setattr(tf.transform.rotation, dim, q[i])
    tf.transform.rotation.w = q[3]
    return tf

# ------------------------------------------------------
#
# ROS/PyBullet interface
# ------------------------------------------------------


class ROSPyBulletInterface:

    def __init__(self):

        # Setup ros node
        rospy.init_node(
            'ros_pybullet_interface', anonymous=True, disable_signals=True
        )
        self.name = rospy.get_name()

        # Setup
        self.dur = rospy.Duration(ROS_DT)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.sensor_pubs = {}
        self.tfs = {}
        self.dynamic_collisionvisual_objects = []
        self.static_collisionvisual_objects = []
        self.static_collision_objects = []
        self.objects = []

        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # get the path to this catkin ws
        self.current_dir = ROOT_DIR

        # Get ros parameters
        robot_config_file_name = rospy.get_param('~robot_config')
        camera_config_file_name = rospy.get_param('~camera_config')
        collision_object_file_names = rospy.get_param('~collision_object_config_file_names', [])
        visual_object_file_names = rospy.get_param('~visual_object_config_file_names', [])
        object_file_names = rospy.get_param('~object_config_file_names', [])

        self.visframes = rospy.get_param("~visframes", [])

        if rospy.has_param('~Z_gravity'):
            grav = [0., 0., rospy.get_param('~Z_gravity')]
        else:
            grav = [0., 0., 0.]
        # Setup PyBullet, note publishers/subscribers are also setup internally
        # to these setup functions

        # Initialise pybullet
        pybullet_interface.initPyBullet(ROS_DT, gravity = grav)
        self.setupPyBulletCamera(camera_config_file_name)
        self.setupPyBulletRobot(robot_config_file_name)

        for file_name in collision_object_file_names:
            self.setupPyBulletCollisionObject(file_name)

        for file_name in visual_object_file_names:
            self.setupPyBulletVisualObject(file_name)

        for file_name in object_file_names:
            self.setupPyBulletObject(file_name)

        if len(self.visframes) > 0:
            rospy.logwarn('Link visualization can lead to significantly slower performance.')

        for linkid in self.visframes:
            self.setupPyBulletVisualLinks(linkid)

        # set the server for changing object state
        self.setObjectStateServer()

        if rospy.get_param('~pybullet_sim_self_loop'):
            pybullet_interface.updateTimeStep(PYBULLET_DT)
            pybullet_interface.runPyBullet()
            self.step = self._null
        else:
            self.step = self._step

        # rospy.sleep(1.0)
        # Main pybullet update
        self.main_timer = rospy.Timer(self.dur, self.updatePyBullet)

    def _null(self, *args, **kwargs):
        pass

    def _step(self):
        pybullet_interface.stepPyBullet()

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
        self.robot = pybullet_interface.PyBulletRobot(
            os.path.join(self.current_dir, config['file_name'])
        )
        self.robot.setBasePositionAndOrientation(
            config['base_position'],
            np.deg2rad(config['base_orient_eulerXYZ'])
        )
        qinit = np.deg2rad(config['init_position'])
        self.robot.setJointPositions(qinit)

        # Specify target joint position
        self.target_joint_position = qinit

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
        rospy.Subscriber(
            TARGET_JOINT_STATE_TOPIC, JointState, self.readTargetJointStateFromROS
        )

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
                    self.sensor_pubs[label] = rospy.Publisher(
                        topic, WrenchStamped, queue_size=10
                    )

            # Setup ros timer to publish sensor readings
            rospy.Timer(self.dur, self.publishPyBulletSensorReadingsToROS)


    def setupPyBulletVisualLinks(self, linkid):
        self.tfs[linkid] = {
            'received': False, 'position': None, 'orientation': None
        }

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

        # obj.changeDynamics(
        #     -1,
        #     config['lateral_friction'],
        #     config['spinning_friction'],
        #     config['rolling_friction'],
        #     config['restitution'],
        #     config['linear_damping'],
        #     config['angular_damping'],
        #     config['contact_stiffness'],
        #     config['contact_damping'],
        #     config['localInertiaDiagonal']
        # )

        if 'tf_frame_id' in config['link_state']:
            tf_frame_id = config['link_state']['tf_frame_id']
            self.tfs[tf_frame_id] = {
                'received': False, 'position': None, 'orientation': None
            }
            self.dynamic_collisionvisual_objects.append({
                'object': obj,
                'tf_frame_id': tf_frame_id,
            })
        else:
            obj.setBasePositionAndOrientation(
                config['link_state']['position'],
                np.deg2rad(config['link_state']['orientation_eulerXYZ'])
            )
            self.static_collisionvisual_objects.append({
                'object': obj,
                'position': config['link_state']['position'],
                'orientation': config['link_state']['orientation_eulerXYZ']
            })

    def setupPyBulletVisualObject(self, file_name):

        # Load config
        config = loadYAMLConfig(os.path.join(self.current_dir, file_name))

        # Setup visual object
        obj = pybullet_interface.PyBulletVisualObject(
            os.path.join(self.current_dir, config['file_name']),
            config['mesh_scale'],
            config['rgba_color'],
        )

        if 'tf_frame_id' in config['link_state']:
            tf_frame_id = config['link_state']['tf_frame_id']
            self.tfs[tf_frame_id] = {
                'received': False, 'position': None, 'orientation': None
            }
            self.dynamic_collisionvisual_objects.append({
                'object': obj,
                'tf_frame_id': tf_frame_id,
            })
        else:
            obj.setBasePositionAndOrientation(
                config['link_state']['position'],
                np.deg2rad(config['link_state']['orientation_eulerXYZ'])
            )
            self.static_collisionvisual_objects.append({
                'object': obj,
                'position': config['link_state']['position'],
                'orientation': config['link_state']['orientation_eulerXYZ'],
            })
            if 'pub_tf' in config['link_state']:
                static_col_obj['pub_tf'] = config['link_state']['pub_tf']
            else:
                static_col_obj['pub_tf'] = False

            if 'name' in config:
                name = config['name']
            else:
                name = 'obj' + str(len(self.static_collision_objects))
            static_col_obj['name'] = name
            self.static_collision_objects.append(static_col_obj)

    def setupPyBulletObject(self, file_name):

            # Load config
            config = loadYAMLConfig(os.path.join(self.current_dir, file_name))

            if 'urdf' in config:
                obj= pybullet_interface.PyBulletObject(os.path.join(self.current_dir, config['file_name']),
                    config['mesh_scale'],
                    config['rgba_color'],
                    config['base_mass'],
                    loadURDF=os.path.join(self.current_dir, config['urdf']))
            else:
                # Setup visual object
                obj = pybullet_interface.PyBulletObject(
                    os.path.join(self.current_dir, config['file_name']),
                    config['mesh_scale'],
                    config['rgba_color'],
                    config['base_mass'],
                )

            obj.changeDynamics(
                -1, # which is the link index
                config['lateral_friction'],
                config['spinning_friction'],
                config['rolling_friction'],
                config['restitution'],
                config['linear_damping'],
                config['angular_damping'],
                config['contact_stiffness'],
                config['contact_damping'],
                config['localInertiaDiagonal']
            )

            obj.setBasePositionAndOrientation(
                config['link_state']['position'],
                config['link_state']['orientation_eulerXYZ'],
            )

            # book-keeping object names and ids
            if 'name' in config:
                name = config['name']
            else:
                name = 'obj' + str(len(self.objects))

            self.objects.append({
                'object_id': obj.getObjectID(),
                'object_name': name
            })


    def setPyBulletCollisionVisualObjectPositionAndOrientation(self):
        for obj in self.dynamic_collisionvisual_objects:
            tf = self.tfs[obj['tf_frame_id']]
            if not tf['received']: continue
            obj['object'].setBasePositionAndOrientation(
                tf['position'], tf['orientation']
            )

    def readROSTfs(self):
        for tf_frame_id in self.tfs.keys():
            try:
                tf = self.tf_buffer.lookup_transform(
                    WORLD_FRAME_ID, tf_frame_id, rospy.Time()
                )
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
            name=[state['name'] for state in states],
            position=[state['position'] for state in states],
            velocity=[state['velocity'] for state in states],
            effort=[state['motor_torque'] for state in states],
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

    def publishStaticTransformsToROS(self):
        for static_obj in self.static_collision_objects:
            if static_obj['pub_tf']:
                self.tf_broadcaster.sendTransform(
                    packTransformStamped(
                        WORLD_FRAME_ID,
                        'ros_pybullet_interface/'+static_obj['name'],
                        static_obj['position'],
                        static_obj['orientation']
                    )
                )

    def publishPyBulletLinkStatesToROS(self, event):
        for state in self.robot.getLinkStates():
            self.tf_broadcaster.sendTransform(
                packTransformStamped(
                    WORLD_FRAME_ID,
                    'ros_pybullet_interface/robot/%s' % state['label'],
                    state['position'],
                    state['orientation']
                )
            )

    def publishObjectStateTransToROS(self):
        for obj in self.objects:
            pos, orient = pybullet_interface.getObjectPosOrient(obj['object_id'])
            self.tf_broadcaster.sendTransform(
                    packTransformStamped(
                        WORLD_FRAME_ID,
                        'ros_pybullet_interface/'+obj['object_name'],
                        pos,
                        orient
                    )
                )

    def visualizeLinks(self):
        for linkid in self.visframes:
            tf = self.tfs[linkid]
            if tf['received']:
                pybullet_interface.visualizeFrameInWorld(
                    tf['position'], tf['orientation']
                )

    def setObjState(self, req):

        obj_id = None
        for obj in self.objects:
            if obj['object_name']==req.obj_name:
                obj_id = obj['object_id']

        if obj_id==None:
            rospy.logwarn(f"{obj_name} was not found...")
            return

        stringPybullet = pybullet_interface.getPybulletObject_hack()
        stringPybullet.changeDynamics(obj_id, -1, angularDamping = 0, rollingFriction = 0, spinningFriction = 0,
                                      localInertiaDiagonal = [1.0, 1.0, 2.0])
        print("object dynamics:", stringPybullet.getDynamicsInfo(obj_id, -1))

        pybullet_interface.setObjectPosOrient(obj['object_id'], req.pos, req.quat)
        pybullet_interface.setObjectVelLinAng(obj['object_id'], req.lin_vel, req.ang_vel)
        rospy.loginfo(f"Returning object position and orientation: {req.pos}, {req.quat}")
        rospy.loginfo(f"Returning object linear and angular velocity: {req.lin_vel}, {req.ang_vel}")

        return setObjectStateResponse("True: Set the state of the object successfully")


    def setObjectStateServer(self):

        s = rospy.Service('set_object_state', setObjectState, self.setObjState)
        rospy.loginfo("Server is ready to set the state of the object.")


    def updatePyBullet(self, event):
        if not pybullet_interface.isPyBulletConnected():
            raise RuntimeError(f"{self.name}: PyBullet disconnected")
        self.robot.commandJointPosition(self.target_joint_position)
        self.readROSTfs()
        self.setPyBulletCollisionVisualObjectPositionAndOrientation()
        self.visualizeLinks()
        self.publishStaticTransformsToROS()
        self.publishObjectStateTransToROS()

        # run simulation step by step or do nothing
        # (as bullet can run the simulation steps automatically from within)
        self.step()

    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.logwarn(f"{self.name}: User interrupted, quitting...")
        except RuntimeError as error:
            rospy.logerr(f"{self.name}: {error}")
        except rospy.ROSException as error:
            rospy.logerr(f"{self.name}: {error}")
        finally:
            self.shutdown()
            sys.exit(0)

    def shutdown(self):
        self.main_timer.shutdown()
        pybullet_interface.closePyBullet()
        rospy.loginfo(f'{self.name}: shutting down')
        rospy.signal_shutdown(f'{self.name}: shutdown')


# ------------------------------------------------------
#
# Main program
# ------------------------------------------------------


if __name__ == '__main__':
    node = ROSPyBulletInterface()
    node.spin()
