#!/usr/bin/env python3
import os
import sys

import numpy as np
import rospy
import tf2_ros
import tf_conversions
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped

from ros_pybullet_interface import pybullet_interface
from ros_pybullet_interface.utils import loadYAMLConfig, ROOT_DIR
from ros_pybullet_interface.srv import setObjectState, setObjectStateResponse
from ros_pybullet_interface.srv import ManualPybulletSteps, ManualPybulletStepsResponse
from ros_pybullet_interface.srv import setClothConstraintParams, setClothConstraintParamsResponse
# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

ROS_FREQ = 100       # ROS loop sampling frequency
ROS_DT = 1.0/float(ROS_FREQ)
PYBULLET_FREQ = ROS_FREQ  # 256 # PyBullet simulation loop sampling frequency
PYBULLET_DT = 1.0/float(PYBULLET_FREQ)
# listens for joint states on this topic
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'
# publishes joint states on this topic
CURRENT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current'
WORLD_FRAME_ID = 'ros_pybullet_interface/world'


# ------------------------------------------------------------
#  REAL ROBOT
# ------------------------------------------------------------
# test on robot and compare with commit:
# https://github.com/cmower/ros_pybullet_interface/commit/12d603ea46791a7db6a6ac0195dcef6cf7597cf1
# REAL_ROBOT_TARGET_JOINT_STATE_TOPIC = 'joint_states'  # publishes joint states on this topic
# TARGET_JOINT_STATE_TOPIC = REAL_ROBOT_TARGET_JOINT_STATE_TOPIC

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
        self.tfs = {}
        self.dynamic_collisionvisual_objects = []
        self.static_collisionvisual_objects = []
        self.static_collision_objects = []
        self.objects = []

        # list for many robots
        self.robots = []
        self.robots_target_joint_position = []
        self.robots_joint_state_pubs = []
        self.robots_sensor_pubs = []

        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # get the path to this catkin ws
        self.current_dir = ROOT_DIR

        # Get ros parameters
        camera_config_file_name = rospy.get_param('~camera_config')
        robot_file_names = rospy.get_param('~robot_config_file_names', [])
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
        pybullet_interface.initPyBullet(ROS_DT, gravity=grav)
        self.setupPyBulletCamera(camera_config_file_name)

        for file_name in robot_file_names:
            self.setupPyBulletRobot(file_name)

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

        sim_loop_flag = rospy.get_param('~pybullet_sim_self_loop')
        if sim_loop_flag == 0:
            pybullet_interface.updateTimeStep(PYBULLET_DT)
            pybullet_interface.runPyBullet()
            self.step = self._null

        elif sim_loop_flag == 1:
            self.step = self._step

        elif sim_loop_flag == 2:
            self.makePybulletStepsServer()
            self.step = self._null

        else:
            rospy.logerr("ROS parameter pybullet_sim_self_loop has invalid value.")
            self.shutdown()
            sys.exit(0)

        # Setup ros timer to publish sensor readings
        rospy.Timer(self.dur, self.publishPyBulletSensorReadingsToROS)

        # Main pybullet update
        self.main_timer = rospy.Timer(self.dur, self.updatePyBullet)

        # ----------------------------------------------------------------------
        # control the avatar via the points on the right wrist and elbow
        human_body_id = self.robots[-1]['robot'].ID
        human_link_id_shoulder = self.robots[-1]['robot'].link_ids[17]
        human_link_id_elbow = self.robots[-1]['robot'].link_ids[19]
        human_link_id_hand = self.robots[-1]['robot'].link_ids[21]

        shoulder = self.dynamic_collisionvisual_objects[-4]['object'].ID
        elbow = self.dynamic_collisionvisual_objects[-3]['object'].ID
        hand = self.dynamic_collisionvisual_objects[-2]['object'].ID

        pp = pybullet_interface.getPybulletObject_hack()

        constraint_id_shoulder = pp.createConstraint(human_body_id, human_link_id_shoulder, shoulder, -1, pp.JOINT_POINT2POINT,
                                                     [0, 0, 0], [0, 0, 0.0], [0, -0.0, -0.0])
        pp.changeConstraint(constraint_id_shoulder, erp=1.0, maxForce=1500)

        offset = -0.025*0
        constraint_id_elbow = pp.createConstraint(human_body_id, human_link_id_elbow, elbow, -1, pp.JOINT_POINT2POINT,
                                                  [0, 0, 0], [0, 0, 0.0], [0, -0.0, offset])
        pp.changeConstraint(constraint_id_elbow, erp=0.75, maxForce=750)
        constraint_id_hand = pp.createConstraint(human_body_id, human_link_id_hand, hand, -1, pp.JOINT_POINT2POINT,
                                                 [0, 0, 0], [0, 0, 0], [0, -0.0, offset])
        pp.changeConstraint(constraint_id_hand, erp=0.75, maxForce=750)

        rospy.Subscriber("ros_pybullet_interface/zono_info", WrenchStamped, self.moveAndRescaleObj)

        # ----------------------------------------------------------------------
        # load cloth and set constraint between robot and cloth
        # robot_body_id = self.robots[0]['robot'].ID
        # robot_link_id = self.robots[0]['robot'].link_ids[-1]
        #
        # path2cloth = f"{self.current_dir}/robots/cloth/cloth.urdf"
        # self.cid = pybullet_interface.loadCloathAndsetClothConstr(
        #     robot_body_id, robot_link_id, path2cloth)
        #
        # # initiate server for changing params of cloth constraint
        # self.setClothConstrParamServer()

    def _null(self, *args, **kwargs):
        pass

    def _step(self):
        pybullet_interface.stepPyBullet()

    def setupPyBulletCamera(self, file_name):

        # Load camera config
        config = loadYAMLConfig(file_name)

        # Extract data from configuration
        pybullet_interface.setupPyBulletCamera(
            config['distance'],
            config['yaw'],
            config['pitch'],
            config['target_position']
        )

    def setupPyBulletRobot(self, file_name):

        # Load robot configuration
        config = loadYAMLConfig(file_name)

        # get the name of the robot
        robot_name = config['name']

        # Setup robot
        robot = pybullet_interface.PyBulletRobot(config['file_name'])
        self.robots.append({"robot": robot,
                            "robot_name": robot_name}
                           )

        robot.setBasePositionAndOrientation(
            config['base_position'],
            np.deg2rad(config['base_orient_eulerXYZ'])
        )
        qinit = np.deg2rad(config['init_position'])
        robot.setJointPositions(qinit)

        # Loop 2 times to get the position and orientation of the base the robots from the vicon
        for _ in range(2):
            try:
                # Read the position and orientation of the robot from the topic
                trans = rospy.wait_for_message(
                    f"vicon_offset/{robot_name}_frame/{robot_name}_frame", TransformStamped, timeout=2)

                # replaces base_position = config['base_position']
                base_position = [trans.transform.translation.x,
                                 trans.transform.translation.y, trans.transform.translation.z]
                # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
                base_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y,
                                    trans.transform.rotation.z, trans.transform.rotation.w]

                robot.setBasePositionAndOrientation(
                    base_position,
                    tf_conversions.transformations.euler_from_quaternion(base_orient_quat)
                )
                break
            except:
                rospy.logwarn(
                    f"{self.name}: /tf topic does NOT have vicon/<subject_name>/<segment_name>_{robot_name}")

        # Specify target joint position
        self.robots_target_joint_position.append(qinit)

        # Setup ros publishers
        publishers_topic_name = os.path.join(robot_name, CURRENT_JOINT_STATE_TOPIC)
        self.robots_joint_state_pubs.append(
            rospy.Publisher(
                publishers_topic_name,
                JointState,
                queue_size=10
            )
        )

        # Setup ros subscriber
        # find the index of the robot in the list
        robot_list_index = len(self.robots) - 1
        # find corresponding to the robot topic to subscribe to
        subscribers_topic_name = os.path.join(robot_name, TARGET_JOINT_STATE_TOPIC)
        # create subscriber that maps topic to index in the list of robot states
        rospy.Subscriber(
            subscribers_topic_name, JointState, self.readTargetJointStateFromROS, robot_list_index)

        self.robots[-1]['sensor_idx'] = -1
        # Setup sensors
        if 'sensors' in config:

            # Setup sensors in PyBullet and their respective ROS publishers
            for label, sensor_parameters in config['sensors'].items():

                # Setup a joint force torque sensor
                if 'joint_force_torque_sensor' in label:

                    # Extract info
                    joint_index = sensor_parameters['joint_index']
                    label = label.split('__')[1]  # remove joint_force_torque_sensor__

                    # Setup sensor
                    robot.setupJointForceTorqueSensor(label, joint_index)

                    # Setup publisher
                    topic = f'{robot_name}/ros_pybullet_interface/joint_force_torque_sensor/{label}'
                    sensor_pubs = {}
                    sensor_pubs[label] = rospy.Publisher(
                        topic, WrenchStamped, queue_size=10
                    )
                    self.robots_sensor_pubs.append(sensor_pubs)
                    self.robots[-1]['sensor_idx'] = len(self.robots_sensor_pubs) - 1

    def setupPyBulletVisualLinks(self, linkid):
        self.tfs[linkid] = {
            'received': False, 'position': None, 'orientation': None
        }

    def setupPyBulletCollisionObject(self, file_name):

        # Load config
        config = loadYAMLConfig(file_name)

        # Setup collision object
        obj = pybullet_interface.PyBulletCollisionObject(
            config['file_name'],
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
        config = loadYAMLConfig(file_name)

        # Setup visual object
        obj = pybullet_interface.PyBulletVisualObject(
            config['file_name'],
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
            static_col_obj = {}
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
        config = loadYAMLConfig(file_name)

        if 'urdf' in config.keys():
            obj = pybullet_interface.PyBulletObject(
                config['file_name'],
                config['mesh_scale'],
                config['rgba_color'],
                config['base_mass'],
                loadURDF=config['urdf'])
        else:
            # Setup visual object
            obj = pybullet_interface.PyBulletObject(
                config['file_name'],
                config['mesh_scale'],
                config['rgba_color'],
                config['base_mass'],
            )

        obj.changeDynamics(
            -1,  # which is the link index
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
            if not tf['received']:
                continue
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

    def readTargetJointStateFromROS(self, msg, args):
        """
            args indicates the index in the list of robot targets
        """
        self.robots_target_joint_position[args] = msg.position

    def publishPyBulletAllRobotJointStateToROS(self):
        for id_robot, robot_dict in enumerate(self.robots):
            robot = robot_dict['robot']
            states = robot.getActiveJointStates()
            msg = JointState(
                name=[state['name'] for state in states],
                position=[state['position'] for state in states],
                velocity=[state['velocity'] for state in states],
                effort=[state['motor_torque'] for state in states],
            )
            msg.header.stamp = rospy.Time.now()
            self.robots_joint_state_pubs[id_robot].publish(msg)

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
        for id_robot, robot_dict in enumerate(self.robots):
            robot = robot_dict['robot']
            robot_sensor_idx = self.robots[id_robot]['sensor_idx']
            if robot_sensor_idx != -1:
                for reading in robot.getSensorReadings():
                    if reading['type'] == 'joint_force_torque':
                        msg = self.packJointForceTorqueROSMsg(reading['reading'])
                        sensor_pubs = self.robots_sensor_pubs[robot_sensor_idx]
                        sensor_pubs[reading['label']].publish(msg)

    def publishStaticTransformsToROS(self):
        for static_obj in self.static_collision_objects:
            if static_obj['pub_tf']:
                self.tf_broadcaster.sendTransform(
                    packTransformStamped(
                        WORLD_FRAME_ID,
                        f"ros_pybullet_interface/{static_obj['name']}",
                        static_obj['position'],
                        static_obj['orientation']
                    )
                )

    def publishPyBulletAllRobotLinkStateToROS(self):
        for id_robot, robot_dict in enumerate(self.robots):
            robot = robot_dict['robot']
            robot_name = robot_dict['robot_name']
            for state in robot.getLinkStates():
                self.tf_broadcaster.sendTransform(
                    packTransformStamped(
                        WORLD_FRAME_ID,
                        f"{robot_name}/ros_pybullet_interface/robot/{state['label']}",
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
                    f"ros_pybullet_interface/{obj['object_name']}",
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
            if obj['object_name'] == req.obj_name:
                obj_id = obj['object_id']

        if obj_id == None:
            rospy.logwarn(f"{obj_name} was not found... in setObjState()")
            return

        pybullet_interface.setObjectPosOrient(obj_id, req.pos, req.quat)
        pybullet_interface.setObjectVelLinAng(obj_id, req.lin_vel, req.ang_vel)
        rospy.loginfo(f"Returning object position and orientation: {req.pos}, {req.quat}")
        rospy.loginfo(f"Returning object linear and angular velocity: {req.lin_vel}, {req.ang_vel}")
        rospy.loginfo(
            f"Returning object dynamics info: {pybullet_interface.getObjectDynamicsInfo(obj_id)}")

        return setObjectStateResponse("True: Set the state of the object successfully")

    def setObjectStateServer(self):

        s = rospy.Service('set_object_state', setObjectState, self.setObjState)
        rospy.loginfo("Server is ready to set the state of the object.")

    def manualPybulletSteps(self, req):

        try:
            num_steps = req.num_pybullet_steps
            # perform a pybullet step x num_steps
            for _ in range(num_steps):
                self._step()

            srv_success = True
            srv_info = "Successfully made {num_steps} number of pyBullet steps."

        except Exception as error:
            srv_success = False
            srv_info = f"PyBullet stepping FAILED: {error}"

        return ManualPybulletStepsResponse(success=srv_success, info=srv_info)

    def makePybulletStepsServer(self):

        s = rospy.Service('manual_pybullet_steps', ManualPybulletSteps, self.manualPybulletSteps)
        rospy.loginfo("Server is ready to perform manual pybullet steps.")

    def setClothConstrParams(self, req):

        try:
            if len(req.ChildPivotPt) != 3:
                rospy.loginfo(
                    "The length of the vector given to the set_cloth_params srv shoudl be 3")
                raise Exception
            # update the parameters of the cloth constraint
            pybullet_interface.setClothConstrParams(
                self.cid, ChildPivotPt=req.ChildPivotPt, erp=req.erp, maxForce=req.maxForce)
            srv_success = True
            srv_info = "Successfully updated the parameters of the cloth constraint."

        except Exception as error:
            srv_success = False
            srv_info = f"Update of the parameters of the cloth constraint FAILED: {error}"
        return setClothConstraintParamsResponse(success=srv_success, info=srv_info)

    def setClothConstrParamServer(self):
        s = rospy.Service('set_cloth_constraint_parameters',
                          setClothConstraintParams, self.setClothConstrParams)
        rospy.loginfo("Server is ready to set the parameters of the cloth constraint.")

    def updatePyBullet(self, event):
        if not pybullet_interface.isPyBulletConnected():
            raise RuntimeError(f"{self.name}: PyBullet disconnected")
        for id_robot, robot_dict in enumerate(self.robots):
            robot = robot_dict['robot']
            robot.commandJointPosition(self.robots_target_joint_position[id_robot])

        self.readROSTfs()
        self.setPyBulletCollisionVisualObjectPositionAndOrientation()
        self.visualizeLinks()
        self.publishPyBulletAllRobotJointStateToROS()
        self.publishPyBulletAllRobotLinkStateToROS()
        self.publishStaticTransformsToROS()
        self.publishObjectStateTransToROS()

        # run simulation step by step or do nothing
        # (as bullet can run the simulation steps automatically from within)
        # or (as we might want to manually control the rate of steps)
        self.step()

    def moveAndRescaleObj(self, msg):

        position = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        width = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

        pp = pybullet_interface.getPybulletObject_hack()
        zono = self.dynamic_collisionvisual_objects[-1]['object'].ID

        # pos = pybullet_interface.getObjectPosOrient(zono)[0]
        self.setupPyBulletVisualObject2(
            "/home/theo/software/pybullet/py3bullet-ROS-ws/src/ros_pybullet_interface/configs/zonotope.yaml", width, position)
        pp.removeBody(zono)

    def moveAndRescaleObjTest(self):

        pp = pybullet_interface.getPybulletObject_hack()
        zono = self.dynamic_collisionvisual_objects[-1]['object'].ID

        self.counter = self.counter + 1

        scale = 0.1
        if self.counter % 10 == 0:
            pos = pybullet_interface.getObjectPosOrient(zono)[0]
            pp.removeBody(zono)

            if self.counter % 20 == 0:
                scale = scale - 0.05
            else:
                scale = scale + 0.05

            # scale = read from msg
            # pos = read from msg

            self.setupPyBulletVisualObject2(
                "/home/theo/software/pybullet/py3bullet-ROS-ws/src/ros_pybullet_interface/configs/zonotope.yaml", [scale, scale, scale], list(pos))

    def setupPyBulletVisualObject2(self, file_name, scale, pose):

        # Load config
        config = loadYAMLConfig(file_name)

        # Setup visual object
        obj = pybullet_interface.PyBulletVisualObject(
            config['file_name'],
            scale,
            config['rgba_color'],
            pos=pose
        )
        # pybullet_interface.setObjectPosOrient(obj.ID, pos3D=pose)

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
            static_col_obj = {}
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
