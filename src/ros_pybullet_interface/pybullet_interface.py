import tf_conversions
import pybullet
import math
import numpy as np


# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

VISUAL_FRAME_LINE_WIDTH = 2

# ------------------------------------------------------
#
# Methods
# ------------------------------------------------------

def initPyBullet(time_step):
    pybullet.connect(pybullet.GUI)
    pybullet.resetSimulation()
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0) # this removes the side menus
    pybullet.setTimeStep(time_step)
    pybullet.setGravity(0,0,-9.8)

def setupPyBulletCamera(distance, yaw, pitch, target_position):
    pybullet.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=target_position
    )

def stepPyBullet():
    pybullet.stepSimulation()

def runPyBullet():
    pybullet.setRealTimeSimulation(1)

def stopPyBullet():
    pybullet.setRealTimeSimulation(0)

def isPyBulletConnected():
    return pybullet.isConnected()

def closePyBullet():
    if pybullet.isConnected():
        pybullet.disconnect()

def updateTimeStep(dt):
    pybullet.setTimeStep(dt)

def asQuaternion(orientation):
    """Ensure orientation is a quaternion."""
    if len(orientation) == 3:
        orientation = tf_conversions.transformations.quaternion_from_euler(
            orientation[0],
            orientation[1],
            orientation[2]
        )
    return orientation

def visualizeFrameInWorld(position, orientation, scale=1.0):
    """Visualize frame, assumes position/orientation defined wrt world."""
    p = np.asarray(position, dtype=float)
    R = scale * tf_conversions.transformations.quaternion_matrix(
        np.asarray(asQuaternion(orientation), dtype=float)
    )
    pybullet.addUserDebugLine(
        p, p+R[:3, 0], [1, 0, 0], lineWidth=VISUAL_FRAME_LINE_WIDTH
    )
    pybullet.addUserDebugLine(
        p, p+R[:3, 1], [0, 1, 0], lineWidth=VISUAL_FRAME_LINE_WIDTH
    )
    pybullet.addUserDebugLine(
        p, p+R[:3, 2], [0, 0, 1], lineWidth=VISUAL_FRAME_LINE_WIDTH
    )

# ------------------------------------------------------
#
# Classes
# ------------------------------------------------------

class PyBulletObject:

    def __init__(self, file_name, mesh_scale, rgba_color, base_mass):
        self.loadMeshVisual(file_name, mesh_scale, rgba_color)
        self.loadMeshCollision(file_name, mesh_scale)
        self.createMultiBody(base_mass)

    def setBasePositionAndOrientation(self, position, orientation):
        '''Note: if len(orientation) is 3 then it is treated as euler angles, otherwise
        we expect a quaternion.'''

        orientation = np.deg2rad(np.array(orientation))

        # Ensure orientation is a quaternion
        orientation = asQuaternion(orientation)

        # Reset base position/orientation
        pybullet.resetBasePositionAndOrientation(
            self.ID, position, orientation
        )

    def visualizeLink(self, link_index, scale=1.0):
        pybullet.addUserDebugLine(
            [0,0,0],
            [scale,0,0],
            [1,0,0],
            parentObjectUniqueId=self.ID,
            lineWidth=2,
            parentLinkIndex=link_index
        )
        pybullet.addUserDebugLine(
            [0,0,0],
            [0,scale,0],
            [0,1,0],
            parentObjectUniqueId=self.ID,
            lineWidth=2,
            parentLinkIndex=link_index
        )
        pybullet.addUserDebugLine(
            [0,0,0],
            [0,0,scale],
            [0,0,1],
            parentObjectUniqueId=self.ID,
            lineWidth=2,
            parentLinkIndex=link_index
        )

    def changeDynamics(self,
                       link_index,
                       lateral_friction,
                       spinning_friction,
                       rolling_friction,
                       restitution,
                       linear_damping,
                       angular_damping,
                       contact_stiffness,
                       contact_damping):
        pybullet.changeDynamics(
            bodyUniqueId=self.ID,
            linkIndex=link_index,
            lateralFriction=lateral_friction,
            spinningFriction=spinning_friction,
            rollingFriction=rolling_friction,
            restitution=restitution,
            linearDamping=linear_damping,
            angularDamping=angular_damping,
            contactStiffness=contact_stiffness,
            contactDamping=contact_damping,
        )

    def loadURDF(self, file_name):
        self.ID = pybullet.loadURDF(file_name, useFixedBase=True) # only support fixed base robots

    def loadMeshVisual(self, file_name, mesh_scale, rgba_color):
        self.visual_ID = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName=file_name,
            meshScale=mesh_scale,
            rgbaColor=rgba_color
        )

    def loadMeshCollision(self, file_name, mesh_scale):
        self.collision_ID = pybullet.createCollisionShape(
            pybullet.GEOM_MESH,
            fileName=file_name,
            meshScale=mesh_scale
        )

    def createCollisionMultiBody(self, base_mass):
        self.ID = pybullet.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=self.collision_ID,
            baseVisualShapeIndex=self.visual_ID
        )

    def createVisualMultiBody(self):
        self.ID = pybullet.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=self.visual_ID
        )

    def createMultiBody(self, base_mass):
        self.ID = pybullet.createMultiBody(
            baseMass=base_mass,
            baseInertialFramePosition=[0, 0, 0],
            baseCollisionShapeIndex=self.collision_ID,
            baseVisualShapeIndex=self.visual_ID
        )


class PyBulletVisualSphere(PyBulletObject):

    def __init__(self, radius, rgba_color):
        visual_id = pybullet.createVisualShape(
            pybullet.GEOM_SPHERE,
            radius=radius,
            rgbaColor=rgba_color
        )
        self.ID = pybullet.createMultiBody(
            baseVisualShapeIndex=visual_id
        )

class PyBulletCollisionObject(PyBulletObject):

    def __init__(self, file_name, mesh_scale, rgba_color, base_mass):
        self.loadMeshVisual(file_name, mesh_scale, rgba_color)
        self.loadMeshCollision(file_name, mesh_scale)
        self.createCollisionMultiBody(base_mass)

    def getPosition(self):
        return pybullet.getLinkState(self.ID, -1, computeForwardKinematics=1)[4]

    def getOrientation(self):
        return pybullet.getLinkState(self.ID, -1, computeForwardKinematics=1)[5]

class PyBulletVisualObject(PyBulletObject):

    def __init__(self, file_name, mesh_scale, rgba_color):
        self.loadMeshVisual(file_name, mesh_scale, rgba_color)
        self.createVisualMultiBody()

    def getPosition(self):
        return pybullet.getLinkState(self.ID, -1, computeForwardKinematics=1)[4]

    def getOrientation(self):
        return pybullet.getLinkState(self.ID, -1, computeForwardKinematics=1)[5]

class PyBulletRobot(PyBulletObject):

    ACTIVE_JOINT_TYPES = {pybullet.JOINT_REVOLUTE, pybullet.JOINT_PRISMATIC}
    SENSOR_TYPE_TO_INDEX_MAPPING = {
        'joint_force_torque': 2
    }

    def __init__(self, file_name):

        # Setup
        self.loadURDF(file_name)
        self.num_joints = pybullet.getNumJoints(self.ID)
        self.joint_ids = []
        self.link_ids = []
        self.link_names = []
        self.active_joint_ids = []
        self.active_joint_names = []
        self.sensors = []

        # Iterate over joints, extract all link names, and information re
        # active joints.
        for i in range(self.num_joints):
            info = pybullet.getJointInfo(self.ID, i)
            self.joint_ids.append(info[0])
            self.link_ids.append(info[16])
            self.link_names.append(info[12].decode('utf-8'))

            if info[2] in self.ACTIVE_JOINT_TYPES:
                self.active_joint_ids.append(info[0])
                self.active_joint_names.append(info[1].decode('utf-8'))

        self.num_active_joints = len(self.active_joint_ids)

    def setJointPositions(self, position):
        for i, joint_id in enumerate(self.active_joint_ids):
            pybullet.resetJointState(self.ID, joint_id, position[i])

    def getActiveJointStates(self):
        states = [None]*self.num_active_joints
        for i, state in enumerate(pybullet.getJointStates(self.ID, self.active_joint_ids)):
            states[i] = {
                'name': self.active_joint_names[i],
                'position': state[0],
                'velocity': state[1],
                'motor_torque': state[3],
            }
        return states

    def getLinkStates(self):
        states = [None]*self.num_joints
        pb_states = pybullet.getLinkStates(self.ID, self.joint_ids, computeForwardKinematics=1)
        for i in range(self.num_joints):
            states[i] = {
                'label': self.link_names[i],
                'position': pb_states[i][4],
                'orientation': pb_states[i][5],
            }
        return states

    def commandJointPosition(self, target_position):
        pybullet.setJointMotorControlArray(
            self.ID,
            self.active_joint_ids,
            pybullet.POSITION_CONTROL,
            targetPositions=target_position,
        )

    def setupJointForceTorqueSensor(self, label, joint_index):
        pybullet.enableJointForceTorqueSensor(self.ID, joint_index, enableSensor=True)
        self.sensors.append({
            'type': 'joint_force_torque',
            'label': label,
            'joint_index': joint_index,
        })

    def getSensorReadings(self):
        readings = [None]*len(self.sensors)
        for i, sensor in enumerate(self.sensors):
            sensor_idx = self.SENSOR_TYPE_TO_INDEX_MAPPING[sensor['type']]
            reading = pybullet.getJointState(self.ID, sensor['joint_index'])[sensor_idx]
            readings[i] = {
                'label': sensor['label'],
                'type': sensor['type'],
                'reading': reading,
            }
        return readings
