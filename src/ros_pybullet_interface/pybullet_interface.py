import tf_conversions
import pybullet
import math
import numpy as np
from .utils import replacePackage


# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

VISUAL_FRAME_LINE_WIDTH = 2

# ------------------------------------------------------
#
# Methods
# ------------------------------------------------------

def initPyBullet(time_step, gravity=[0, 0, -9.81]):
    pybullet.connect(pybullet.GUI)
    # pybullet.connect(pybullet.DIRECT) # --- this command will disable the visualization
    pybullet.resetSimulation()
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0) # this removes the side menus
    pybullet.setTimeStep(time_step)
    pybullet.setGravity(gravity[0],gravity[1],gravity[2])

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


def getPybulletObject_hack():
    """
    This function should be used only for prototyping reasons
    and just for temporary convenience.
    It is called hack! to remember that cannot be used in release code!
    """
    return pybullet

def setObjectPosOrient(obj_id, pos3D=None, quat=None):

    if pos3D!=None:
        p3D, orient4D = pybullet.getBasePositionAndOrientation(obj_id)
        pybullet.resetBasePositionAndOrientation(obj_id, pos3D, orient4D)

    if quat!=None:
        p3D, orient4D = pybullet.getBasePositionAndOrientation(obj_id)
        pybullet.resetBasePositionAndOrientation(obj_id, p3D, quat)

def setObjectVelLinAng(obj_id, lin_vel3D, ang_vel3D):

    if lin_vel3D!=None:
        pybullet.resetBaseVelocity(obj_id, linearVelocity=lin_vel3D)

    if ang_vel3D!=None:
        pybullet.resetBaseVelocity(obj_id, angularVelocity=ang_vel3D)

def getObjectPosOrient(obj_id):
    return pybullet.getBasePositionAndOrientation(obj_id)

def getObjectDynamicsInfo(obj_id):
    return pybullet.getDynamicsInfo(obj_id, -1)

def getObjectLinAngVel(obj_id):
    return pybullet.getBaseVelocity(obj_id)

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

    def __init__(self, file_name, mesh_scale, rgba_color, base_mass, loadURDF=None):

        if loadURDF==None:
            file_name = replacePackage(file_name)
            self.loadMeshVisual(file_name, mesh_scale, rgba_color)
            self.loadMeshCollision(file_name, mesh_scale)
            self.createMultiBody(base_mass)
        else:
            file_name = replacePackage(loadURDF)
            self.loadURDF(file_name, fixed_base=False)


    def setBasePositionAndOrientation(self, position, orientation):
        '''Note: if len(orientation) is 3 then it is treated as euler angles, otherwise
        we expect a quaternion.'''

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
                       contact_damping,
                       localInertiaDiagonal):
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
            localInertiaDiagonal=localInertiaDiagonal,
        )

    def loadURDF(self, file_name, fixed_base=True):
        file_name = replacePackage(file_name)
        self.ID = pybullet.loadURDF(file_name, useFixedBase=fixed_base) # only support fixed base robots

    def loadMeshVisual(self, file_name, mesh_scale, rgba_color):
        file_name = replacePackage(file_name)
        self.visual_ID = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName=file_name,
            meshScale=mesh_scale,
            rgbaColor=rgba_color
        )

    def loadMeshCollision(self, file_name, mesh_scale):
        file_name = replacePackage(file_name)
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

    def getObjectID(self):
        return self.ID


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
        file_name = replacePackage(file_name)
        self.loadMeshVisual(file_name, mesh_scale, rgba_color)
        self.loadMeshCollision(file_name, mesh_scale)
        self.createCollisionMultiBody(base_mass)

    def getPosition(self):
        return pybullet.getLinkState(self.ID, -1, computeForwardKinematics=1)[4]

    def getOrientation(self):
        return pybullet.getLinkState(self.ID, -1, computeForwardKinematics=1)[5]

class PyBulletVisualObject(PyBulletObject):

    def __init__(self, file_name, mesh_scale, rgba_color):
        file_name = replacePackage(file_name)
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
        # one for each joint and one for the base
        states = [None]*(self.num_joints+1)
        # inlcude info about the pose of the base of the robot
        base_pose = pybullet.getBasePositionAndOrientation(self.ID)
        states[0] = {
            'label': "robot_base",
            'position': base_pose[0],
            'orientation': base_pose[1],
        }
        # inlcude info about the pose of all the links of the robot
        pb_states = pybullet.getLinkStates(self.ID, self.joint_ids, computeForwardKinematics=1)
        for i in range(self.num_joints):
            states[i+1] = {
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
            positionGains = [0.8] * len(target_position),
            velocityGains = [1.5] * len(target_position),
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
