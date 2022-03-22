from .pybullet_robot_joints import Joints
from .pybullet_robot_links import Links
from .pybullet_robot_ik import Ik
from .pybullet_robot_urdf import URDF
from .pybullet_object import PybulletObject

from ros_pybullet_interface.srv import RobotInfo, RobotInfoResponse
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateResponse
from ros_pybullet_interface.srv import CalculateInverseKinematics, CalculateInverseKinematicsResponse
from ros_pybullet_interface.srv import ResetEffState, ResetEffStateResponse

class PybulletRobot(PybulletObject):

    """Interface for robots in Pybullet (simulated and visualizations). Note, visualized robots do not interact with objects in Pybullet."""

    def init(self):

        # Setup robot
        self.urdf = URDF(self)
        self.body_unique_id = self.urdf.load()
        self.joints = Joints(self)
        self.links = Links(self, self.joints, self.urdf)
        self.ik = Ik(self, self.joints)

        # Setup services
        self.srvs['robot_info'] = self.node.Service(f'rpbi/{self.name}/robot_info', RobotInfo, self.service_robot_info)
        if not self.is_visual_robot:
            self.srvs['move_to_joint_state'] = self.node.Service(f'rpbi/{self.name}/move_to_joint_state', ResetJointState, self.service_move_to_joint_state)
            self.srvs['move_to_eff_state'] = self.node.Service(f'rpbi/{self.name}/move_to_eff_state', ResetEffState, self.service_move_eff_to_state)
        self.srvs['ik'] = self.node.Service(f'rpbi/{self.name}/ik', CalculateInverseKinematics, self.service_ik)

    @property
    def is_visual_robot(self):
        return self.config.get('is_visual_robot', False)

    def service_robot_info(self, req):
        return RobotInfoResponse(
            robot_name=self.name,
            root_link_name=self.links.root_link_name,
            bodyUniqueId=self.body_unique_id,
            numJoints=self.joints.num_joints,
            numDof=self.joints.ndof,
            joint_info=[j.joint_info_msg for j in self.joints],
        )

    def service_move_to_joint_state(self, req):

        if self.joints.control_mode not in {self.pb.POSITION_CONTROL, self.pb.VELOCITY_CONTROL}:
            success = False
            message = f'robot in {self.joints.control_mode} control mode, currently this service only supports POSITION_CONTROL/VELOCITY_CONTROL modes!'
            self.node.logerr(message)
            return ResetJointStateResponse(message=message, success=success)

        # Setup
        success = True
        message = 'successfully moved robot to joint state'

        # Move joints
        try:
            self.joints.move_to_joint_state(req.joint_state, req.duration)
        except Exception as err:
            success = False
            message = 'failed to move robot to target joint state, exception: ' + str(err)

        return ResetJointStateResponse(message=message, success=success)

    def service_ik(self, req):
        resp = CalculateInverseKinematicsResponse(success=True, message='solved ik')
        try:
            resp.solution = self.ik.solve(req.problem)
        except Exception as err:
            resp.success = False
            resp.message = 'failed to solve ik, exception: ' + str(err)
        if resp.success:
            self.node.loginfo(resp.message)
        else:
            self.node.logerr(resp.message)
        return resp

    def service_move_eff_to_state(self, req):

        # Setup
        success = True
        message = 'moved robot to target eff position'

        # Solve ik
        try:
            target_joint_state = self.ik.solve(req.problem)
        except Exception as err:
            success = False
            message = 'failed to solve ik, exception: ' + str(err)
            self.node.logerr(message)
            return ResetEffStateResponse(success=success, message=message)

        # Move robot
        try:
            self.joints.move_to_joint_state(target_joint_state, req.duration)
        except Exception as err:
            success = False
            message = 'failed to move robot to target eff state, exception: ' + str(err)
            self.node.logerr(message)
            return ResetEffStateResponse(success=success, message=message)

        self.node.loginfo(message)
        return ResetEffStateResponse(success=success, message=message)
