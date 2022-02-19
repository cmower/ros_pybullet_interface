from ros_pybullet_interface.msg import JointInfo

class Joint:

    joint_types_str = ["JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"]

    def __init__(self, info):
        self.info = info

        # Extract data
        self.jointIndex = info[0]
        self.jointName = info[1].decode('utf-8')
        self.jointType = info[2]
        self.jointTypeStr = Joint.joint_types_str[info[2]]
        self.qIndex = info[3]
        self.uIndex = info[4]
        self.flags = info[5]
        self.jointDamping = info[6]
        self.jointFriction = info[7]
        self.jointLowerLimit = info[8]
        self.jointUpperLimit = info[9]
        self.jointMaxForce = info[10]
        self.jointMaxVelocity = info[11]
        self.linkName = info[12].decode('utf-8')
        self.jointAxis = info[13]
        self.parentFramePos = info[14]
        self.parentFrameOrn = info[15]
        self.parentIndex = info[16]

    def as_ros_msg(self):
        msg = JointInfo()
        msg.jointIndex = self.jointIndex
        msg.jointName = self.jointName
        msg.jointType = self.jointType
        msg.jointTypeStr = self.jointTypeStr
        msg.qIndex = self.qIndex
        msg.uIndex = self.uIndex
        msg.flags = self.flags
        msg.jointDamping = self.jointDamping
        msg.jointFriction = self.jointFriction
        msg.jointLowerLimit = self.jointLowerLimit
        msg.jointUpperLimit = self.jointUpperLimit
        msg.jointMaxForce = self.jointMaxForce
        msg.jointMaxVelocity = self.jointMaxVelocity
        msg.linkName = self.linkName
        msg.jointAxis.x = self.jointAxis[0]
        msg.jointAxis.y = self.jointAxis[1]
        msg.jointAxis.z = self.jointAxis[2]
        msg.parentFramePos.x = self.parentFramePos[0]
        msg.parentFramePos.y = self.parentFramePos[1]
        msg.parentFramePos.z = self.parentFramePos[2]
        msg.parentFrameOrn.x = self.parentFrameOrn[0]
        msg.parentFrameOrn.y = self.parentFrameOrn[1]
        msg.parentFrameOrn.z = self.parentFrameOrn[2]
        msg.parentFrameOrn.w = self.parentFrameOrn[3]
        msg.parentIndex = self.parentIndex
        return msg
