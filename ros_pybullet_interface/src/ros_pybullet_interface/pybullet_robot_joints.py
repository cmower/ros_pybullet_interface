from ros_pybullet_interface.msg import JointInfo

class Joint:

    joint_types_str = ["JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"]

    def __init__(self, info):
        self.info = info

    @property
    def jointIndex(self):
        return self.info[0]

    @property
    def jointName(self):
        return self.info[1].decode('utf-8')

    @property
    def jointType(self):
        return self.info[2]

    @property
    def jointTypeStr(self):
        return Joint.joint_types_str[self.jointType]

    @property
    def qIndex(self):
        return self.info[3]

    @property
    def uIndex(self):
        return self.info[4]

    @property
    def flags(self):
        return self.info[5]

    @property
    def jointDamping(self):
        return self.info[6]

    @property
    def jointFriction(self):
        return self.info[7]

    @property
    def jointLowerLimit(self):
        return self.info[8]

    @property
    def jointUpperLimit(self):
        return self.info[9]

    @property
    def jointMaxForce(self):
        return self.info[10]

    @property
    def jointMaxVelocity(self):
        return self.info[11]

    @property
    def linkName(self):
        return self.info[12].decode('utf-8')

    @property
    def jointAxis(self):
        return self.info[13]

    @property
    def parentFramePos(self):
        return self.info[14]

    @property
    def parentFrameOrn(self):
        return self.info[15]

    @property
    def parentIndex(self):
        return self.info[16]

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
