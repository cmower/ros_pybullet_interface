
class Joint:

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
