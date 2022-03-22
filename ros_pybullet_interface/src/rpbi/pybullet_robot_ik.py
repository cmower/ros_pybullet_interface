from sensor_msgs.msg import JointState

class Ik:

    def __init__(self, pb_obj, joints):
        self.pb_obj = pb_obj
        self.joints = joints

    def solve(self, problem):

        # Setup arguments for pybullet.calculateInverseKinematics
        args = [
            self.pb_obj.body_unique_id,
            self.joints.link_names.index(problem.link_name),
            problem.targetPosition[:3]
        ]

        # Setup keyword arguments for pybullet.calculateInverseKinematics
        kwargs = {}
        if problem.targetOrientation:
            kwargs['targetOrientation'] = problem.targetOrientation
        if problem.lowerLimits:
            kwargs['lowerLimits'] = problem.lowerLimits
        if problem.upperLimits:
            kwargs['upperLimits'] = problem.upperLimits
        if problem.jointRanges:
            kwargs['jointRanges'] = problem.jointRanges
        if problem.restPoses:
            kwargs['restPoses'] = problem.restPoses
        if problem.jointDamping:
            kwargs['jointDamping'] = problem.jointDamping
        if problem.solver:
            kwargs['solver'] = problem.solver
        if problem.currentPosition:
            kwargs['currentPosition'] = problem.currentPosition
        if problem.maxNumIterations > 0:
            kwargs['maxNumIterations'] = problem.maxNumIterations
        if problem.residualThreshold:
            kwargs['residualThreshold'] = problem.residualThreshold

        # Solve the problem
        positions = self.pb_obj.pb.calculateInverseKinematics(*args, **kwargs)

        # Pack solution
        names = [
            joint.jointName for joint in self.joints
            if not joint.is_fixed()
        ]
        solution = JointState(name=names, position=positions)
        solution.header.stamp = self.pb_obj.node.time_now()

        return solution
