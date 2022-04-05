from sensor_msgs.msg import JointState
from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem

class Ik:

    def __init__(self, pb_obj, joints):
        self.pb_obj = pb_obj
        self.joints = joints

        if self.start_ik_callback:
            topic = f'rpbi/{self.pb_obj.name}/ik'
            self.pb_obj.node.Subscriber(topic, CalculateInverseKinematicsProblem, self.callback)

    @property
    def start_ik_callback(self):
        return self.pb_obj.config.get('start_ik_callback', False)

    def callback(self, problem):
        self.joints.joint_state_target_callback(self.solve(problem))

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
        if problem.resetPoses:
            kwargs['resetPoses'] = problem.resetPoses
        if problem.jointDamping:
            kwargs['jointDamping'] = problem.jointDamping
        if problem.solver:
            kwargs['solver'] = problem.solver
        # if problem.currentPosition:
        #     kwargs['currentPosition'] = problem.currentPosition
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

        if (problem.dt>0.0) and problem.currentPosition:
            dt = problem.dt
            solution.velocity = [(p-c)/dt for p, c in zip(positions, problem.currentPosition)]

        return solution
