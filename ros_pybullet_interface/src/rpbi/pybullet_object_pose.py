import tf_conversions
import numpy as np

class PybulletObjectPose:

    def __init__(self, pb_obj):

        # Initial setup
        self.pb_obj = pb_obj
        self.config = pb_obj.config.get('object_tf', {})
        self.num_debug_thrown = 0

        # Setup pose
        self.pose = np.zeros(3), np.array([0., 0., 0., 1.])
        if self.tf_specified():
            # user specified /tf -> start listener
            self.pb_obj.timers['pose_listener'] = self.pb_obj.node.Timer(self.dt, self.listener)
        # else: pose is always identity

    @property
    def tf_id(self):
        return self.config.get('tf_id')

    @property
    def hz(self):
        return int(self.config.get('hz', 30))

    @property
    def dt(self):
        return self.pb_obj.node.Duration(1.0/float(self.hz))

    @property
    def max_debug_limit(self):
        return self.hz*2  # i.e. 2 secs worth

    def tf_specified(self):
        return self.tf_id is not None

    def listener(self, event):
        tf = self.pb_obj.node.tf.get_tf_msg('rpbi/world', self.tf_id)
        if tf:
            self.pose = self.pb_obj.node.tf.msg_to_pos_quat(tf)

    def get(self):
        return self.pose  # pose is the tuple pos,quat

    def start_reset_pose(self):
        self.pb_obj.timers['pose_reset'] = self.pb_obj.node.Timer(self.dt, self.reset_pose)

    def reset_pose(self, event):

        # Check that pybullet object has body unique id
        if self.pb_obj.body_unique_id is None:
            if self.num_debug_thrown < self.max_debug_limit:
                self.pb_obj.node.logdebug(f'body unique id for pybullet object {self.pb_obj.name} is None')
                self.num_debug_thrown += 1
                return
            else:
                msg = f'body unique id for pybullet object {self.pb_obj.name} is None max number of iterations without this changing has been reached!'
                self.pb_obj.node.logerr(msg)
                raise RuntimeError(msg)
            
        # Reset num_debug_thrown to 0
        self.num_debug_thrown = 0

        # Update pose
        pos, ori = self.get()
        self.pb_obj.pb.resetBasePositionAndOrientation(self.pb_obj.body_unique_id, pos, ori)
