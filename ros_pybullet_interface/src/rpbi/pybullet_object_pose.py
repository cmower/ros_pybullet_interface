import tf_conversions
import numpy as np

class PybulletObjectPose:

    def __init__(self, pb_obj):

        # Initial setup
        self.pb_obj = pb_obj
        self.config = pb_obj.config.get('object_tf', {})

        # Setup pose
        self.pose = np.zeros(3), np.array([0., 0., 0., 1.])
        if self.tf_id:
            # user specified /tf -> start listener
            dt = self.pb_obj.node.Duration(1.0/float(self.hz))
            self.pb_obj.timers['pose_listener'] = self.pb_obj.node.Timer(dt, self.listener)
        # else: pose is always identity

    @property
    def tf_id(self):
        return self.config.get('tf_id')

    @property
    def hz(self):
        return self.config.get('hz', 30)

    def listener(self, event):
        tf = self.pb_obj.node.tf.get_tf_msg('rpbi/world', self.tf_id)
        if tf:
            self.pose = self.pb_obj.node.tf.msg_to_pos_quat(tf)

    def get(self):
        return self.pose  # pose is the tuple pos,quat

    # def start_resetter(self):
    #     self.pb_obj.timers['pose_resetter'] = self.pb_obj.node.Timer(self.dt, self._update_pose)

    # def _update_pose(self, event):
    #     if self.pb_obj.body_unique_id is None: return

    #     # Update base
    #     msg = self.pb_obj.node.tf.get_tf_msg('rpbi/world', self.base_tf_id)
    #     if msg is None: return
    #     self.base = self.pb_obj.node.tf.msg_to_matrix(msg)

    #     # Reset base position and orientation
    #     pos, ori = self.get()
    #     self.pb_obj.pb.resetBasePositionAndOrientation(self.pb_obj.body_unique_id, pos, ori)

    # def start_pose_broadcaster(self):
    #     self.pb_obj.timers['broadcast_pose'] = self.pb_obj.node.Timer(self.dt, self._broadcast_pose)

    # def _broadcast_pose(self, event):
    #     if not isinstance(self.pb_obj.body_unique_id, int): return
    #     pos, ori = self.pb_obj.pb.getBasePositionAndOrientation(self.pb_obj.body_unique_id)
    #     self.pb_obj.node.tf.set_tf('rpbi/world', self.tf_frame_id, pos, ori)
