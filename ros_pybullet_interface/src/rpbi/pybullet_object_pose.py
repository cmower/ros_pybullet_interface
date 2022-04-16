import tf_conversions
import numpy as np

class PybulletObjectPose:

    def __init__(self, pb_obj):
        self.pb_obj = pb_obj
        self.config = pb_obj.config.get('object_tf', {})
        self.base = None

    @property
    def is_static(self):
        return self.config.get('is_static', True)

    @property
    def base_tf_id(self):
        return self.config.get('base_tf_id', 'rpbi/world')

    @property
    def timeout(self):
        return self.config.get('timeout', 5.0)

    @property
    def dt(self):
        hz = self.config.get('hz', 50)
        return self.pb_obj.node.Duration(1.0/float(hz))

    @property
    def offset(self):
        offset = self.config.get('offset', [0.0]*3)
        T = np.eye(4)
        T[:3, 3] = np.array(offset[:3])
        if len(offset) == 6:
            T = tf_conversions.transformations.euler_matrix(offset[3], offset[4], offset[5])
            T[:3, 3] = pos
        elif len(offset) == 7:
            T = tf_conversions.transformations.quaternion_matrix(offset[3:])
            T[:3, 3] = pos
        return T

    @property
    def tf_frame_id(self):
        return self.config.get('tf_frame_id', f'rpbi/{self.pb_obj.name}')

    @property
    def broadcast_tf(self):
        return self.config.get('broadcast_tf', False)

    def get_base_from_tf(self):
        pos, rot = self.pb_obj.node.wait_for_tf('rpbi/world', self.base_tf_id, timeout=self.timeout)
        self.base = self.pb_obj.node.tf.pos_quat_to_matrix(pos, rot)

    def is_defined(self):
        return self.base is not None

    def get(self):
        T = self.offset @ self.base
        pos = T[:3,-1].flatten()
        ori = tf_conversions.transformations.quaternion_from_matrix(T)
        return pos, ori

    def start_resetter(self):
        self.pb_obj.timers['pose_resetter'] = self.pb_obj.node.Timer(self.dt, self._update_pose)

    def _update_pose(self, event):
        if self.pb_obj.body_unique_id is None: return

        # Update base
        msg = self.pb_obj.node.tf.get_tf_msg('rpbi/world', self.base_tf_id)
        if msg is None: return
        self.base = self.pb_obj.node.tf.msg_to_matrix(msg)

        # Reset base position and orientation
        pos, ori = self.get()
        self.pb_obj.pb.resetBasePositionAndOrientation(self.pb_obj.body_unique_id, pos, ori)

    def start_pose_broadcaster(self):
        self.pb_obj.timers['broadcast_pose'] = self.pb_obj.node.Timer(self.dt, self._broadcast_pose)

    def _broadcast_pose(self, event):
        if not isinstance(self.pb_obj.body_unique_id, int): return
        pos, ori = self.pb_obj.pb.getBasePositionAndOrientation(self.pb_obj.body_unique_id)
        self.pb_obj.node.tf.set_tf('rpbi/world', self.tf_frame_id, pos, ori)
