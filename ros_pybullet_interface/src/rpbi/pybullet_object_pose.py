import tf_conversions
import numpy as np
from functools import partial

"""

Config documentation
--------------------


        ^
        |
        O -->
         \
always >> \ ^
static     \|
            B -->
           /
          /  << either static or non-static
       ^ /
       |/
       W -->


The world frame (W in above) is always defined in the ROS /tf topic as
rpbi/world. A pybullet object pose is always defined with respect to a
base transformation W<-B and offset B<-O. The offset transform B<-O is
always static, and is defined in the config file - if unset then the
identity transform is used. The base transform W<-B is synced with a
ROS transform frame using the tf2 library - thus, the transform can be
static (is_static=true) or non-static (is_static=false, default). If
unset then the base transform is the identity. An example config is
shown below.


object_tf:
    pose_reset_hz: 100
    offset:
        transform: [0, 0.1, 0, 1.57, 0, -3.14]
        is_deg: false
        hz: 20
    base:
        is_static: true
        tf_id: "base_tf_id_name"
        hz: 20


offset
------

  - transform (list[float], default is [0,0,0])
      The offset transform. If length 3 then assumed to be
      position(3), if length is 6 then assumed to be position(3)+euler
      angles(3) (is_deg can be used to specify if the rotation part is
      given in degrees [is_deg=true] or radians [is_deg=false]), if
      length is 7 then assumed to be position(3)+quaternion(4).

  - is_deg (bool, default is false)
      If true then when the transform is length 6 (i.e
      position(3)+euler angles(3)) the rotation part is assumed to be
      given in degrees, otherwise radians. If the transform is given
      as either a position(3) or position(3)+quaterion(4) then is_deg
      is ignored.

  - hz (float/int, default is 0)
      The frequency that the offset transform is broadcast to ROS. If
      hz is 0, then the transform is not broadcast.

base
----

  - is_static (bool)

  - tf_id (string, default is rpbi/{pb_obj_name})

  - hz (int)


"""

class PoseConfig:

    def __init__(self, pb_obj, config):
        self.pb_obj = pb_obj
        self.d = config

class OffsetConfig(PoseConfig):

    def transform(self):
        transform = np.array(self.d.get('transform', [0.0]*3))
        T = np.eye(4)
        T[:3, 3] = offset[:3]
        if offset.shape[0] == 6:
            if self.is_deg:
                offset[3:] = np.deg2rad(offset[3:])
            T[:3, :3] = tf_conversions.transformations.euler_matrix(offset[3], offset[4], offset[5])[:3, :3]
        elif offset.shape[0] == 7:
            T[:3, :3] = tf_conversions.transformations.quaternion_matrix(offset[3:])[:3, :3]
        return T

    @property
    def is_deg(self):
        return self.d.get('is_deg', False)

    @property
    def hz(self):
        return self.d.get('hz', 0)

class BaseConfig(PoseConfig):

    @property
    def is_static(self):
        return self.d.get('is_static', False)

    @property
    def tf_id(self):
        return self.d.get('tf_id', f'rpbi/{self.pb_obj.name}')

    @property
    def hz(self):
        return self.d.get('hz', 30)

class Config:

    def __init__(self, pb_obj):
        config = pb_obj.config.get('object_tf', {})
        self.config = config
        self.offset = OffsetConfig(pb_obj, config.get('offset', {}))
        self.base = BaseConfig(pb_obj, config.get('base', {}))

    @property
    def pose_reset_hz(self):
        return self.config.get('pose_reset_hz', 30)

class PybulletObjectPose:

    def __init__(self, pb_obj):

        # Setup
        self.pb_obj = pb_obj
        self.config = Config(pb_obj)

        # Setup offset
        self.offset_tf_id = f'{self.config.base.tf_id}_offset'
        self.offset = self.config.offset.transform()

        # Start offset transform broadcaster
        if self.config.offset.hz>0:
            dt = self.pb_obj.node.Duration(1.0/float(self.config.offset.hz))
            handle = partial(self._broadcast_transform, parent_id=self.config.base.tf_id, child_id=self.offset_tf_id, transform=self.offset)
            self.pb_obj.timers['offset_broadcaster'] = self.pb_obj.node.Timer(dt, handle)

        # Get base
        self.base = np.eye(4)  # initialize to identity
        if self.config.base.is_static:

            pos, quat = self.pb_obj.node.wait_for_tf('rpbi/world', self.tf_id, timeout=5.)
            self.base = self.pb_obj.node.tf.pos_quat_to_matrix(pos, quat)

            if self.config.base.hz > 0:
                handle = partial(self._broadcast_transform, parent_id='rpbi/world', child_id=self.config.base.tf_id, transform=self.base)
                dt = self.pb_obj.node.Duration(1.0/float(self.config.base.hz))
                self.pb_obj.timers['base_broadcaster'] = self.pb_obj.node.Timer(dt, handle)

        else:
            if self.config.base.hz > 0:
                dt = self.pb_obj.node.Duration(1.0/float(self.config.base.hz))
                self.pb_obj.timers['base_listener'] = self.pb_obj.node.Timer(dt, self._listen_to_base_transform)
            else:
                raise ValueError("hz in base config must be greater than 0 for a non-static frame")

    def _broadcast_transform(self, event, parent_id, child_id, transform):
        self.pb_obj.node.tf.set_matrix(parent_id, child_id, transform)

    def _listen_to_base_transform(self, event):
        msg = self.pb_obj.node.tf.get_tf_msg('rpbi/world', self.config.base.tf_id)
        if msg is None: return
        self.base = self.pb_obj.node.tf.msg_to_matrix(msg)

    def get(self):
        T = self.offset @ self.base
        pos = T[:3, 3].flatten()
        ori = tf_conversions.transformations.quaternion_from_matrix(T)
        return pos, ori

    def start_reset_base_position_and_orientation(self):
        dt = self.pb_obj.node.Duration(1.0/float(self.config.pose_reset_hz))
        self.pb_obj.timers['pose_resetter'] = self.pb_obj.node.Timer(dt, self._update_pose)

    def _update_pose(self, event):
        if self.pb_obj.body_unique_id is None: return
        pos, ori = self.get()
        self.pb_obj.pb.resetBasePositionAndOrientation(self.pb_obj.body_unique_id, pos, ori)
