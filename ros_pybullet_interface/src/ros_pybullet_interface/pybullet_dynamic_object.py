import numpy as np
import tf_conversions
from .pybullet_object import PybulletObject
from .utils import TimeoutExceeded

class PybulletDynamicObject(PybulletObject):

    def init(self):

        # Setup variables
        self.base_position = None
        self.base_orientation = None
        self.dynamic_object_tf_broadcast_frequency = None

        # Setup object
        self.get_frame_offset()

        # Get object base tf frame (optional, default to world frame)
        self.object_base_tf_frame_id = self.config.get('object_base_tf_frame_id', 'rpbi/world')

        if self.object_base_tf_frame_id != 'rpbi/world':
            # base frame is not world frame -> listen to tf frames

            # Get timer timeout if static
            if self.object_base_tf_frame_is_static:
                self.object_base_tf_frame_listener_timeout = self.config.get('object_base_tf_frame_listener_timeout', 2)

            # Start looping: collect object tf
            got_tf = False
            self.object_base_tf_frame_listener_timer_start_time = self.node.time_now()
            while not got_tf:
                pos_base, rot_base = self.node.tf.get_tf('rpbi/world', self.object_base_tf_frame_id)
                if pos_base is not None:
                    got_tf = True
                else:
                    # Compute time since the callback started
                    time_since_start = (self.node.time_now() - self.object_base_tf_frame_listener_timer_start_time).to_sec()

                    # Check if timeout exceeded
                    if (time_since_start > self.object_base_tf_frame_listener_timeout):
                        raise TimeoutExceeded(f'reached timeout ({self.object_base_tf_frame_listener_timeout} secs) to retrieve frame {self.object_base_tf_frame_id}!')

        else:

            # Base is world, so use zero transform
            pos_base = np.zeros(3)
            rot_base = np.array([0, 0, 0, 1])

        # Get base position/orientation
        T = self.offset_T @ self.node.tf.position_and_quaternion_to_matrix(pos_base, rot_base)
        self.base_position = T[:3,-1].flatten()
        self.base_orientation = tf_conversions.transformations.quaternion_from_matrix(T)

        # Initialize the dynamic object in Pybullet
        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])
        self.base_collision_shape_index = self.create_collision_shape(self.config['createCollisionShape'])
        self.body_unique_id = self.pb.createMultiBody(
            baseMass=self.config['baseMass'],
            baseVisualShapeIndex=self.base_visual_shape_index,
            baseCollisionShapeIndex=self.base_collision_shape_index,
            basePosition=self.base_position,
            baseOrientation=self.base_orientation,
        )

        # Get name of transform frame (optional)
        self.tf_frame_id = self.config.get('tf_frame_id', f'rpbi/{self.name}')

        # Set velocity
        self.pb.resetBaseVelocity(
            self.body_unique_id,
            self.config.get('linearVelocity', [0.0]*3),
            np.deg2rad(self.config.get('angularVelocity', [0.0]*3)),
        )

        # Start broadcasting object tf (optional)
        if self.config.get('broadcast_object_tf_frame', False):

            # Get tf broadcaster frequency
            self.dynamic_object_tf_broadcast_frequency = self.config.get('dynamic_object_tf_broadcast_frequency', 50)
            dynamic_object_tf_broadcast_dt = 1.0/float(self.dynamic_object_tf_broadcast_frequency)

            # Start timer
            self.node.Timer(self.node.Duration(dynamic_object_tf_broadcast_dt), self.broadcast_dynamic_object_tf)

    def broadcast_dynamic_object_tf(self, event):
        pos, ori = self.pb.getBasePositionAndOrientation(self.body_unique_id)
        self.node.tf.set_tf('rpbi/world', self.tf_frame_id, pos, ori)
