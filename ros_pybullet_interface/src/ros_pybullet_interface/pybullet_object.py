import tf_conversions
import numpy as np
from .config import replace_package


class PybulletObject:


    def __init__(self, pb, node, config):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Init config
        self.config = config
        self.name = self.config['name']
        del self.config['name']

        # Setup variables
        self.body_unique_id = None
        self.base_collision_shape_index = None
        self.base_visual_shape_index = None
        self.linear_offset = None
        self.rotational_offset_R = None
        self.offset_T = None
        self.object_base_tf_frame_id = None
        self.object_base_tf_frame_is_static = None
        self.object_base_tf_frame_listener_frequency = None
        self.object_base_tf_frame_listener_timer = None

        # Initialize object
        self.init()


    def init(self):
        raise NotImplementedError('a child class of PybulletObject needs to implement an init method')


    def create_visual_shape(self, config):
        config['shapeType'] = getattr(self.pb, config['shapeType'])  # expect string
        if 'fileName' in config.keys():
            config['fileName'] = replace_package(config['fileName'])
        return self.pb.createVisualShape(**config)


    def create_collision_shape(self, config):
        config['shapeType'] = getattr(self.pb, config['shapeType'])  # expect string
        if 'fileName' in config.keys():
            config['fileName'] = replace_package(config['fileName'])
        return self.pb.createCollisionShape(**config)


    def change_dynamics(self, config, link_index=-1):
        config['bodyUniqueId'] = self.body_unique_id
        config['linkIndex'] = link_index
        if 'activationState' in config.keys():
            config['activationState'] = getattr(self.pb, config['activationState'])
        self.pb.changeDynamics(**config)


    def get_frame_offset(self):

        # Get linear/rotational offset
        # Note: rotational offset in Euler angles [degrees]
        self.linear_offset = np.asarray(self.config.get('linear_offset', np.zeros(3)))
        rotational_offset = np.deg2rad(self.config.get('rotational_offset', np.zeros(3)))
        rotational_offset_quat = tf_conversions.transformations.quaternion_from_euler(
            rotational_offset[0],
            rotational_offset[1],
            rotational_offset[2],
        )
        self.rotational_offset_R = tf_conversions.transformations.quaternion_matrix(rotational_offset_quat)

        # Compute transform matrix for offset
        self.offset_T = np.eye(4)
        self.offset_T[:3,:3] = self.rotational_offset_R
        self.offset_T[:3,-1] = self.linear_offset


    def setup_object_base_tf_frame(self):

        # Get object base tf frame (optional, default to world frame)
        self.object_base_tf_frame_id = self.config.get('object_base_tf_frame_id', 'rpbi/world')

        # Check if the object base frame is static or not
        self.object_base_tf_frame_is_static = self.config['object_base_tf_frame_is_static']

        if self.object_base_tf_frame_id != 'rpbi/world':
            # base frame is not world frame -> listen to tf frames and reset object base position/orientation

            # Get listener frequency (optional, default to 50Hz)
            self.object_base_tf_frame_listener_frequency = self.config.get('object_base_tf_frame_listener_frequency', 50)

            # Start looping: collect object tf
            object_base_tf_frame_listener_dt = 1.0/float(self.object_base_tf_frame_listener_frequency)
            self.object_base_tf_frame_listener_timer = self.node.Timer(self.node.Duration(object_base_tf_frame_listener_dt), self.object_base_tf_frame_listener_callback)

        else:
            # base frame is world frame -> reset base position/orientation using offset

            # Get pos/rot for offset in world frame
            pos_use = self.offset_T[:3,-3].flatten()
            rot_use = tf_conversions.transformations.quaternion_from_matrix(self.offset_T)

            # Set base position/orientation
            self.pb.resetBasePositionAndOrientation(self.body_unique_id, pos_use, rot_use)


    def object_base_tf_frame_listener_callback(self, event):

        # Get object base tf frame in world
        pos, rot = self.node.tf.get_tf('rpbi/world', self.object_base_tf_frame_id)

        # Failed to retrieve tf, loop again
        if pos is None: return

        # Apply offset
        T0 = tf_conversions.quaternion_matrix(rot)
        T0[:3,-1] = pos
        T = self.offset_T @ T0
        pos_use = T[:3,-3].flatten()
        rot_use = tf_conversions.transformations.quaternion_from_matrix(T)

        # Set object position/orientation in Pybullet
        self.pb.resetBasePositionAndOrientation(self.body_unique_id, pos_use, rot_use)

        # Shutdown if tf frame is static
        if self.object_base_tf_frame_is_static:
            self.object_base_tf_frame_listener_timer.shutdown()
