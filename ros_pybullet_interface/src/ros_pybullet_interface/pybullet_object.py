import tf_conversions
import numpy as np
from .config import replace_package


class PybulletObject:


    def __init__(self, pb, node, config):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node
        self.pubs = {}
        self.srvs = {}
        self.subs = {}
        self.timers = {}

        # Init config
        self.config = config
        self.name = self.config['name']
        del self.config['name']

        # Setup variables
        self.body_unique_id = None
        self.offset = None  # object offset in object base frame
        self.base = None  # object base frame defined in rpbi/world

        # Initialize object
        self.init()


    def init(self):
        raise NotImplementedError('a child class of PybulletObject needs to implement an init method')


    def create_visual_shape(self, config):
        if isinstance(config['shapeType'], str):
            config['shapeType'] = getattr(self.pb, config['shapeType'])
        if 'fileName' in config.keys():
            config['fileName'] = replace_package(config['fileName'])
        return self.pb.createVisualShape(**config)


    def create_collision_shape(self, config):
        if isinstance(config['shapeType'], str):
            config['shapeType'] = getattr(self.pb, config['shapeType'])
        if 'fileName' in config.keys():
            config['fileName'] = replace_package(config['fileName'])
        return self.pb.createCollisionShape(**config)


    def change_dynamics(self, config, link_index=-1):
        config['bodyUniqueId'] = self.body_unique_id
        config['linkIndex'] = link_index
        if 'mass' in config.keys():
            del config['mass']  # use baseMass in config
        if 'activationState' in config.keys():
            config['activationState'] = getattr(self.pb, config['activationState'])
        self.pb.changeDynamics(**config)


    def get_object_offset_in_base_tf(self, object_tf_config):

        offset = object_tf_config.get('offset', [0.0]*6)
        if len(offset) == 3:
            # pos
            offset_lin = np.array(offset)
            offset_rot = np.array([0., 0., 0., 1.])
        elif len(offset) == 6:
            # pos, eul
            offset_lin = np.array(offset[:3])
            offset_eul = np.deg2rad(offset[3:])
            offset_rot = tf_conversions.transformations.quaternion_from_euler(offset_eul[0], offset_eul[1], offset_eul[2])
        elif len(offset) == 7:
            # pos, quat
            offset_lin = np.array(offset[:3])
            offset_rot = np.array(offset[3:])
        else:
            raise ValueError("offset is incorrect length, got %d, expected either 3, 6, or 7" % len(offset))

        return self.node.tf.position_and_quaternion_to_matrix(offset_lin, offset_rot)


    def get_static_object_base_tf_in_world(self, object_tf_config):
        base_tf_id = object_tf_config.get('base_tf_id', 'rpbi/world')
        if base_tf_id != 'rpbi/world':
            pos, rot = self.node.wait_for_tf('rpbi/world', base_tf_id, timeout=object_tf_config.get('timeout'))
        else:
            # base frame is world -> return identity transformation
            pos = np.zeros(3)
            rot = np.array([0., 0., 0., 1.])
        return self.node.tf.position_and_quaternion_to_matrix(pos, rot)


    def start_object_base_tf_listener(self, object_tf_config):
        self.base_tf_id = object_tf_config.get('base_tf_id', 'rpbi/world')
        freq = object_tf_config.get('listener_frequency', 50)
        self.timers['object_base_tf_listener'] = self.node.Timer(self.node.Duration(1.0/float(freq)), self._update_object_base_tf)


    def _update_object_base_tf(self, event):
        pos, rot = self.node.tf.get_tf('rpbi/world', self.object_base_tf_frame_id)
        if pos is None: return
        self.base = self.node.tf.position_and_quaternion_to_matrix(pos, rot)


    def start_applying_non_static_object_tf(self, object_tf_config):
        freq = object_tf_config.get('listener_frequency', 50)
        self.timers['applying_non_static_object_tf'] = self.node.Timer(self.node.Duration(1.0/float(freq)), self._update_non_static_object_tf)


    def _update_non_static_object_tf(self, event):
        if self.base is None: return
        self.reset_base_position_and_orientation()

    def get_base_position_and_orientation(self, offset, base):
        T = offset @ base
        pos = T[:3,-1].flatten()
        rot = tf_conversions.transformations.quaternion_from_matrix(T)
        return pos, rot

    def reset_base_position_and_orientation(self):
        pos, rot = self.get_base_position_and_orientation(self.offset, self.base)
        self.pb.resetBasePositionAndOrientation(self.body_unique_id, pos, rot)


    def destroy(self):

        # Remove object from pybullet
        self.pb.removeBody(self.body_unique_id)

        # Close all ROS communication
        for t in self.timers.values():
            t.shutdown()
        for s in self.subs.values():
            s.unregister()
        for p in self.pubs.values():
            p.unregister()
        for s in self.srvs.values():
            s.shutdown()


class PybulletObjectArray:


    def __init__(self, pb, node, config, object_type, num_objects):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Create object array
        self.objects = []
        for i in range(num_objects):

            # Update config
            config_i = config.copy()
            config_i['name'] = config['name'] + str(i)
            if 'object_base_tf_frame_id' in config.keys():
                if config['object_base_tf_frame_id'] != 'rpbi/world':
                    config_i['object_base_tf_frame_id'] = config['object_base_tf_frame_id'] + str(i)
            if 'tf_frame_id' in config.keys():
                config_i['tf_frame_id'] = config['tf_frame_id'] + str(i)

            # Append object
            self.objects.append(object_type(pb, node, config_i))

    def destroy(self):
        for obj in self.objects:
            obj.destroy()
