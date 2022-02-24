from .pybullet_object import PybulletObject
from visualization_msgs.msg import Marker

class PybulletVisualObject(PybulletObject):

    """Simply visualizes an object in Pybullet."""

    def init(self):

        self.marker = None

        create_visual_shape_config = self.config['createVisualShape']
        self.base_visual_shape_index = self.create_visual_shape(create_visual_shape_config)

        object_tf_config = self.config.get('object_tf', {})
        self.offset = self.get_object_offset_in_base_tf(object_tf_config)

        if object_tf_config.get('is_static', True):
            # object base is world or specified as static -> get base frame and apply
            self.base = get_static_object_base_tf_in_world(object_tf_config)
            pos, rot = self.get_base_position_and_orientation(self.offset, self.base)
            self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index, basePosition=pos, baseOrientation=rot)
        else:
            # object base is non static
            self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index)
            self.start_object_base_tf_listener(object_tf_config)
            self.start_applying_non_static_object_tf(object_tf_config)

        if self.config.get('publish_as_marker', False):
            self.pubs['marker'] = self.node.Publisher(f'rpbi/{self.name}/marker', Marker, queue_size=10)

            self.marker = Marker()
            self.marker.header.frame_id = 'rpbi/world'
            self.marker.id = 0
            self.marker.action = Marker.ADD
            if self.visual_shape_type == self.pb.GEOM_SPHERE:
                self.marker.type = Marker.SPHERE
                diameter = 2.0*create_visual_shape_config.get('radius', 0.5)  # default from Pybullet documentation
                self.marker.scale.x = diameter
                self.marker.scale.y = diameter
                self.marker.scale.z = diameter
            elif self.visual_shape_type == self.pb.GEOM_BOX:
                self.marker.type = Marker.CUBE
                half_extents = create_visual_shape_config.get('halfExtents', [1., 1., 1.])  # default from Pybullet documentation
                self.marker.scale.x = 2.0*half_extents[0]
                self.marker.scale.y = 2.0*half_extents[1]
                self.marker.scale.z = 2.0*half_extents[2]
            elif self.visual_shape_type == self.pb.GEOM_CYLINDER:
                self.marker.type = Marker.CYLINDER
                length = create_visual_shape_config.get('length', 1.) # default from Pybullet documentation
                diameter = 2.0*create_visual_shape_config.get('radius', 0.5)  # default from Pybullet documentation
                self.marker.scale.x = diameter
                self.marker.scale.y = diameter
                self.marker.scale.z = length
            else:
                raise TypeError(f"The shape type with id {self.visual_shape_type} is not supported for publishing markers, currently only sphere, box, and cylinders are supported.")
            rgbaColor = create_visual_shape_config.get('rgbaColor', [0., 0., 0., 1.])
            self.marker.color.r = rgbaColor[0]
            self.marker.color.g = rgbaColor[1]
            self.marker.color.b = rgbaColor[2]
            self.marker.color.a = rgbaColor[3]

            freq = self.config.get('publish_as_marker_frequency', 50)
            self.timers['marker'] = self.node.Timer(self.node.Duration(1.0/float(freq)), self.publish_as_marker)

    def publish_as_marker(self, event):
        pos, rot = self.pb.getBasePositionAndOrientation(self.body_unique_id)
        self.marker.header.stamp = self.node.time_now()
        self.marker.pose.position.x = pos[0]
        self.marker.pose.position.y = pos[1]
        self.marker.pose.position.z = pos[2]
        self.marker.pose.orientation.x = rot[0]
        self.marker.pose.orientation.y = rot[1]
        self.marker.pose.orientation.z = rot[2]
        self.marker.pose.orientation.w = rot[3]
        self.pubs['marker'].publish(self.marker)
