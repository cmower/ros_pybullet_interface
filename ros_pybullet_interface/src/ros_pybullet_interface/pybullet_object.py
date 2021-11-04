from .config import replace_package


"""Base pybullet object class"""


# Defaults
MESH_SCALE_DEFAULT = [1, 1, 1]
RGBA_COLOR_DEFAULT = [1,1,1,1]
RADIUS_DEFAULT = 0.5
HALF_EXTENDS_DEFAULT = [1,1,1]
HEIGHT_DEFAULT = 1.0
PLANE_NORMAL_DEFAULT = [0, 0, 1]

# Main class
class PybulletObject:

    # Config:
    # See sub-classes.

    def __init__(self, config, tf_interface, pybullet, ros_api=None):

        # Initial init
        self.body_unique_id = None  # object unique id, as returned from self.pb.loadURDF or self.pb.createMultiBody
        self.visual_id = None       # unique id from self.pb.createVisualShape, this isn't always required in sub-classes, when required use self.init_visual_id
        self.collision_id = None    # unique id from createCollisionShape, this isn't always required in sub-classes, when required use self.init_collision_id
        self.config = config
        self.name = self.config['name']
        self.pb = pybullet
        self.ros_api = ros_api

        # Init tf2 interface
        self.tf = tf_interface  # see tf_interface.py

        # Initialize subclass
        self.init()


    def init_visual_id(self):
        # Config:
        #
        # object_type: str
        #      'mesh', 'sphere', 'box', 'cylinder', or
        #      'plane'. Required parameters below.
        #
        # When object_type == 'mesh'
        #
        # file_name: str
        #      Filename for mesh object, only supports .obj files.
        #
        # mesh_scale: float[3]
        #      Default is [1,1,1].
        #
        # rgba_color: float[4]
        #      Color components for red, green, blue, and alpha, each
        #      in range [0, 1], default is [1,1,1,1].
        #
        # When object_type == 'sphere'
        #
        # radius:
        #      Default is 0.5.
        #
        # rgba_color: float[4]
        #      Color components for red, green, blue, and alpha, each
        #      in range [0, 1], default is [1,1,1,1].
        #
        # When object_type == 'box'
        #
        # half_extends: float[3]
        #      Default [1,1,1].
        #
        # rgba_color: float[4]
        #      Color components for red, green, blue, and alpha, each
        #      in range [0, 1], default is [1,1,1,1].
        #
        # When object_type == 'cylinder'
        #
        # radius: float
        #     Default is 0.5.
        #
        # height: float
        #     Default is 1.0.
        #
        # rgba_color: float[4]
        #      Color components for red, green, blue, and alpha, each
        #      in range [0, 1], default is [1,1,1,1].
        #
        # When object_type == 'plane'
        #
        # plane_normal: float[3]
        #      Default [0, 0, 1].
        #
        # rgba_color: float[4]
        #      Color components for red, green, blue, and alpha, each
        #      in range [0, 1], default is [1,1,1,1].
        #

        # Init
        object_type = self.config['object_type']

        # Setup input for self.pb.createVisualShape
        if object_type == 'mesh':
            create_visual_shape_input = dict(
                shapeType=self.pb.GEOM_MESH,
                fileName=replace_package(self.config['file_name']),
                meshScale=self.config.get('mesh_scale', MESH_SCALE_DEFAULT),
                rgbaColor=self.config.get('rgba_color', RGBA_COLOR_DEFAULT),
            )

        elif object_type == 'sphere':
            create_visual_shape_input = dict(
                shapeType=self.pb.GEOM_SPHERE,
                radius=self.config.get('radius', RADIUS_DEFAULT),
                rgbaColor=self.config.get('rgba_color', RGBA_COLOR_DEFAULT),
            )

        elif object_type == 'box':
            create_visual_shape_input = dict(
                shapeType=self.pb.GEOM_BOX,
                halfExtents=self.config.get('half_extends', HALF_EXTENDS_DEFAULT),
                rgbaColor=self.config.get('rgba_color', RGBA_COLOR_DEFAULT),
            )

        elif object_type == 'cylinder':
            create_visual_shape_input = dict(
                shapeType=self.pb.GEOM_CYLINDER,
                radius=self.config.get('radius', RADIUS_DEFAULT),
                length=self.config.get('height', HEIGHT_DEFAULT),
                rgbaColor=self.config.get('rgba_color', RGBA_COLOR_DEFAULT),
            )

        elif object_type == 'plane':
            create_visual_shape_input = dict(
                shapeType=self.pb.GEOM_PLANE,
                planeNormal=self.config.get('plane_normal', PLANE_NORMAL_DEFAULT),
                rgbaColor=self.config.get('rgba_color', RGBA_COLOR_DEFAULT),
            )

        else:
            raise ValueError(f'object type ({object_type}) not recognized!')

        # Create visual id
        self.visual_id = self.pb.createVisualShape(**create_visual_shape_input)


    def init_collision_id(self):
        # Config:
        #
        #
        # object_type: str
        #      'mesh', 'sphere', 'box', 'cylinder', or
        #      'plane'. Required parameters below.
        #
        # When object_type == 'mesh'
        #
        # file_name: str
        #      Filename for mesh object, only supports .obj files.
        #
        # mesh_scale: float[3]
        #      Default is [1,1,1].
        #
        # When object_type == 'sphere'
        #
        # radius:
        #      Default is 0.5.
        #
        # When object_type == 'box'
        #
        # half_extends: float[3]
        #      Default [1,1,1].
        #
        # When object_type == 'cylinder'
        #
        # radius: float
        #     Default is 0.5.
        #
        # height: float
        #     Default is 1.0.
        #
        # When object_type == 'plane'
        #
        # plane_normal: float[3]
        #      Default [0, 0, 1].
        #

        # Init
        object_type = self.config['object_type']

        # Set input for createCollisionShape
        if object_type == 'mesh':
            create_collision_shape_input = dict(
                shapeType=self.pb.GEOM_MESH,
                fileName=replace_package(self.config['file_name']),
                meshScale=self.config.get('mesh_scale', MESH_SCALE_DEFAULT),
            )

        elif object_type == 'sphere':
            create_collision_shape_input = dict(
                shapeType=self.pb.GEOM_SPHERE,
                radius=self.config.get('radius', RADIUS_DEFAULT),
            )

        elif object_type == 'box':
            create_collision_shape_input = dict(
                shapeType=self.pb.GEOM_BOX,
                halfExtents=self.config.get('half_extends', HALF_EXTENDS_DEFAULT),
            )

        elif object_type == 'cylinder':
            create_collision_shape_input = dict(
                shapeType=self.pb.GEOM_CYLINDER,
                radius=self.config.get('radius', RADIUS_DEFAULT),
                height=self.config.get('height', HEIGHT_DEFAULT),
            )

        elif object_type == 'plane':
            create_collision_shape_input = dict(
                shapeType=self.pb.GEOM_PLANE,
                planeNormal=self.config.get('plane_normal', PLANE_NORMAL_DEFAULT),
            )

        else:
            raise ValueError(f'object type ({object_type}) not recognized!')

        # Create collision object
        self.collision_id = self.pb.createCollisionShape(**create_collision_shape_input)


    def init_dynamics(self, link_index=-1):
        # Config:
        #
        # lateral_friction: float
        #      Lateral (linear) contact friction
        #
        # spinning_friction: float
        #      Torisional friction around the contact normal
        #
        # rolling_friction: float
        #      Torsional friction orthogonal to contact normal (keep
        #      this value very close to zero, otherwise the simulation
        #      can become very unrealistic.
        #
        # restitution: float
        #      Bouncyness of contact. Keep it a bit less than 1,
        #      preferably closer to 0.
        #
        # linear_damping: float
        #      Linear damping of the link.
        #
        # angular_damping: float
        #      Angular damping of the link.
        #
        # contact_stiffness: float
        #      Stiffness of the contact constraints, used together
        #      with contact_damping.
        #
        # contact_damping: float
        #      Damping of the contact constraints for this
        #      body/link. Used together with contact_stiffness. This
        #      overrides the value if it was specified in the URDF
        #      file in the contact section.
        #
        self.pb.changeDynamics(
            self.body_unique_id,
            linkIndex=link_index,
            lateralFriction=self.config['lateral_friction'],
            spinningFriction=self.config['spinning_friction'],
            rollingFriction=self.config['rolling_friction'],
            restitution=self.config['restitution'],
            linearDamping=self.config['linear_damping'],
            angularDamping=self.config['angular_damping'],
            contactStiffness=self.config['contact_stiffness'],
            contactDamping=self.config['contact_damping'],
        )


    # Methods that must be implemented in sub-classes (otherwise they throw NotImplementedError's)

    def init(self):
        raise NotImplementedError('Need to implement init method for a PybulletObject subclass!')

    def update(self):
        raise NotImplementedError('Need to implement update method for a PybulletObject subclass!')
