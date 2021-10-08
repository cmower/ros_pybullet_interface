import numpy
import tf_conversions
from .pybullet_object import PybulletObject

class PybulletDynamicObject(PybulletObject):

    """Objects motion defined by pybullet, only publishes pose as tf frames to ROS"""

    # Config:
    #
    # name: str
    #      Name of object.
    #
    # base_mass: float
    #      Mass of the base [kg].
    #
    # init_position: float[3]
    #      Initial base position, Cartesian world position.
    #
    # init_base_orient_eulerXYZ: float[3]
    #      Initial base orientation as Euler XYZ angles (degrees).
    #
    # init_linear_velocity: float[3]
    #      Initial linear velocity, Cartesian world coordinates.
    #
    # init_angular_velocity: float[3]
    #      Initial angular velocity [wx,wy,wz] in Cartesian world
    #      coordinates (degrees).
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

    def init(self):

        # Initialize visual_id and collision_id
        self.init_visual_id()
        self.init_collision_id()

        # Get initial position/orientation
        init_ori = tf_conversions.transformations.quaternion_from_euler(*numpy.deg2rad(self.config['init_base_orient_eulerXYZ']))
        init_pos = self.config['init_position']

        # Initialize body id
        self.body_unique_id = self.pb.createMultiBody(
            baseMass=self.config['base_mass'],
            baseVisualShapeIndex=self.visual_id,
            baseCollisionShapeIndex=self.collision_id,
            basePosition=init_pos,
            baseOrientation=init_ori,
        )

        # Set dynamic parameters
        self.init_dynamics()

        # Init velocity
        self.pb.resetBaseVelocity(self.body_unique_id, self.config.get('init_linear_velocity', [0, 0, 0]), self.config['init_angular_velocity'])

        # Set tf_frame_id
        self.tf_frame_id = f'rpbi/{self.name}'

    def update(self):
        pos, ori = self.pb.getBasePositionAndOrientation(self.body_unique_id)
        self.tf.set_tf('rpbi/world', self.tf_frame_id, pos, ori)
