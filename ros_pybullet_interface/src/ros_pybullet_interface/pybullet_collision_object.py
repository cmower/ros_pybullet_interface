import pybullet
import numpy
from .pybullet_object import PybulletObject

class PybulletCollisionObject(PybulletObject):

    """Visualizes an object (given tf frame). Other objects will react to this in pybullet, but this object will be unaffected."""

    # Config:
    #
    # name: str
    #      Name of object.
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
    # tf_frame_id: str
    #      Frame unique id for object base pose.
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

        # Initialize body id
        self.body_unique_id = pybullet.createMultiBody(
            baseMass=0.0,  # makes object static
            baseVisualShapeIndex=self.visual_id,
            baseCollisionShapeIndex=self.collision_id,
        )

        # Set dynamic parameters
        self.init_dynamics()

        # Set tf_frame_id
        self.tf_frame_id = self.config['tf_frame_id']

    def update(self):

        # Retrieve transform
        pos, rot = self.tf.get_tf('rpbi/world', self.tf_frame_id)
        if pos is None: return

        # Reset base position/orientation
        pybullet.resetBasePositionAndOrientation(self.body_unique_id, pos, rot)
