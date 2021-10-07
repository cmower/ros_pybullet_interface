import pybullet
from .pybullet_object import PybulletObject

class PybulletVisualObject(PybulletObject):

    """Simply visualizes an object in pybullet."""

    # Config:
    #
    # name: str
    #      Name of object.
    #
    # tf_frame_id: str
    #      Frame unique id for object base pose.
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

    def init(self):

        # Initialize visual_id
        self.init_visual_id()

        # Initialize body id
        self.body_unique_id = pybullet.createMultiBody(baseVisualShapeIndex=self.visual_id)

        # Get tf frame id
        self.tf_frame_id = self.config['tf_frame_id']

    def update(self):

        # Retrieve transform
        pos, rot = self.tf.get_tf('rpbi/world', self.tf_frame_id)
        if pos is None: return

        # Reset base position/orientation
        pybullet.resetBasePositionAndOrientation(self.body_unique_id, pos, rot)
