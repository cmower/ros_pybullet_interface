import rospkg
import yaml
import numpy as np
import math

ROOT_DIR = rospkg.RosPack().get_path('ros_pybullet_interface')


def loadYAMLConfig(file_name):
    with open(file_name, 'r') as configfile:
         config = yaml.load(configfile, Loader=yaml.FullLoader)
    return config



def rotation_matrix_from_axis_angle(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    source: https://stackoverflow.com/questions/6802577/rotation-of-3d-vector
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

# v = [3, 5, 0]
# axis = [4, 4, 1]
# theta = 1.2
# print(np.dot(rotation_matrix_from_axis_angle(axis, theta), v))
# [ 2.74911638  4.77180932  1.91629719]
