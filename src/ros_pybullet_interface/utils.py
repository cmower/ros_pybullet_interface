import rospkg
# ------------------------------------------------------
#
# Helpful functions
# ------------------------------------------------------

import yaml

ROOT_DIR = rospkg.RosPack().get_path('ros_pybullet_interface')


def loadYAMLConfig(file_name):
    with open(file_name, 'r') as configfile:
         config = yaml.load(configfile, Loader=yaml.FullLoader)
    return config
