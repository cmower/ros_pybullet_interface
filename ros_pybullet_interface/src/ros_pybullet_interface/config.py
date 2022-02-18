import re
import yaml
import rospkg

"""Load config files"""

rp = rospkg.RosPack()

def ros_package_path(package):
    """Wrapper for rp.get_path - to ease effort when porting to ROS2."""
    return rp.get_path(package)

def replace_package(path):
    """Repalce package name in {} with path"""
    matches = re.findall(r'{.+?}', path)
    if len(matches) > 0:
        match = matches[0]
        package = match[1:-1]
        root = rp.get_path(package)
        path = path.replace(match, root)
    return path

def load_config(file_name):
    """Load config file"""
    with open(replace_package(file_name), 'r') as configfile:
         config = yaml.load(configfile, Loader=yaml.FullLoader)
    return config
