import re
import yaml
import rospkg

"""Load config files"""

rp = rospkg.RosPack()

def ros_package_path(package):
    """Wrapper for rp.get_path - to ease effort when porting to ROS2.

Syntax
------

    path = config.ros_package_path(package)

Parameters
----------

    package (string)
        Name of ROS package.

Returns
-------

    path (string)
        Absolute path to ROS package.

"""
    return rp.get_path(package)

def replace_package(path):
    """Replace package in path to file with absolute path to file.

E.g. when path = "{ros_package}/path/to/file.txt" then
replace_package(path) will return the absolute path to file.txt. When
ROS package not specified between "{" and "}" then the path is
returned.

Syntax
------

    abs_path = config.replace_package(path)

Parameters
----------

    path (string)
        Relative path to file. Note, ROS package must be specified within "{" and "}".

Returns
-------

    abs_path (string)
        Absolute path to file.

"""
    matches = re.findall(r'{.+?}', path)
    if len(matches) > 0:
        match = matches[0]
        package = match[1:-1]
        root = rp.get_path(package)
        path = path.replace(match, root)
    return path

def load_config(path):
    """Load config file.

Syntax
------

    config = config.load_config(path)

Parameters
----------

    path (string)
        Path to config YAML file. Note, you can use reletive paths to
        ROS packages by specifying the ROS package name between "{"
        and "}" and the beginning of path.

Returns
-------

    config (dict)
        Configuration.

"""
    with open(replace_package(path), 'r') as configfile:
         config = yaml.load(configfile, Loader=yaml.FullLoader)
    return config
