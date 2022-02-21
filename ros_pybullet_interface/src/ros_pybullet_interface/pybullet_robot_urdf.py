import time
from .config import ros_package_path

def urdf_contains_ros_package_statements(filename):
    """Returns true when the URDF contains "package://" statements, false otherwise.

Syntax
------

    r = urdf_contains_ros_package_statements(filename)

Parameters
----------

    filename (string)
        Path to URDF.

Returns
-------

    r (bool)
        True when URDF contains "package://" statements, False otherwise.

"""
    ret = False
    with open(filename, 'r') as f:
        for line in f.readlines():
            if 'package://' in line:
                ret = True
                break
    return ret

def replace_ros_package_statements(filename):
    """Replace "package://" statements with absolute paths in a new file.

Syntax
------

    filename_out = replace_ros_package_statements(filename_in)

Parameters
----------

    filename_in (string)
        Path to URDF containing "package://" statements.

Returns
-------

    filename_out (string)
        Path to file in /tmp containing matching URDF with "package://" statements replaced by absolute paths.

"""

    # Load urdf
    with open(filename, 'r') as fin:
        urdf_file_lines = fin.readlines()

    # Create new urdf in /tmp
    stamp = time.time_ns()  # ensure filename uniqueness
    temp_filename = '/tmp/pybullet_robot_urdf_{stamp}.urdf'
    with open(temp_filename, 'w') as fout:

        # Loop over urdf file lines
        for line in urdf_file_lines:

            # Check if package statement in line
            if 'package://' in line:
                # True -> replace package with absolute path
                idx = line.find("package://")
                package_name = line[idx:].split('/')[2]
                abs_path = ros_package_path(package_name)
                old = "package://" + package_name
                new = abs_path
                line_out = line.replace(old, new)
            else:
                # False -> no package statement, use line
                line_out = line

            # Write line to new temp file
            fout.write(line_out)

    return temp_filename
