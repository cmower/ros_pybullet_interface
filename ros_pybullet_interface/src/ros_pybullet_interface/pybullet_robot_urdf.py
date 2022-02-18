import time

def urdf_contains_ros_package_statements(filename):
    ret = False
    with open(filename, 'r') as f:
        for line in f.readlines():
            if 'package://' in line:
                ret = True
                break
    return ret

def replace_ros_package_statements(filename):

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
                abs_path = rp.get_path(package_name)
                old = "package://" + package_name
                new = abs_path
                line_out = line.replace(old, new)
            else:
                # False -> no package statement, use line
                line_out = line

            # Write line to new temp file
            fout.write(line_out)

    return temp_filename
