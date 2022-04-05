import time
from custom_ros_tools.config import ros_package_path, replace_package
from .pybullet_object_pose import PybulletObjectPose

class URDF:

    def __init__(self, pb_obj):
        self.pb_obj = pb_obj

        # If user specifies robot_description then get it from ros parameter
        if self.pb_obj.config['loadURDF']['fileName'] == 'robot_description':
            self.pb_obj.config['loadURDF']['fileName'] = self.get_urdf_from_robot_description()

        # Ensure urdf filename is an absolute path
        self.pb_obj.config['loadURDF']['fileName'] = replace_package(self.pb_obj.config['loadURDF']['fileName'])

        # Ensure no package:// statements appear in urdf (handle otherwise)
        if self.urdf_contains_ros_package_statements():
            self.replace_ros_package_statements()

        # Create pose object
        self.pose = PybulletObjectPose(pb_obj)

        # Setup pose
        self.after_load_urdf_reset_base_velocity = False
        if self.is_fixed_base:
            self.set_base_position_and_orientation_in_config()
        else:
            if not self.pb_obj.is_visual_robot:
                self.set_base_position_and_orientation_in_config()  # initial pose
                self.after_load_urdf_reset_base_velocity = True  # inital velocity (set after pybullet.loadURDF is called in load method)
            else:
                self.pose.start_resetter()

    def set_base_position_and_orientation_in_config(self):
        self.pose.get_base_from_tf()
        tf_base_pos, tf_base_ori = self.pose.get()
        if not self.user_given_base_position():
            self.pb_obj.config['loadURDF']['basePosition'] = tf_base_pos
        if not self.user_given_base_orientation():
            self.pb_obj.config['loadURDF']['baseOrientation'] = tf_base_ori

        # Parse loadURDF input
        if 'flags' in self.pb_obj.config['loadURDF']:
            self.pb_obj.config['loadURDF']['flags'] = self.pb_obj.node.parse_options(self.pb_obj.config['loadURDF']['flags'])

    @property
    def is_fixed_base(self):
        return self.pb_obj.config['loadURDF'].get('useFixedBase', 0)

    @property
    def filename(self):
        return replace_package(self.pb_obj.config['loadURDF']['fileName'])

    @filename.setter
    def filename(self, value):
        self.pb_obj.config['loadURDF']['fileName'] = value

    def get_urdf_from_robot_description(self):

        # Get urdf string from ROS
        urdf_str = self.pb_obj.node.get_param('robot_description')

        # Dump urdf to temp file
        stamp = time.time_ns()  # ensure filename uniqueness
        temp_filename = f'/tmp/pybullet_robot_urdf_robot_description_{stamp}.urdf'

        with open(temp_filename, 'w') as fout:
            fout.write(urdf_str)

        return temp_filename

    def user_given_base_position(self):
        return 'basePosition' in self.pb_obj.config['loadURDF']

    def user_given_base_orientation(self):
        return 'baseOrientation' in self.pb_obj.config['loadURDF']

    def urdf_contains_ros_package_statements(self):
        """Returns true when the URDF contains "package://" statements, false otherwise."""
        with open(self.filename, 'r') as f:
            return 'package://' in f.read()

    def replace_ros_package_statements(self):
        """Replace "package://" statements with absolute paths in a new file."""

        # Load urdf
        with open(self.filename, 'r') as fin:
            urdf_file_lines = fin.readlines()

        # Create new urdf in /tmp
        stamp = time.time_ns()  # ensure filename uniqueness
        temp_filename = f'/tmp/pybullet_robot_urdf_{stamp}.urdf'
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

        self.filename = temp_filename

    def load(self):

        # Load URDF
        body_unique_id = self.pb_obj.pb.loadURDF(**self.pb_obj.config['loadURDF'])

        # If non fixed base and non visual robot -> allow user to set initial base velocity
        if self.after_load_urdf_reset_base_velocity:
            reset_base_velocity_input = self.pb_obj.config.get('resetBaseVelocity')
            if isinstance(reset_base_velocity_input, dict):
                self.pb_obj.pb.resetBaseVelocity(body_unique_id, **reset_base_velocity_input)
            elif reset_base_velocity_input is None:
                pass
            else:
                raise ValueError("resetBaseVelocity config type is not supported")

        # If user wants to broadcast base pose -> start broadcaster
        if self.pose.broadcast_tf:
            self.pose.start_pose_broadcaster()

        return body_unique_id
