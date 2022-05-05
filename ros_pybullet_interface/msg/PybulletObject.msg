# object types
uint8 VISUAL=0
uint8 COLLISION=1
uint8 DYNAMIC=2
uint8 ROBOT=3
uint8 SOFT=4

uint8 object_type  # use above

# Configuration filename
# NOTE: if filename is specified then config is ignored
string filename

# Configuration as a string
# NOTE: this is used when filename is not given
#
# If you want to use this then you need to send the configuration as a
# string that is ultimately a yaml file. The best way to do this is is
# to use the config_to_str method provided in
# custom_ros_tools.config. See example below.
#
#   from custom_ros_tools.config import config_to_str
#   from ros_pybullet_interface.msg import PybulletObject
#
#   # make config
#   config = {}
#   # ...
#
#   req = PybulletObject(config=config_to_str(config))
#
string config
