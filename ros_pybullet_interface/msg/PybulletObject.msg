# object types
uint8 VISUAL=0
uint8 COLLISION=1
uint8 DYNAMIC=2
uint8 ROBOT=3

uint8 object_type  # use above

# Configuration filename
# NOTE: if filename is specified then config is ignored
string filename

# Configuration as a string
# NOTE: this is used when filename is not given
string config