import yaml



# ------------------------------------------------------
#
# Helpful functions
# ------------------------------------------------------

def loadYAMLConfig(file_name):
    with open(file_name, 'r') as configfile:
         config = yaml.load(configfile, Loader=yaml.FullLoader)
    return config