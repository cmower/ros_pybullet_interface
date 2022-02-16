
class UniqueDict(dict):

    def __setitem__(self, key, value):
        if key not in self:
            dict.__setitem__(self, key, value)
        else:
            raise KeyError(f"Key '{key}' already exists")
