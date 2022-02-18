
class UniqueDict(dict):

    """Dictionary that only enforces unique keys."""

    def __setitem__(self, key, value):
        if key not in self:
            dict.__setitem__(self, key, value)
        else:
            raise KeyError(f"Key '{key}' already exists!")

class TimeoutExceeded(Exception):
    pass
