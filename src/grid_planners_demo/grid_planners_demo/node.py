from tf_transformations import euler_from_quaternion

class Node:
    def __init__(self, x, y, theta=0.0, g=0.0, h=0.0, f=0.0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.h = h
        self.f = f
        self.parent = parent

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def get_coords(self):
        return (self.x, self.y)

    def get_theta(self):
        return self.theta

    def set_coords(self, coords):
        self.x = coords[0]
        self.y = coords[1]

    def set_theta(self, theta):
        self.theta = theta

    @classmethod
    def from_tf(cls, position, quaternion):
        new_state = cls(0.0, 0.0, 0.0)
        new_state.x = position[0]
        new_state.y = position[1]
        # This is the corrected line for the data type mismatch
        (roll, pitch, yaw) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        new_state.theta = yaw
        return new_state
