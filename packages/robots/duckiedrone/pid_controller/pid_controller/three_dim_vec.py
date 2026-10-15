# ruff: noqa: ANN001,ANN204,D102,D105,D107,E501,W292
"""Small vector helpers used by the Duckiedrone PID controller."""

from dataclasses import astuple, dataclass
from math import sqrt

from geometry_msgs.msg import Vector3


@dataclass
class ThreeDimVec:
    """Store three values addressed as x, y, and z."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @property
    def list(self) -> list[float]:
        return list(astuple(self))

    def as_ros_vector3(self) -> Vector3:
        return Vector3(x=self.x, y=self.y, z=self.z)

    def __iter__(self):
        return iter((self.x, self.y, self.z))

    def __mul__(self, other: float):
        return ThreeDimVec(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other: float):
        return self.__mul__(other)

    def __truediv__(self, other: float):
        return ThreeDimVec(self.x / other, self.y / other, self.z / other)

    def __add__(self, other):
        return ThreeDimVec(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return ThreeDimVec(self.x - other.x, self.y - other.y, self.z - other.z)

    @property
    def magnitude(self) -> float:
        return sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    @property
    def xy_magnitude(self) -> float:
        return sqrt(self.x * self.x + self.y * self.y)


@dataclass
class Position(ThreeDimVec):
    """Position vector."""


@dataclass
class Velocity(ThreeDimVec):
    """Velocity vector."""


@dataclass
class Error(ThreeDimVec):
    """Error vector."""


class RPY(ThreeDimVec):
    """Roll-pitch-yaw tuple stored in x/y/z."""

    def __init__(self, r: float = 0.0, p: float = 0.0, y: float = 0.0):
        super().__init__(r, p, y)
        self.r = self.x
        self.p = self.y
        self.y = self.z