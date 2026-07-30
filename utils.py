import numpy as np

from vpython import vec

def mm_to_cm(mm: float):
    return mm/10

def rgb_col(red:int, green:int, blue:int):
    return vec(red, green, blue)/255

def vector_to_array(value) -> np.ndarray:
    """Convert a VPython vector into a NumPy array."""
    return np.array([value.x, value.y, value.z], dtype=float)


def array_to_vector(value):
    """Convert an iterable or NumPy array into a VPython vector."""
    return vec(*value)


def normalise(value: np.ndarray) -> np.ndarray:
    """Return a normalised copy of a vector."""
    magnitude = np.linalg.norm(value)

    if magnitude == 0:
        return value.copy()

    return value / magnitude


def rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Create a 3D rotation matrix around an arbitrary axis."""
    axis = normalise(axis)

    x, y, z = axis
    cosine = np.cos(angle)
    sine = np.sin(angle)
    inverse_cosine = 1.0 - cosine

    return np.array([
        [
            cosine + x * x * inverse_cosine,
            x * y * inverse_cosine - z * sine,
            x * z * inverse_cosine + y * sine,
        ],
        [
            y * x * inverse_cosine + z * sine,
            cosine + y * y * inverse_cosine,
            y * z * inverse_cosine - x * sine,
        ],
        [
            z * x * inverse_cosine - y * sine,
            z * y * inverse_cosine + x * sine,
            cosine + z * z * inverse_cosine,
        ],
    ])


def euler_rotation_matrix(rotation_x: float, rotation_y: float, rotation_z: float) -> np.ndarray:
    """Create a rotation matrix from XYZ Euler rotations."""
    rotate_x = rotation_matrix(np.array([1.0, 0.0, 0.0]), rotation_x)
    rotate_y = rotation_matrix(np.array([0.0, 1.0, 0.0]), rotation_y)
    rotate_z = rotation_matrix(np.array([0.0, 0.0, 1.0]), rotation_z)

    return rotate_z @ rotate_y @ rotate_x
