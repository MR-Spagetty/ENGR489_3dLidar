"""Hierarchical transform groups for VPython simulation objects."""

from __future__ import annotations

from typing import Iterable

import numpy as np
from vpython import vec

from utils import vector_to_array as vec_to_arr


_EPSILON = 1e-12


def _normalise(value: np.ndarray) -> np.ndarray:
    magnitude = np.linalg.norm(value)
    if magnitude < _EPSILON:
        return np.zeros(3, dtype=float)
    return value / magnitude


def _safe_up(axis: np.ndarray, up: np.ndarray) -> np.ndarray:
    """Return an up vector perpendicular to axis, avoiding VPython box skew."""
    axis_direction = _normalise(axis)
    if np.linalg.norm(axis_direction) < _EPSILON:
        return up

    corrected = up - np.dot(up, axis_direction) * axis_direction
    corrected = _normalise(corrected)

    if np.linalg.norm(corrected) >= _EPSILON:
        return corrected

    fallback = np.array([0.0, 1.0, 0.0])
    if abs(np.dot(fallback, axis_direction)) > 0.99:
        fallback = np.array([0.0, 0.0, 1.0])

    return _normalise(
        fallback - np.dot(fallback, axis_direction) * axis_direction
    )


class Obj:
    """Associates a local reference transform with a visible VPython object."""

    def __init__(self, ref, controlled, colliders=None) -> None:
        self.ref = ref
        self.ref.visible = False

        self.working_pos = np.zeros(3, dtype=float)
        self.working_up = np.zeros(3, dtype=float)
        self.working_axis = np.zeros(3, dtype=float)

        self.colliders = list(colliders) if colliders else []
        self.out = controlled

    # Compatibility with older code and the raycaster versions that used
    # camel-case working transform names.
    @property
    def workingPos(self):
        return self.working_pos

    @workingPos.setter
    def workingPos(self, value):
        self.working_pos = value

    @property
    def workingUp(self):
        return self.working_up

    @workingUp.setter
    def workingUp(self, value):
        self.working_up = value

    @property
    def workingAxis(self):
        return self.working_axis

    @workingAxis.setter
    def workingAxis(self, value):
        self.working_axis = value


class Group:
    """A hierarchical transform group containing Obj instances."""

    def __init__(
        self,
        parent: Group | None,
        children: Iterable[Obj] | None = None,
    ) -> None:
        self.parent = parent
        self.children = list(children) if children is not None else []

        self.offset = vec(0, 0, 0)
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0

        self.child_groups: list[Group] = []

        if parent is not None:
            parent.child_groups.append(self)

    @property
    def rotX(self) -> float:
        return self.rot_x

    @rotX.setter
    def rotX(self, value: float) -> None:
        self.rot_x = value

    @property
    def rotY(self) -> float:
        return self.rot_y

    @rotY.setter
    def rotY(self, value: float) -> None:
        self.rot_y = value

    @property
    def rotZ(self) -> float:
        return self.rot_z

    @rotZ.setter
    def rotZ(self, value: float) -> None:
        self.rot_z = value

    @property
    def childGroups(self) -> list[Group]:
        return self.child_groups

    def toggle_visible(self):
        for child in self.children:
            child.out.visible = not child.out.visible
        for child in self.child_groups:
            child.toggle_visible()

    def get_rot(self) -> np.ndarray:
        """Return the XYZ Euler rotation matrix used by the project utilities."""
        cosine_x, sine_x = np.cos(self.rot_x), np.sin(self.rot_x)
        cosine_y, sine_y = np.cos(self.rot_y), np.sin(self.rot_y)
        cosine_z, sine_z = np.cos(self.rot_z), np.sin(self.rot_z)

        rotation_x = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cosine_x, -sine_x],
            [0.0, sine_x, cosine_x],
        ])
        rotation_y = np.array([
            [cosine_y, 0.0, sine_y],
            [0.0, 1.0, 0.0],
            [-sine_y, 0.0, cosine_y],
        ])
        rotation_z = np.array([
            [cosine_z, -sine_z, 0.0],
            [sine_z, cosine_z, 0.0],
            [0.0, 0.0, 1.0],
        ])

        # Apply X, then Y, then Z to column vectors.
        return rotation_z @ rotation_y @ rotation_x

    def prop(self) -> None:
        """Propagate this complete group tree to its controlled objects."""
        self.apply()
        for child_group in self.child_groups:
            child_group.prop()

    def apply(self, children: list[Obj] | None = None) -> None:
        called_from_child = children is not None

        if children is None:
            children = self.children
            for child in children:
                child.working_pos = vec_to_arr(child.ref.pos)
                child.working_up = vec_to_arr(child.ref.up)
                child.working_axis = vec_to_arr(child.ref.axis)

        rotation = self.get_rot()
        offset = vec_to_arr(self.offset)

        for child in children:
            child.working_axis = rotation @ child.working_axis
            child.working_up = rotation @ child.working_up
            child.working_pos = rotation @ child.working_pos + offset

        if self.parent is not None:
            self.parent.apply(children)

        if called_from_child:
            return

        for child in children:
            corrected_up = _safe_up(child.working_axis, child.working_up)
            child.working_up = corrected_up
            child.out.pos = vec(*child.working_pos)
            child.out.axis = vec(*child.working_axis)
            child.out.up = vec(*corrected_up)


obj = Obj
group = Group
