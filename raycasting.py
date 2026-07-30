"""Raycasting against VPython objects and hierarchical Group/Obj trees."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field
from math import inf

import numpy as np
from vpython import vec, vector

RAYCASTING_VERSION = "2026-07-30-complete-rounded-v5"
_EPSILON = 1e-10


@dataclass
class BoxCollider:
    """Local-space oriented box collider attached to an Obj wrapper."""

    pos: vector
    axis: vector
    width: float
    height: float
    up: vector = field(default_factory=lambda: vec(0, 1, 0))


@dataclass
class RayHit:
    obj: object
    visual: object
    distance: float
    point: vector
    normal: vector
    collider: BoxCollider | None = None


def _to_numpy(value) -> np.ndarray:
    if hasattr(value, "x"):
        return np.array([value.x, value.y, value.z], dtype=float)
    return np.asarray(value, dtype=float)


def _to_vpython(value) -> vector:
    value = _to_numpy(value)
    return vec(float(value[0]), float(value[1]), float(value[2]))


def _normalise(value) -> np.ndarray:
    value = _to_numpy(value)
    magnitude = np.linalg.norm(value)
    if magnitude < _EPSILON:
        return np.zeros(3, dtype=float)
    return value / magnitude


def _is_group(value) -> bool:
    return (
        hasattr(value, "children")
        and hasattr(value, "child_groups")
        and not hasattr(value, "out")
    )


def _is_obj(value) -> bool:
    return hasattr(value, "out") and hasattr(value, "colliders")


def _is_iterable_container(value) -> bool:
    return isinstance(value, Iterable) and not isinstance(value, (str, bytes))


def _visual_type_name(visual) -> str:
    """Return a useful lower-case VPython primitive type name."""
    name = type(visual).__name__.lower()

    # VPython implementations sometimes expose generic internal class names.
    for attribute in ("_objName", "objName", "type"):
        value = getattr(visual, attribute, None)
        if isinstance(value, str) and value:
            return value.lower()

    return name


def _looks_like_sphere(visual) -> bool:
    name = _visual_type_name(visual)
    return "sphere" in name or (
        hasattr(visual, "radius")
        and hasattr(visual, "pos")
        and not hasattr(visual, "axis")
    )


def _looks_like_cylinder(visual) -> bool:
    name = _visual_type_name(visual)
    return "cylinder" in name or (
        hasattr(visual, "radius")
        and hasattr(visual, "axis")
        and hasattr(visual, "pos")
        and not hasattr(visual, "length")
    )


def _object_basis(visual):
    """Return orthonormal local X/Y/Z directions for a rendered object."""
    forward = _normalise(visual.axis)
    if np.linalg.norm(forward) < _EPSILON:
        return None

    up_source = getattr(visual, "up", vec(0, 1, 0))
    up = _to_numpy(up_source)
    up = up - np.dot(up, forward) * forward
    up = _normalise(up)

    if np.linalg.norm(up) < _EPSILON:
        fallback = np.array([0.0, 1.0, 0.0])
        if abs(np.dot(fallback, forward)) > 0.99:
            fallback = np.array([0.0, 0.0, 1.0])
        up = _normalise(fallback - np.dot(fallback, forward) * forward)

    side = _normalise(np.cross(forward, up))
    if np.linalg.norm(side) < _EPSILON:
        return None

    up = _normalise(np.cross(side, forward))
    return forward, up, side


def _local_vector_to_world(local_vector, basis) -> np.ndarray:
    forward, up, side = basis
    local = _to_numpy(local_vector)
    return forward * local[0] + up * local[1] + side * local[2]


def _collider_world_transform(wrapper, collider: BoxCollider):
    visual = wrapper.out
    basis = _object_basis(visual)
    if basis is None:
        return None

    world_position = _to_numpy(visual.pos) + _local_vector_to_world(
        collider.pos, basis
    )
    world_axis = _local_vector_to_world(collider.axis, basis)
    world_up = _local_vector_to_world(collider.up, basis)

    return (
        world_position,
        world_axis,
        world_up,
        float(collider.width),
        float(collider.height),
    )


def ray_sphere_intersection(
    ray_origin,
    ray_direction,
    sphere_position,
    sphere_radius,
    max_distance=inf,
):
    """Return the nearest exact ray/sphere hit, or None."""
    origin = _to_numpy(ray_origin)
    direction = _normalise(ray_direction)
    centre = _to_numpy(sphere_position)
    radius = float(sphere_radius)

    if np.linalg.norm(direction) < _EPSILON or radius <= 0:
        return None

    offset = origin - centre
    half_b = np.dot(offset, direction)
    c = np.dot(offset, offset) - radius * radius
    discriminant = half_b * half_b - c

    if discriminant < 0:
        return None

    root = np.sqrt(max(0.0, discriminant))
    near = -half_b - root
    far = -half_b + root

    if near >= 0:
        distance = near
    elif far >= 0:
        distance = far
    else:
        return None

    if distance > max_distance:
        return None

    point = origin + direction * distance
    normal = _normalise(point - centre)
    return float(distance), point, normal


def ray_cylinder_intersection(
    ray_origin,
    ray_direction,
    cylinder_position,
    cylinder_axis,
    cylinder_radius,
    max_distance=inf,
):
    """
    Return the nearest exact hit on a finite VPython cylinder.

    VPython cylinder.pos is the centre of its first circular cap and axis points
    from that cap to the centre of the second cap. Both the curved wall and the
    two circular caps are tested.
    """
    origin = _to_numpy(ray_origin)
    direction = _normalise(ray_direction)
    base = _to_numpy(cylinder_position)
    axis = _to_numpy(cylinder_axis)
    radius = float(cylinder_radius)
    length = np.linalg.norm(axis)

    if (
        np.linalg.norm(direction) < _EPSILON
        or length < _EPSILON
        or radius <= 0
    ):
        return None

    axis_direction = axis / length
    relative_origin = origin - base
    origin_axial = np.dot(relative_origin, axis_direction)
    direction_axial = np.dot(direction, axis_direction)

    origin_radial = relative_origin - origin_axial * axis_direction
    direction_radial = direction - direction_axial * axis_direction

    candidates = []

    # Curved wall: solve the quadratic in the plane perpendicular to the axis.
    a = np.dot(direction_radial, direction_radial)
    b = 2.0 * np.dot(origin_radial, direction_radial)
    c = np.dot(origin_radial, origin_radial) - radius * radius

    if a > _EPSILON:
        discriminant = b * b - 4.0 * a * c
        if discriminant >= 0:
            root = np.sqrt(max(0.0, discriminant))
            for distance in (
                (-b - root) / (2.0 * a),
                (-b + root) / (2.0 * a),
            ):
                if distance < 0 or distance > max_distance:
                    continue
                axial_position = origin_axial + distance * direction_axial
                if -_EPSILON <= axial_position <= length + _EPSILON:
                    point = origin + direction * distance
                    centre_line_point = base + axial_position * axis_direction
                    normal = _normalise(point - centre_line_point)
                    candidates.append((float(distance), point, normal))

    # Circular end caps.
    if abs(direction_axial) > _EPSILON:
        for cap_axial, cap_normal in (
            (0.0, -axis_direction),
            (length, axis_direction),
        ):
            distance = (cap_axial - origin_axial) / direction_axial
            if distance < 0 or distance > max_distance:
                continue

            point = origin + direction * distance
            cap_centre = base + cap_axial * axis_direction
            radial = point - cap_centre
            if np.dot(radial, radial) <= radius * radius + _EPSILON:
                candidates.append((float(distance), point, cap_normal.copy()))

    if not candidates:
        return None

    return min(candidates, key=lambda hit: hit[0])


def ray_obb_intersection(
    ray_origin,
    ray_direction,
    box_position,
    box_axis,
    box_up,
    box_width,
    box_height,
    max_distance=inf,
):
    """Return the nearest ray/oriented-box hit, or None."""
    origin = _to_numpy(ray_origin)
    direction = _normalise(ray_direction)
    centre = _to_numpy(box_position)
    axis = _to_numpy(box_axis)
    up = _to_numpy(box_up)

    length = np.linalg.norm(axis)
    if np.linalg.norm(direction) < _EPSILON or length < _EPSILON:
        return None

    local_x = axis / length
    up = up - np.dot(up, local_x) * local_x
    local_y = _normalise(up)

    if np.linalg.norm(local_y) < _EPSILON:
        fallback = np.array([0.0, 1.0, 0.0])
        if abs(np.dot(fallback, local_x)) > 0.99:
            fallback = np.array([0.0, 0.0, 1.0])
        local_y = _normalise(fallback - np.dot(fallback, local_x) * local_x)

    local_z = _normalise(np.cross(local_x, local_y))
    if np.linalg.norm(local_z) < _EPSILON:
        return None
    local_y = _normalise(np.cross(local_z, local_x))

    relative_origin = origin - centre
    basis = (local_x, local_y, local_z)
    local_origin = np.array([np.dot(relative_origin, value) for value in basis])
    local_direction = np.array([np.dot(direction, value) for value in basis])
    extents = np.array([
        length / 2.0,
        float(box_height) / 2.0,
        float(box_width) / 2.0,
    ])

    near_distance = -inf
    far_distance = inf
    near_axis = far_axis = -1
    near_sign = far_sign = 0.0

    for dimension in range(3):
        component_origin = local_origin[dimension]
        component_direction = local_direction[dimension]
        extent = extents[dimension]

        if abs(component_direction) < _EPSILON:
            if component_origin < -extent or component_origin > extent:
                return None
            continue

        first = (-extent - component_origin) / component_direction
        second = (extent - component_origin) / component_direction
        first_sign, second_sign = -1.0, 1.0

        if first > second:
            first, second = second, first
            first_sign, second_sign = second_sign, first_sign

        if first > near_distance:
            near_distance = first
            near_axis = dimension
            near_sign = first_sign

        if second < far_distance:
            far_distance = second
            far_axis = dimension
            far_sign = second_sign

        if near_distance > far_distance:
            return None

    if far_distance < 0:
        return None

    if near_distance >= 0:
        distance, hit_axis, hit_sign = near_distance, near_axis, near_sign
    else:
        distance, hit_axis, hit_sign = far_distance, far_axis, far_sign

    if distance > max_distance:
        return None

    point = origin + direction * distance
    local_normal = np.zeros(3, dtype=float)
    if hit_axis >= 0:
        local_normal[hit_axis] = hit_sign
    normal = _normalise(
        local_x * local_normal[0]
        + local_y * local_normal[1]
        + local_z * local_normal[2]
    )
    return float(distance), point, normal


def _visual_intersection(visual, origin, direction, max_distance):
    if _looks_like_sphere(visual):
        return ray_sphere_intersection(
            origin, direction, visual.pos, visual.radius, max_distance
        )

    if _looks_like_cylinder(visual):
        return ray_cylinder_intersection(
            origin,
            direction,
            visual.pos,
            visual.axis,
            visual.radius,
            max_distance,
        )

    if all(hasattr(visual, name) for name in ("pos", "axis", "width", "height")):
        return ray_obb_intersection(
            origin,
            direction,
            visual.pos,
            visual.axis,
            getattr(visual, "up", vec(0, 1, 0)),
            visual.width,
            visual.height,
            max_distance,
        )

    return None


def _collect_ignore_values(value, ignored_groups, ignored_wrappers, ignored_visuals):
    if value is None:
        return
    if _is_group(value):
        ignored_groups.add(id(value))
        return
    if _is_obj(value):
        ignored_wrappers.add(id(value))
        return
    if _is_iterable_container(value):
        for item in value:
            _collect_ignore_values(
                item, ignored_groups, ignored_wrappers, ignored_visuals
            )
        return
    ignored_visuals.add(id(value))


def build_ignore_sets(ignore=None):
    ignored_groups, ignored_wrappers, ignored_visuals = set(), set(), set()
    _collect_ignore_values(
        ignore, ignored_groups, ignored_wrappers, ignored_visuals
    )
    return ignored_groups, ignored_wrappers, ignored_visuals


def _collect_candidates(
    value,
    output,
    seen,
    ignored_groups,
    ignored_wrappers,
    ignored_visuals,
):
    if value is None or id(value) in seen:
        return
    seen.add(id(value))

    if _is_group(value):
        if id(value) in ignored_groups:
            return
        for child in value.children:
            _collect_candidates(
                child,
                output,
                seen,
                ignored_groups,
                ignored_wrappers,
                ignored_visuals,
            )
        for child_group in value.child_groups:
            _collect_candidates(
                child_group,
                output,
                seen,
                ignored_groups,
                ignored_wrappers,
                ignored_visuals,
            )
        return

    if _is_obj(value):
        if id(value) in ignored_wrappers or id(value.out) in ignored_visuals:
            return
        output.append((value, value.out))
        return

    if _is_iterable_container(value):
        for item in value:
            _collect_candidates(
                item,
                output,
                seen,
                ignored_groups,
                ignored_wrappers,
                ignored_visuals,
            )
        return

    # Raw VPython primitive.
    if id(value) not in ignored_visuals and hasattr(value, "pos"):
        output.append((value, value))


def raycast(
    groups,
    ray_origin,
    ray_direction,
    max_distance=inf,
    ignore=None,
    ignored_objects=None,
    debug=False,
):
    """
    Cast a ray against Group trees, Obj wrappers, raw VPython primitives, or
    nested iterables containing any of them.

    Obj wrappers with custom BoxCollider entries use those colliders only.
    Otherwise spheres and cylinders use exact rounded intersections and
    box-like objects use oriented-box intersections.
    """
    if ignored_objects is not None:
        ignore = ignored_objects if ignore is None else [ignore, ignored_objects]

    ignored_groups, ignored_wrappers, ignored_visuals = build_ignore_sets(ignore)
    candidates = []
    _collect_candidates(
        groups,
        candidates,
        set(),
        ignored_groups,
        ignored_wrappers,
        ignored_visuals,
    )

    origin = _to_numpy(ray_origin)
    direction = _normalise(ray_direction)
    if np.linalg.norm(direction) < _EPSILON:
        if debug:
            print("[raycast] zero-length direction")
        return None

    if debug:
        print(
            f"[raycast {RAYCASTING_VERSION}] collected "
            f"{len(candidates)} candidate object(s)"
        )

    nearest = None

    for index, (owner, visual) in enumerate(candidates):
        colliders = getattr(owner, "colliders", None) if _is_obj(owner) else None
        results = []

        if colliders:
            for collider in colliders:
                parameters = _collider_world_transform(owner, collider)
                if parameters is None:
                    continue
                result = ray_obb_intersection(
                    origin, direction, *parameters, max_distance=max_distance
                )
                if result is not None:
                    results.append((result, collider))
        else:
            result = _visual_intersection(
                visual, origin, direction, max_distance
            )
            if result is not None:
                results.append((result, None))

        if debug:
            print(
                f"[raycast] candidate {index}: "
                f"type={_visual_type_name(visual)}, hits={len(results)}"
            )

        for (distance, point, normal), collider in results:
            if nearest is None or distance < nearest.distance:
                nearest = RayHit(
                    obj=owner,
                    visual=visual,
                    distance=distance,
                    point=_to_vpython(point),
                    normal=_to_vpython(normal),
                    collider=collider,
                )

    if debug:
        if nearest is None:
            print("[raycast] result: no hit")
        else:
            print(
                f"[raycast] result: {_visual_type_name(nearest.visual)} "
                f"at distance {nearest.distance}"
            )

    return nearest
