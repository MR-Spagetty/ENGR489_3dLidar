# raycasting.py

from dataclasses import dataclass, field
from collections.abc import Iterable

import numpy as np
from vpython import vec, vector

RAYCASTING_VERSION = "2026-07-30-recursive-collection-v3"


@dataclass
class BoxCollider:
    """
    A non-rendered box collider defined in the local coordinate system of an obj.

    axis determines the collider's length and local X direction.
    up determines its local Y direction.
    width extends along its local Z direction.
    height extends along its local Y direction.
    """

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
    """Convert a VPython vector or array-like value into a NumPy vector."""
    if hasattr(value, "x"):
        return np.array([value.x, value.y, value.z], dtype=float)

    return np.asarray(value, dtype=float)


def _to_vpython(value) -> vector:
    """Convert an array-like value into a VPython vector."""
    return vec(float(value[0]), float(value[1]), float(value[2]))


def _normalise(value: np.ndarray) -> np.ndarray:
    magnitude = np.linalg.norm(value)

    if magnitude < 1e-12:
        return np.zeros(3, dtype=float)

    return value / magnitude


def _object_basis(wrapper):
    """
    Build the object's world-space basis from the rendered VPython object.

    The rendered object's transform is the source of truth. Cached working_*
    values can still be zero before the first prop() call or stale if raycast()
    is called between transform updates.

    Returns:

        forward: local X direction in world space
        up:      local Y direction in world space
        side:    local Z direction in world space
    """
    visual = wrapper.out
    forward = _normalise(_to_numpy(visual.axis))
    up = _normalise(_to_numpy(visual.up))

    if np.linalg.norm(forward) < 1e-12:
        return None

    # Remove any component of up that points along forward.
    up = up - np.dot(up, forward) * forward
    up = _normalise(up)

    if np.linalg.norm(up) < 1e-12:
        fallback = np.array([0.0, 1.0, 0.0])

        if abs(np.dot(fallback, forward)) > 0.99:
            fallback = np.array([0.0, 0.0, 1.0])

        up = fallback - np.dot(fallback, forward) * forward
        up = _normalise(up)

    # VPython's width direction corresponds to axis × up.
    side = _normalise(np.cross(forward, up))

    if np.linalg.norm(side) < 1e-12:
        return None

    # Recalculate up to ensure the basis is orthogonal.
    up = _normalise(np.cross(side, forward))

    return forward, up, side


def _local_vector_to_world(local_vector, basis) -> np.ndarray:
    """Rotate a vector from an obj's local space into world space."""
    forward, up, side = basis
    local = _to_numpy(local_vector)

    return (
        forward * local[0]
        + up * local[1]
        + side * local[2]
    )


def _collider_world_transform(wrapper, collider: BoxCollider):
    """
    Convert one local collider into world-space box parameters.

    This uses workingPos, workingAxis and workingUp produced by the existing
    group propagation code. It does not alter or replace prop().
    """
    basis = _object_basis(wrapper)

    if basis is None:
        return None

    wrapper_position = _to_numpy(wrapper.out.pos)

    world_position = (
        wrapper_position
        + _local_vector_to_world(collider.pos, basis)
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


def ray_obb_intersection(
    ray_origin,
    ray_direction,
    box_position,
    box_axis,
    box_up,
    box_width,
    box_height,
    max_distance=float("inf"),
):
    """
    Intersect a ray with an oriented bounding box.

    Returns:

        distance, hit_position, hit_normal

    or None when there is no intersection.
    """
    origin = _to_numpy(ray_origin)
    direction = _normalise(_to_numpy(ray_direction))

    centre = _to_numpy(box_position)
    axis = _to_numpy(box_axis)
    up = _to_numpy(box_up)

    if np.linalg.norm(direction) < 1e-12:
        return None

    box_length = np.linalg.norm(axis)

    if box_length < 1e-12:
        return None

    local_x = axis / box_length

    up = up - np.dot(up, local_x) * local_x
    local_y = _normalise(up)

    if np.linalg.norm(local_y) < 1e-12:
        fallback = np.array([0.0, 1.0, 0.0])

        if abs(np.dot(fallback, local_x)) > 0.99:
            fallback = np.array([0.0, 0.0, 1.0])

        local_y = fallback - np.dot(fallback, local_x) * local_x
        local_y = _normalise(local_y)

    local_z = _normalise(np.cross(local_x, local_y))

    if np.linalg.norm(local_z) < 1e-12:
        return None

    local_y = _normalise(np.cross(local_z, local_x))

    relative_origin = origin - centre

    local_origin = np.array([
        np.dot(relative_origin, local_x),
        np.dot(relative_origin, local_y),
        np.dot(relative_origin, local_z),
    ])

    local_direction = np.array([
        np.dot(direction, local_x),
        np.dot(direction, local_y),
        np.dot(direction, local_z),
    ])

    half_extents = np.array([
        box_length / 2.0,
        box_height / 2.0,
        box_width / 2.0,
    ])

    near_distance = -np.inf
    far_distance = np.inf

    near_axis = -1
    near_sign = 0.0

    far_axis = -1
    far_sign = 0.0

    epsilon = 1e-10

    for dimension in range(3):
        component_origin = local_origin[dimension]
        component_direction = local_direction[dimension]
        extent = half_extents[dimension]

        if abs(component_direction) < epsilon:
            if component_origin < -extent or component_origin > extent:
                return None

            continue

        first = (
            -extent - component_origin
        ) / component_direction

        second = (
            extent - component_origin
        ) / component_direction

        first_sign = -1.0
        second_sign = 1.0

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
        distance = near_distance
        hit_axis = near_axis
        hit_sign = near_sign
    else:
        # Ray started inside the box, so use the exit face.
        distance = far_distance
        hit_axis = far_axis
        hit_sign = far_sign

    if distance < 0 or distance > max_distance:
        return None

    hit_position = origin + direction * distance

    local_normal = np.zeros(3, dtype=float)

    if hit_axis >= 0:
        local_normal[hit_axis] = hit_sign

    world_normal = (
        local_x * local_normal[0]
        + local_y * local_normal[1]
        + local_z * local_normal[2]
    )

    world_normal = _normalise(world_normal)

    return (
        float(distance),
        hit_position,
        world_normal,
    )


def _visual_box_parameters(wrapper):
    """
    Return the world-space box parameters for an ordinary object without custom
    colliders.

    The visual object's width and height are used, while workingPos,
    workingAxis and workingUp come from the existing prop() method.
    """
    visual = wrapper.out if hasattr(wrapper, "out") else wrapper

    if not hasattr(visual, "width") or not hasattr(visual, "height"):
        return None

    return (
        _to_numpy(visual.pos),
        _to_numpy(visual.axis),
        _to_numpy(visual.up),
        float(visual.width),
        float(visual.height),
    )


def _is_group(value) -> bool:
    """
    Detect a group without importing or changing the existing group class.
    """
    return hasattr(value, "children") and not hasattr(value, "out")


def _is_visual(value) -> bool:
    """Return True for a raw VPython-like render object."""
    return (
        hasattr(value, "pos")
        and hasattr(value, "axis")
        and not _is_group(value)
        and not hasattr(value, "out")
    )


def _is_obj(value) -> bool:
    """
    Detect an obj wrapper without importing or changing the existing obj class.
    """
    return hasattr(value, "out") and (hasattr(value, "working_pos") or hasattr(value, "workingPos"))


def _collect_ignore_values(
    value,
    ignored_groups,
    ignored_wrappers,
    ignored_visuals,
):
    if value is None:
        return

    if _is_group(value):
        ignored_groups.add(id(value))
        return

    if _is_obj(value):
        ignored_wrappers.add(id(value))
        return

    if _is_visual(value):
        ignored_visuals.add(id(value))
        return

    if isinstance(value, Iterable) and not isinstance(
        value,
        (str, bytes),
    ):
        for item in value:
            _collect_ignore_values(
                item,
                ignored_groups,
                ignored_wrappers,
                ignored_visuals,
            )

        return

    ignored_visuals.add(id(value))


def build_ignore_sets(ignore=None):
    ignored_groups = set()
    ignored_wrappers = set()
    ignored_visuals = set()

    _collect_ignore_values(
        ignore,
        ignored_groups,
        ignored_wrappers,
        ignored_visuals,
    )

    return (
        ignored_groups,
        ignored_wrappers,
        ignored_visuals,
    )


def _collect_candidates(
    value,
    output,
    ignored_groups,
    ignored_wrappers,
    ignored_visuals,
    visited,
):
    """Recursively collect groups, Obj wrappers, visuals and nested iterables."""
    if value is None:
        return

    value_id = id(value)
    if value_id in visited:
        return

    if _is_group(value):
        visited.add(value_id)
        if value_id in ignored_groups:
            return

        for child in getattr(value, "children", []):
            _collect_candidates(
                child, output, ignored_groups, ignored_wrappers,
                ignored_visuals, visited,
            )

        child_groups = getattr(value, "child_groups", None)
        if child_groups is None:
            child_groups = getattr(value, "childGroups", [])

        for child_group in child_groups:
            _collect_candidates(
                child_group, output, ignored_groups, ignored_wrappers,
                ignored_visuals, visited,
            )
        return

    if _is_obj(value):
        visited.add(value_id)
        if value_id in ignored_wrappers or id(value.out) in ignored_visuals:
            return
        output.append(value)
        return

    if _is_visual(value):
        visited.add(value_id)
        if value_id not in ignored_visuals:
            output.append(value)
        return

    if isinstance(value, Iterable) and not isinstance(value, (str, bytes)):
        visited.add(value_id)
        for item in value:
            _collect_candidates(
                item, output, ignored_groups, ignored_wrappers,
                ignored_visuals, visited,
            )


def raycast(
    groups,
    ray_origin,
    ray_direction,
    max_distance=float("inf"),
    ignore=None,
    ignored_objects=None,
    debug=False,
):
    """
    Cast a ray against all objects in the supplied groups.

    When an obj has custom BoxCollider entries, each collider is transformed
    into world space and tested separately.

    When an obj has no custom colliders, its normal visual bounding box is used.

    Both `ignore` and `ignored_objects` are accepted for compatibility.
    """
    if ignored_objects is not None:
        if ignore is None:
            ignore = ignored_objects
        else:
            ignore = [ignore, ignored_objects]

    if _is_group(groups) or _is_obj(groups) or _is_visual(groups):
        groups = [groups]

    ignored_groups, ignored_wrappers, ignored_visuals = (
        build_ignore_sets(ignore)
    )

    wrappers = []
    _collect_candidates(
        groups,
        wrappers,
        ignored_groups,
        ignored_wrappers,
        ignored_visuals,
        set(),
    )

    if debug:
        print(
            f"[raycast] input type={type(groups).__name__}; "
            f"collected {len(wrappers)} candidate object(s)"
        )

    origin = _to_numpy(ray_origin)
    direction = _normalise(_to_numpy(ray_direction))

    if np.linalg.norm(direction) < 1e-12:
        return None

    nearest_hit = None

    for index, wrapper in enumerate(wrappers):
        visual = wrapper.out if hasattr(wrapper, "out") else wrapper
        colliders = getattr(wrapper, "colliders", None)

        if debug:
            print(
                f"[raycast] candidate {index}: type={type(visual).__name__}, "
                f"pos={getattr(visual, 'pos', None)}, "
                f"axis={getattr(visual, 'axis', None)}, "
                f"width={getattr(visual, 'width', None)}, "
                f"height={getattr(visual, 'height', None)}, "
                f"colliders={len(colliders) if colliders else 0}"
            )

        if colliders:
            for collider in colliders:
                world_parameters = _collider_world_transform(
                    wrapper,
                    collider,
                )

                if world_parameters is None:
                    continue

                (
                    collider_position,
                    collider_axis,
                    collider_up,
                    collider_width,
                    collider_height,
                ) = world_parameters

                result = ray_obb_intersection(
                    ray_origin=origin,
                    ray_direction=direction,
                    box_position=collider_position,
                    box_axis=collider_axis,
                    box_up=collider_up,
                    box_width=collider_width,
                    box_height=collider_height,
                    max_distance=max_distance,
                )

                if result is None:
                    continue

                distance, point, normal = result

                if (
                    nearest_hit is None
                    or distance < nearest_hit.distance
                ):
                    nearest_hit = RayHit(
                        obj=wrapper,
                        visual=visual,
                        collider=collider,
                        distance=distance,
                        point=_to_vpython(point),
                        normal=_to_vpython(normal),
                    )

            # Do not also raycast against the compound's overall bounding box.
            continue

        parameters = _visual_box_parameters(wrapper)

        if parameters is None:
            if debug:
                print(f"[raycast] candidate {index} skipped: no width/height")
            continue

        (
            visual_position,
            visual_axis,
            visual_up,
            visual_width,
            visual_height,
        ) = parameters

        result = ray_obb_intersection(
            ray_origin=origin,
            ray_direction=direction,
            box_position=visual_position,
            box_axis=visual_axis,
            box_up=visual_up,
            box_width=visual_width,
            box_height=visual_height,
            max_distance=max_distance,
        )

        if result is None:
            if debug:
                print(f"[raycast] candidate {index}: no intersection")
            continue

        distance, point, normal = result

        if nearest_hit is None or distance < nearest_hit.distance:
            nearest_hit = RayHit(
                obj=wrapper,
                visual=visual,
                collider=None,
                distance=distance,
                point=_to_vpython(point),
                normal=_to_vpython(normal),
            )

    if debug:
        if nearest_hit is None:
            print("[raycast] result: no hit")
        else:
            print(
                f"[raycast] result: hit at distance {nearest_hit.distance}, "
                f"point={nearest_hit.point}"
            )

    return nearest_hit
