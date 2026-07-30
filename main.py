from vpython import *
from groups import *
from utils import *
from raycasting import raycast, BoxCollider
import numpy as np

RATE = 8000
LASER_DIST = 20
LASER_FREQ = 10 # Hz
LASER_ROT_VEL_RPM = 363.6
LASER_ROT_VEL_RAD_PER_SEC = LASER_ROT_VEL_RPM/60 * radians(360)
LASER_SAMPLE_RATE_Hz = 8000 # Hz
LASER_POINT_RETENTION = 4000

ALT_YAW_EXTRA = 0 # extra offset from alt yaw system for future

LASER_ROT_VEL = LASER_ROT_VEL_RAD_PER_SEC / RATE
LASER_POINT_RATE = RATE / min(RATE, LASER_SAMPLE_RATE_Hz)

def point(pos=vec(0, 0, 0), color=color.white, axis = vec(1, 0, 0)):
    return sphere(pos=pos, color=color, axis=axis)

origin_x = arrow(axis=vec(3,0,0), color=color.green)
origin_y = arrow(axis=vec(0,3,0), color=color.yellow)
origin_y = arrow(axis=vec(0,0,3), color=color.blue)

laser_trail_source = sphere( make_trail=True, trail_type="points", trail_radius=0.1, interval=1, retain = LASER_POINT_RETENTION , color=color.red, opacity=0)
laser_pose = arrow(shaftwidth=0.2)
laser_pose.color = color.red

lidar_center_pos = vec(0, 0, 0)

env = group(None, [
    obj(point(pos = vec (20, 0, 0), axis=vec(0, 10, 0)), cylinder(radius=3)),
    obj(point(pos = vec (20, 15, 0)), sphere(radius=3))
    ])

frame_colliders = [
    # Front cross rail
    BoxCollider(
        pos=vec(0, 0, 0),
        axis=vec(mm_to_cm(48), 0, 0),
        width=mm_to_cm(265),
        height=mm_to_cm(48),
    ),

    # Rear cross rail
    BoxCollider(
        pos=vec(
            mm_to_cm((-432 + 48) / 2 + 24),
            0,
            0,
        ),
        axis=vec(mm_to_cm(48), 0, 0),
        width=mm_to_cm(265),
        height=mm_to_cm(48),
    ),

    # Left longitudinal rail
    BoxCollider(
        pos=vec(
            0,
            0,
            mm_to_cm(48 / 2 + 265 / 2),
        ),
        axis=vec(mm_to_cm(432), 0, 0),
        width=mm_to_cm(48),
        height=mm_to_cm(48),
    ),

    # Right longitudinal rail
    BoxCollider(
        pos=vec(
            0,
            0,
            -mm_to_cm(48 / 2 + 265 / 2),
        ),
        axis=vec(mm_to_cm(432), 0, 0),
        width=mm_to_cm(48),
        height=mm_to_cm(48),
    ),

    # Vertical mast
    BoxCollider(
        pos=vec(
            0,
            mm_to_cm((48 + 144) / 2),
            0,
        ),
        axis=vec(0, mm_to_cm(144), 0),
        width=mm_to_cm(24),
        height=mm_to_cm(24),
        up=vec(0,0,1)
    ),
]

frame_visual = compound(
    [
        box(
            pos=collider.pos,
            axis=collider.axis,
            up=collider.up,
            width=collider.width,
            height=collider.height,
        )
        for collider in frame_colliders
    ],
    origin=vec(0, 0, 0),
    color=color.gray(0.6),
)

robot = group(env, [
    obj(point(pos=vec(0, 0, 0), axis=vec(1, 0, 0)), frame_visual, colliders=frame_colliders),
    obj(point(pos=vec(mm_to_cm((-432+130)/2), mm_to_cm((48+95)/2), mm_to_cm(48/2+265/2)), axis=vec(0, mm_to_cm(95), 0)), box(width=mm_to_cm(65), height=mm_to_cm(130), color=rgb_col(200, 30, 30))),
    obj(point(pos=vec(mm_to_cm((-432-3)/2+24), mm_to_cm((150-48)/2), 0), axis=vec(mm_to_cm(3), 0, 0)), box(width=mm_to_cm(150), height=mm_to_cm(150), color=rgb_col(200, 30, 30)))
])

wheel_PB = group(robot, [
    ])
wheel_PB.offset=vec(1, 0, 1)
wheel_PS = group(robot, [
    ])
wheel_PS.offset=vec(-1, 0, 1)

altYaw = group(robot, [
    obj(point(pos=vec(0, mm_to_cm(5/2), 0), axis=vec(0, mm_to_cm(5), 0)), box(width=mm_to_cm(30), height=mm_to_cm(30)))
    ])
altYaw.offset = vec(0, mm_to_cm(144+48/2+ALT_YAW_EXTRA), 0)

pitch = group(altYaw, [
    obj(point(pos=vec(0, mm_to_cm(-5/2), 0), axis=vec(0, mm_to_cm(5), 0)), cylinder(radius=mm_to_cm(133/2))),
    obj(point(pos=lidar_center_pos+vec(0, mm_to_cm((31.5+5)/2), 0), axis=vec(mm_to_cm(70.28), 0, 0)), box(height=mm_to_cm(31.5), width=mm_to_cm(70.28)))
    ])
pitch.offset = vec(0, mm_to_cm(76.9102+14.5), 0)

yaw = group(pitch, [
    obj(arrow(pos=vec(0, mm_to_cm(42-31.5), 0), axis=vec(1, 0, 0)*15), laser_pose),
    obj(point(pos = vec(0, 0, 0), axis = vec(0, mm_to_cm(51-31.5), 0)), cylinder(radius=mm_to_cm(70.04/2), color=color.blue))
    ])
yaw.offset = lidar_center_pos + vec(0, mm_to_cm(31.5+5/2), 0)

tick = 0
while True:
    rate(RATE)

    yaw.rotY += LASER_ROT_VEL
    pitch.rotZ -= radians(10)/RATE

    env.prop()
    tick += 1
    if tick >= LASER_POINT_RATE:
        tick = 0
        hit = raycast(groups=env, ray_origin=laser_pose.pos, ray_direction=laser_pose.axis.norm(), max_distance=1000, ignored_objects=[yaw])
        if hit is not None:
            laser_trail_source.pos = hit.point
