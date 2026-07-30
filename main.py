from vpython import *
import numpy as np

RATE = 8000
LASER_DIST = 20
LASER_FREQ = 10 # Hz
LASER_ROT_VEL_RPM = 363.6
LASER_ROT_VEL_RAD_PER_SEC = LASER_ROT_VEL_RPM/60 * radians(360)
LASER_SAMPLE_RATE_Hz = 8000 # Hz
LASER_POINT_RETENTION = 40000

ALT_YAW_EXTRA = 0 # extra offset from alt yaw system for future

def mm_to_cm(mm):
    return mm/10

def rgbCol(r: int, g:int, b:int):
    return vec(r, g, b)/255

LASER_ROT_VEL = LASER_ROT_VEL_RAD_PER_SEC / RATE
LASER_POINT_RATE = RATE / min(RATE, LASER_SAMPLE_RATE_Hz)
def vecToArr(vec):
    return np.array([vec.x, vec.y, vec.z])

class obj:
    def __init__(self, ref, controlled) -> None:
        self.ref = ref
        ref.visible = False
        self.workingPos = np.array([0, 0, 0])
        self.workingAxis = np.array([0, 0, 0])
        self.workingUp = np.array([0, 0, 0])
        self.out = controlled

class group():
    """docstring for group."""
    def __init__(self, parent: group|None, children: list[obj]):
        super(group, self).__init__()
        self.parent = parent
        self.children = children
        self.offset = vec(0, 0, 0)
        self.rotX = 0.
        self.rotY = 0.
        self.rotZ = 0.
        self.childGroups = []
        if (parent is not None):
            parent.childGroups.append(self)

    def get_rot(self):

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(self.rotX), -np.sin(self.rotX)],
            [0, np.sin(self.rotX), np.cos(self.rotX)]
            ])
        R_y = np.array([
            [np.cos(self.rotY), 0, np.sin(self.rotY)],
            [0, 1, 0],
            [-np.sin(self.rotY), 0, np.cos(self.rotY)]
            ])
        R_z = np.array([
            [np.cos(self.rotZ), -np.sin(self.rotZ), 0],
            [np.sin(self.rotZ), np.cos(self.rotZ), 0],
            [0, 0, 1]
            ])
        R_combined = R_x @ R_y @ R_z
        return R_combined

    def prop(self):
        for child in self.childGroups:
            child.prop()
        self.apply()

    def apply(self, children: None|list[obj] = None):
        frm_child = False
        if children is None:
            children = self.children
            for child in children:
                child.workingPos = vecToArr(child.ref.pos)
                child.workingUp = vecToArr(child.ref.up)
                child.workingAxis = vecToArr(child.ref.axis)
        else:
            frm_child = True

        for child in children:
            rot = self.get_rot()
            child.workingAxis = rot @ child.workingAxis
            child.workingUp = rot @ child.workingUp
            child.workingPos = rot @ child.workingPos + vecToArr(self.offset)

        if self.parent is not None:
            self.parent.apply(children)
        if frm_child:
            return
        for child in children:
            child.out.pos = vec(*child.workingPos)
            child.out.axis = vec(*child.workingAxis)
            child.out.up = vec(*child.workingUp)


def point(pos=vec(0, 0, 0), color=color.white, axis = vec(1, 0, 0)):
    return sphere(pos=pos, color=color, axis=axis)

origin_x = arrow(axis=vec(3,0,0), color=color.green)
origin_y = arrow(axis=vec(0,3,0), color=color.yellow)
origin_y = arrow(axis=vec(0,0,3), color=color.blue)

laser_trail_source = sphere( make_trail=True, trail_type="points", trail_radius=0.1, interval=LASER_POINT_RATE, retain = LASER_POINT_RETENTION , color=color.red, opacity=0)
laser_pose = arrow(shaftwidth=0.2)
laser_pose.color = color.red

lidar_center_pos = vec(0, 0, 0)

robot = group(None, [
    obj(point(), compound([
        box(axis=vec(mm_to_cm(48), 0, 0), width=mm_to_cm(265), height=mm_to_cm(48)),
        box(width=mm_to_cm(265), height=mm_to_cm(48), pos=vec(mm_to_cm((-432+48)/2+24), 0, 0), axis=vec(mm_to_cm(48), 0, 0)),
        box(pos= vec(0, 0, mm_to_cm(48/2 + 265/2)), axis=vec(mm_to_cm(432), 0, 0), width=mm_to_cm(48), height=mm_to_cm(48)),
        box(pos= vec(0, 0, -mm_to_cm(48/2 + 265/2)), axis=vec(mm_to_cm(432), 0, 0), width=mm_to_cm(48), height=mm_to_cm(48)),
        box(pos=vec(0, mm_to_cm((48+144)/2), 0), axis=vec(0, mm_to_cm(144), 0), width=mm_to_cm(24), height=mm_to_cm(24))
        ], color=color.gray(0.6), origin = vec(0, 0, 0)))
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


while True:
    rate(RATE)

    yaw.rotY += LASER_ROT_VEL
    pitch.rotZ += radians(10)/RATE

    robot.prop()

    laser_trail_source.pos = laser_pose.pos + laser_pose.axis.norm()* LASER_DIST
