from vpython import *
import numpy as np

RATE = 100

LASER_ROT_VEL = radians(1)

def vecToArr(vec):
    return np.array([vec.x, vec.y, vec.z])

class obj:
    def __init__(self, ref, controlled) -> None:
        self.ref = ref
        self.workingPos = np.array([0, 0, 0])
        self.workingAxis = np.array([0, 0, 0])
        self.out = controlled

class group():
    """docstring for group."""
    def __init__(self, parent: group|None, children: list[obj]):
        super(group, self).__init__()
        self.parent = parent
        self.children = children
        self.offset = vec(0, 0, 0)
        self.rotX = 0
        self.rotY = 0
        self.rotZ = 0

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

    def apply(self, children: None|list[obj] = None):
        frm_child = False
        if children is None:
            children = self.children
            for child in children:
                child.workingPos = vecToArr(child.ref.pos)
                child.workingAxis = vecToArr(child.ref.axis)
        else:
            frm_child = True

        for child in children:
            rot = self.get_rot()
            child.workingAxis = rot @ (child.workingAxis - child.workingPos) + child.workingPos
            child.workingPos = rot @ child.workingPos + vecToArr(self.offset)

        if self.parent is not None:
            self.parent.apply(children)
        if frm_child:
            return
        for child in children:
            child.out.pos = vec(*child.workingPos)
            child.out.axis = vec(*child.workingAxis)



# g = group()

laser_trail_source = sphere( make_trail=True, trail_type="curve", trail_radius=1, interval=1, retain = 200 , color=color.red, opacity=0)
laser_pose = arrow(pos=vec(0,0,0), axis=vec(1,0,0))
laser_pose.length = 20
laser_pose_ref = arrow(pos=vec(0, 0, 0), axis=vec(1, 0, 0))
# centralPlatform = compound([laser_pose])

altYaw = group(None, [])
pitch = group(altYaw, [])
yaw = group(pitch, [obj(laser_pose_ref, laser_pose)])


groups = [
        yaw, pitch, altYaw
        ]
while True:
    rate(RATE)

    for g in groups:
        g.apply()

    laser_trail_source.pos = laser_pose.pos + laser_pose.axis*laser_pose.length
