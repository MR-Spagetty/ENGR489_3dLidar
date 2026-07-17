from vpython import *

LASER_ROT_VEL = radians(1)

# g = group()

laser_trail_source = sphere( make_trail=True, trail_type="curve", trail_radius=1, interval=1, retain = 200 , color=color.red, opacity=0)
laser_pose = arrow(pos=vec(0,0,0), axis=vec(1,0,0))
laser_pose.length = 20
# centralPlatform = compound([laser_pose])

while True:
    rate(100)
    laser_pose.axis = rotate(laser_pose.axis, LASER_ROT_VEL, axis=vec(0,1,0))
    laser_trail_source.pos = laser_pose.pos + laser_pose.axis*laser_pose.length
