from vpython import *
from utils import mm_to_cm, rgb_col, point
from groups import group, obj

env = group(None, [
    ])
def table(pos = vec(0, 0, 0)):
    # width 160, depth 80 inc curve thickness 3 therefore cyrve diam is 1.5
    # leg height 72 width 5 5.5 in from edge
    table = group(env, [
        obj(point(pos = vec(0, 72+1.5, -1.5/2), axis=vec(0, 3, 0)), box(height=160, width=80-1.5)),
        obj(point(pos = vec(-80, 72+1.5,40-1.5), axis=vec(160, 0, 0)), cylinder(radius=1.5)),
        obj(point(pos = vec(-80+2.5, 72/2, -40+2.5), axis=vec(0, 72, 0)), box(width=5, height=5, color=color.gray(0.2))),
        obj(point(pos = vec(80-2.5, 72/2, -40+2.5), axis=vec(0, 72, 0)), box(width=5, height=5, color=color.gray(0.2))),
        obj(point(pos = vec(-80+2.5, 72/2, 40-2.5-5.5), axis=vec(0, 72, 0)), box(width=5, height=5, color=color.gray(0.2))),
        obj(point(pos = vec(80-2.5, 72/2, 40-2.5-5.5), axis=vec(0, 72, 0)), box(width=5, height=5, color=color.gray(0.2))),
        obj(point(pos = vec(0, 72-2.5, 40-2.5/2-5.5), axis=vec(150, 0, 0)), box(width=2.5, height=5, color=color.gray(0.2))),
        obj(point(pos = vec(0, 72-2.5, -40+2.5/2), axis=vec(150, 0, 0)), box(width=2.5, height=5, color=color.gray(0.2))),
        obj(point(pos = vec(80-2.5/2, 72-2.5, -2.75), axis=vec(0, 0, 65)), box(width=2.5, height=5, color=color.gray(0.2))),
        obj(point(pos = vec(-80+2.5/2, 72-2.5, -2.75), axis=vec(0, 0, 65)), box(width=2.5, height=5, color=color.gray(0.2))),

        obj(point(pos = vec(0, 56+mm_to_cm(1), -40+5+4), axis=vec(0, mm_to_cm(2), 0)), box(height=160, width=8, color=color.gray(0.2))),
        obj(point(pos = vec(0, 56+1.5, -40+5+8-0.1), axis=vec(0, 3, 0)), box(height=160, width=mm_to_cm(2), color=color.gray(0.2))),
        obj(point(pos = vec(0, 56+4, -40+5+0.1), axis=vec(0, 8, 0)), box(height=160, width=mm_to_cm(2), color=color.gray(0.2))),
        ])
    table.offset = pos
    return table

table(vec(  0, 0, -90))
table(vec(160, 0, -90))
table(vec(320, 0, -90))
table(vec(380, 0, -90))
