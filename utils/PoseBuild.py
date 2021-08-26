import numpy as np
from utils.PoseClass import *


def follow_man_with_PID(man, kp, ki, kd, prev_fb_error):
    if man.landmarks:
        man.torso_area()

        fb_error = 14000 - man.torso_area
        fb_speed = kp * fb_error + ki * fb_error + kd * (fb_error - prev_fb_error)
        fb_speed = int(np.clip(fb_speed, -100, 100))

    else:
        fb_error = fb_speed = 0
    return fb_error, fb_speed


def man_up_down_with_PID(man, kp, ki, kd, prev_ud_error):
    if man.landmarks:

        ud_error = man.y / 2 - man.nose[1]
        ud_speed = kp * ud_error + ki * ud_error + kd * (ud_error - prev_ud_error)
        ud_speed = int(np.clip(ud_speed, -100, 100))

    else:
        ud_error = ud_speed = 0

    return ud_error, ud_speed


def man_yaw_with_PID(man, kp, ki, kd, prev_yaw_error):
    if man.landmarks:

        yw_error = man.x / 2 - man.nose[0]
        yw_speed = kp * yw_error + ki * yw_error + kd * (yw_error - prev_yaw_error)
        yw_speed = -int(np.clip(yw_speed, -100, 100))

    else:
        yw_error = yw_speed = 0

    return yw_error, yw_speed
