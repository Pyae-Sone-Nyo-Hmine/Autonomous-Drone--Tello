import cv2
import cv2.aruco as aruco
import keyboard as kb
from utils.MarkerClass import *
from time import sleep


# detects the ar marker and gives values of 4 corners and ids
def ar_detector(img, draw_box_id=True):
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    aruco_param = aruco.DetectorParameters_create()
    corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_param)

    # draw the border and id on the marker
    if draw_box_id:
        aruco.drawDetectedMarkers(img, corners, ids)

    return corners, ids


# prints out average length and centroid values from the class Marker
def print_avg_length_centroid(my_ar):
    try:
        my_ar.average_length_func()
        my_ar.centroid_func()
        print("Average length: " + str(my_ar.average_length))
        print("Centroid: " + str(my_ar.centroid))

    except:
        print("AR marker not detected")


# prints out the position and orientation with respect to the ar marker
def print_pos_and_ori(my_ar):
    try:
        my_ar.average_length_func()
        my_ar.centroid_func()
        my_ar.position_func()
        my_ar.orientation_func()
        print(my_ar.position + " " + my_ar.orientation)

    except:
        print("AR marker not Detected")


# control motion using keys from keyboard
def motion_func(my_drone):
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if kb.is_pressed('left'):
        lr = -speed

    elif kb.is_pressed('right'):
        lr = speed

    if kb.is_pressed('up'):
        fb = speed

    elif kb.is_pressed('down'):
        fb = -speed

    if kb.is_pressed('w'):
        ud = speed

    elif kb.is_pressed('s'):
        ud = -speed

    if kb.is_pressed('a'):
        yv = -speed - 20

    elif kb.is_pressed('d'):
        yv = speed + 20

    if kb.is_pressed('q'):
        my_drone.land()

    if kb.is_pressed('e'):
        my_drone.takeoff()

    return [lr, fb, ud, yv]


# orient the up and down speed with respect to the marker
def ar_marker_up_down_with_PID(my_ar, kp, ki, kd, prev_ud_error):
    my_ar.centroid_func()

    ud_error = my_ar.centroid[0] - 360
    ud_speed = kp * ud_error + ki * ud_error + kd * (ud_error - prev_ud_error)
    ud_speed = int(np.clip(ud_speed, -100, 100))

    return ud_error, ud_speed


# orient the yaw speed with respect to the marker
def ar_marker_yaw_with_PID(my_ar, kp, ki, kd, prev_yaw_error):
    my_ar.centroid_func()
    yaw_error = my_ar.centroid[1] - 240
    yaw_speed = kp * yaw_error + ki * yaw_error + kd * (yaw_error - prev_yaw_error)
    yaw_speed = -(int(np.clip(yaw_speed, -100, 100)))

    return yaw_error, yaw_speed


# follow the ar tag at a respectable distance
def follow_ar_with_PID(my_ar, kp, ki, kd, prev_fb_error):
    my_ar.average_length_func()

    fb_error = my_ar.average_length - 120
    fb_speed = kp * fb_error + ki * fb_error + kd * (fb_error - prev_fb_error)
    fb_speed = -(int(np.clip(fb_speed, -100, 100)))

    return fb_error, fb_speed


# orient the left right speed using the slope of the ar marker
def slope_orientation_with_PID(my_ar, kp, ki, kd, prev_s_error):
    my_ar.orientation_func()

    s_error = my_ar.slope_x
    s_speed = kp * s_error + ki * s_error + kd * (s_error - prev_s_error)
    s_speed = int(np.clip(s_speed, -100, 100))

    if abs(my_ar.slope_y) > 0.1:
        s_speed = 0

    return s_error, - s_speed


# take pictures if p is pressed every 100 milliseconds
def take_picture(img, take):
    if kb.is_pressed("p"):
        cv2.imwrite("/Users/pyaey/OneDrive/Desktop/Tello/" + "tello_image" + str(take) + ".png", img)
        sleep(0.1)

        take += 1

    return take


# save video of tello drone
def take_video( img):
    video = cv2.VideoWriter("tello_video.avi", cv2.VideoWriter_fourcc(*'MJPG'), 80, (720, 480))
    if kb.is_pressed('v'):
        video.write(img)
