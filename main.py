from djitellopy import tello
from utils.MarkerBuild import *
from utils.PoseBuild import *

# Initialize my_drone
my_drone = tello.Tello()

# Connect to my_drone
my_drone.connect()

# Check battery
print("Battery : " + str(my_drone.get_battery()))

# Turn on video streaming
my_drone.streamon()

# counter for taking picture and video
TAKE = 0


def flight_mode(take=TAKE):
    # MAIN LOOP
    while True:
        img = my_drone.get_frame_read().frame
        img = cv2.resize(img, (720, 480))

        cv2.imshow("Tello Stream", img)

        take = take_picture(img, take)
        take_video(image = img)

        cv2.waitKey(1)

        values = motion_func(my_drone=my_drone)
        my_drone.send_rc_control(values[0], values[1], values[2], values[3])

        if kb.is_pressed('1') or kb.is_pressed('2'):
            break
    cv2.destroyAllWindows()


def aruco_tracking(follow_AR=True, slope_orient=True, take=TAKE):
    # constants for pid
    prev_ud_error = 0
    prev_yaw_error = 0
    prev_fb_error = 0
    prev_lr_error = 0

    udP = 0.25
    udI = 0
    udD = 2

    lrP = 55
    lrI = 0
    lrD = 100

    ywP = 0.3
    ywI = 0
    ywD = 1.5

    fbP = 0.25
    fbI = 0
    fbD = 0.6

    # MAIN LOOP
    while True:
        img = my_drone.get_frame_read().frame
        img = cv2.resize(img, (720, 480))
        corners, ids = ar_detector(img)
        my_ar = Marker(corners, ids)
        cv2.imshow("Aruco Tracking", img)

        take = take_picture(img, take)
        take_video(img)

        cv2.waitKey(1)

        try:
            values = motion_func(my_drone=my_drone)

            prev_ud_error, ud_speed = ar_marker_up_down_with_PID(my_ar, udP, udI, udD, prev_ud_error)
            prev_yaw_error, yaw_speed = ar_marker_yaw_with_PID(my_ar, ywP, ywI, ywD, prev_yaw_error)

            if follow_AR:
                prev_fb_error, fb_speed = follow_ar_with_PID(my_ar, fbP, fbI, fbD, prev_fb_error)
            else:
                fb_speed = 0

            if slope_orient:
                prev_lr_error, lr_speed = slope_orientation_with_PID(my_ar, lrP, lrI, lrD, prev_lr_error)
            else:
                lr_speed = 0

            my_drone.send_rc_control(lr_speed + values[0], fb_speed + values[1], yaw_speed + values[2],
                                     ud_speed + values[3])

        except:
            values = motion_func(my_drone=my_drone)
            my_drone.send_rc_control(values[0], values[1], values[2], values[3])

        if kb.is_pressed('0') or kb.is_pressed('2'):
            break
    cv2.destroyAllWindows()


def pose_tracking(draw=True, take=TAKE):
    # constants for pid
    prev_fb_error = 0
    prev_ud_error = 0
    prev_yaw_error = 0

    fbP = 0.001
    fbI = 0
    fbD = 0.00005

    udP = 0.025
    udI = 0
    udD = 0.02

    ywP = 0.1
    ywI = 0
    ywD = 0.03

    # MAIN LOOP
    while True:

        img = my_drone.get_frame_read().frame
        img = cv2.resize(img, (720, 480))

        man = Pose(img)

        if draw:
            mp_drawing.draw_landmarks(img, man.results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        cv2.imshow("Human Tracking", img)
        cv2.waitKey(1)

        take = take_picture(img, take)
        take_video(img)

        values = motion_func(my_drone)
        prev_fb_error, fb_speed = follow_man_with_PID(man, fbP, fbI, fbD, prev_fb_error)
        prev_ud_error, ud_speed = man_up_down_with_PID(man, udP, udI, udD, prev_ud_error)
        prev_yaw_error, yw_speed = man_yaw_with_PID(man, ywP, ywI, ywD, prev_yaw_error)

        my_drone.send_rc_control(values[0], fb_speed + values[1], yw_speed + values[2], ud_speed + values[3])

        if kb.is_pressed('0') or kb.is_pressed('1'):
            break

    cv2.destroyAllWindows()


while True:

    track_aruco = False
    track_pose = False

    if kb.is_pressed('2'):
        track_aruco = False
        track_pose = True

    elif kb.is_pressed('1'):
        track_aruco = True
        track_pose = False

    elif kb.is_pressed('0'):
        track_aruco = False
        track_pose = False

    if track_aruco:
        aruco_tracking()

    elif track_pose:
        pose_tracking()

    else:
        flight_mode()
