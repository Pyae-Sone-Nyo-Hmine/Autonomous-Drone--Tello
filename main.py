from djitellopy import tello
from utils.build import *

# Initialize my_drone
my_drone = tello.Tello()

# Connect to my_drone
my_drone.connect()

# Check battery
print("Battery : " + str(my_drone.get_battery()))

# Turn on video streaming
my_drone.streamon()

# Turn on functions
follow_AR = True
slope_orient = True

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


# counter for taking picture and video
take = 0

# video writer for saving video from the tello
video = cv2.VideoWriter("tello_video.avi", cv2.VideoWriter_fourcc(*'MJPG'), 80, (720, 480))

# MAIN LOOP
while True:
    img = my_drone.get_frame_read().frame
    img = cv2.resize(img, (720, 480))
    corners, ids = ar_detector(img)
    my_ar = Marker(corners, ids)
    cv2.imshow("Tello Stream", img)

    take = take_picture(img, take)
    take_video(video, img)

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
