import cv2
import numpy as np
import time

def turn_left():
    # TODO: เขียนโค้ดสั่งให้รถเลี้ยวซ้าย
    print("Turning left")
    pass

def turn_right():
    # TODO: เขียนโค้ดสั่งให้รถเลี้ยวขวา
    print("Turning right")
    pass

def go_straight():
    # TODO: เขียนโค้ดสั่งให้รถตรงไป
    print("Going straight")
    pass

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])


lower_green = np.array([40, 70, 70])
upper_green = np.array([80, 255, 255])

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        red_count = cv2.countNonZero(mask_red)
        green_count = cv2.countNonZero(mask_green)


        if red_count > green_count and red_count > 500:
            turn_right()
        elif green_count > red_count and green_count > 500:
            turn_left()
        else:
            go_straight()

        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", mask_red)
        cv2.imshow("Green Mask", mask_green)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.02)

finally:
    cap.release()
    cv2.destroyAllWindows()
