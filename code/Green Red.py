import cv2
import numpy as np
import time

# —— ฟังก์ชันควบคุมมอเตอร์ —— #
def turn_left():
    # TODO: เขียนโค้ดสั่งให้รถเลี้ยวซ้าย
    print("Turning left")
    # ex: motor_left.stop(); motor_right.forward()
    pass

def turn_right():
    # TODO: เขียนโค้ดสั่งให้รถเลี้ยวขวา
    print("Turning right")
    # ex: motor_right.stop(); motor_left.forward()
    pass

def go_straight():
    # TODO: เขียนโค้ดสั่งให้รถตรงไป
    print("Going straight")
    # ex: motor_left.forward(); motor_right.forward()
    pass

# —— ตั้งค่ากล้อง —— #
cap = cv2.VideoCapture(1)  # 1 = กล้องตัวที่หนึ่ง
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

# ปรับขนาดเฟรม (ถ้าต้องการ)
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# —— ช่วงสีในภาพ HSV —— #
# สีแดง (ต้องกรอบสองช่วง เพราะ hue ของแดงคร่อม 0°/180°)
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])

# สีเขียว
lower_green = np.array([40, 70, 70])
upper_green = np.array([80, 255, 255])

# —— Loop หลัก —— #
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # แปลง BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # สร้าง mask สำหรับสีแดง
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # สร้าง mask สำหรับสีเขียว
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # นับ pixel ของแต่ละสี
        red_count = cv2.countNonZero(mask_red)
        green_count = cv2.countNonZero(mask_green)

        # ดีบั๊กดูค่าจำนวน pixel
        # print(f"Red: {red_count}, Green: {green_count}")

        # ตัดสินใจเลี้ยว
        if red_count > green_count and red_count > 500:  # 500 = threshold ขั้นต่ำ
            turn_right()
        elif green_count > red_count and green_count > 500:
            turn_left()
        else:
            go_straight()

        # แสดงผลลัพธ์บนหน้าจอ (ถ้าต้องการ)
        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", mask_red)
        cv2.imshow("Green Mask", mask_green)

        # กด 'q' เพื่อออก
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # ลดโหลด CPU
        time.sleep(0.02)

finally:
    cap.release()
    cv2.destroyAllWindows()
