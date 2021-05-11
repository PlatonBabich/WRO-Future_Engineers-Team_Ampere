from bot import AdamIMU
import cv2
import numpy as np


HUE_MIN = 45
HUE_MAX = 100
SAT_MIN = 104
SAT_MAX = 255
VAL_MIN = 4
VAL_MAX = 250

robot = AdamIMU()

import serial
import time
ser = serial.Serial(
        port='/dev/serial0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.01,
        #dsrdtr=False,
        #inter_byte_timeout=0.01 # Alternative
)
if (ser.isOpen() == False):
        ser.open()

def mapn(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def constrain(x, out_min, out_max):
    if x < out_min:
        return out_min
    elif out_max < x:
        return out_max
    else:
        return x

def CRC8( mas):
    st_byt = 0
    crc = 0
    while st_byt < len(mas):
        dat = mas[st_byt]
        for i in range(8):
            fb = crc ^ dat
            fb &= 1
            crc >>= 1
            dat >>= 1
            if fb == 1:
                crc ^= 0x8c
        st_byt += 1
    return crc
def send(mass):
    dat = [mass[0], mass[1]>> 8,mass[1] & 0xff]
    checkdat = [mass[0], mass[1]>> 8,mass[1] & 0xff, CRC8(dat), ord('$')]
    ser.write(bytearray(checkdat))


robot.fprint("START FRAME ADAM",robot)
count=0
n=0

t=0

def hsv_Output():
    global HUE_MIN ,HUE_MAX ,SAT_MIN ,SAT_MAX ,VAL_MIN ,VAL_MAX

    if len(robot.mesegRobot)>1:
        if robot.mesegRobot[0] =='S':
            if robot.mesegRobot[1]=="A":
                HUE_MIN=int(robot.mesegRobot[2:-1])
                robot.fprint(HUE_MIN)
            elif robot.mesegRobot[1] == "B":
                SAT_MIN = int(robot.mesegRobot[2:-1])
                robot.fprint(SAT_MIN)
            elif robot.mesegRobot[1] == "C":
                VAL_MIN= int(robot.mesegRobot[2:-1])
                robot.fprint(VAL_MIN)
            elif robot.mesegRobot[1] == "D":
                HUE_MAX = int(robot.mesegRobot[2:-1])
                robot.fprint(HUE_MAX)
            elif robot.mesegRobot[1] == "E":
                SAT_MAX = int(robot.mesegRobot[2:-1])
                robot.fprint(SAT_MAX)
            elif robot.mesegRobot[1] == "F":
                VAL_MAX = int(robot.mesegRobot[2:-1])
                robot.fprint(VAL_MAX)
def turnOrange(frame,hsv_or):
    x1 = 40
    x2 = 599
    y1 = 260
    y2 = 400

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    frame_crop = frame[y1:y2, x1:x2]

    imageHSV = cv2.cvtColor(frame_crop, cv2.COLOR_BGR2HSV)

    hsv_Output()
    mask = cv2.inRange(imageHSV, np.array([hsv_or[0], hsv_or[1], hsv_or[2]]), np.array([hsv_or[3], hsv_or[4], hsv_or[5]]))
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # перебираем все найденные контуры
    flag_line = False
    for contour in contours:
        # Создаем прямоугольник вокруг контура
        x, y, w, h = cv2.boundingRect(contour)
        # вычисляем площадь найденного контура
        area = cv2.contourArea(contour)
        if area > 40:
            # отрисовываем найденный контур
            cv2.drawContours(frame_crop, contour, -1, (255, 0, 0), 2)
            return True
    return False

hsv_or=[40, 31, 0, 113, 256, 256] #0, 50, 54, 150, 122, 241            45, 50, 0, 250, 0, 256
def turnBlue(frame,hsv_blue):
    xn1 = 40
    xn2 = 599
    yn1 = 260
    yn2 = 400

    cv2.rectangle(frame, (xn1, yn1), (xn2, yn2), (255, 50, 0), 2)

    frame_crop = frame[yn1:yn2, xn1:xn2]

    imageHSV = cv2.cvtColor(frame_crop, cv2.COLOR_BGR2HSV)

    hsv_Output()
    mask = cv2.inRange(imageHSV, np.array([hsv_blue[0], hsv_blue[1], hsv_blue[2]]), np.array([hsv_blue[3], hsv_blue[4], hsv_blue[5]]))
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # перебираем все найденные контуры
    flag_line = False
    for contour in contours:
        # Создаем прямоугольник вокруг контура
        x, y, w, h = cv2.boundingRect(contour)
        # вычисляем площадь найденного контура
        area = cv2.contourArea(contour)
        if area > 40:
            # отрисовываем найденный контур
            cv2.drawContours(frame_crop, contour, -1, (0, 0, 255), 2)
            return True
    return False
hsv_blue=[91, 18, 4, 186, 236, 223] #91, 154, 104, 255, 4, 250    86, 4, 40, 127, 245, 245


power = 110
while True:

    frame = robot.get_frame()
    frame = cv2.resize(frame, (640, 480))
    x1 = 590
    x2 = 639
    y1 = 480
    y2 = 235
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    frame_crop = frame[y2:y1, x1:x2]

    imageHSV = cv2.cvtColor(frame_crop, cv2.COLOR_BGR2HSV)

    hsv_Output()
    mask = cv2.inRange(imageHSV, np.array([HUE_MIN, SAT_MIN, VAL_MIN]), np.array([HUE_MAX, SAT_MAX, VAL_MAX]))
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # перебираем все найденные контуры
    flag_line = False

    for contour in contours:
        # Создаем прямоугольник вокруг контура
        x, y, w, h = cv2.boundingRect(contour)
        # вычисляем площадь найденного контура
        area = cv2.contourArea(contour)
        if area > 40:
            # отрисовываем найденный контур
            cv2.drawContours(frame_crop, contour, -1, (0, 0, 255), 2)
            e = 45 - h
            p = e * 1
            fact=int(mapn(p,8,-100,0,255))
            fact=constrain(fact, 0, 255)
            powred=0
            power=int(mapn(powred, 0, 100, 512, 1000))

            #robot.fprint(p)
            send([fact, power])
    flag1 = turnOrange(frame, hsv_or)
    flag2 = turnBlue(frame, hsv_blue)
    # robot.printf(flag1, flag2)
    if flag1 or flag2:
        powred = 0


        #turnBlue(frame, hsv_blue)
        # if flag_line:
        #     if mask[1] == hsv_or[0] and mask[2] == hsv_or[1]:
        #         powred = 0
    # turnOrange(frame, hsv_or)
    # turnBlue(frame, hsv_blue)
    #cv2.bitwise_or(frame, frame, mask=mask)

    # robot.set_frame(cv2.bitwise_or(frame, frame, mask=mask),80)
    # robot.set_frame(cv2.bitwise_or(frame, frame, mask=mask), 80)
    robot.set_frame(frame, 80)
    #robot.fprint(robot.key,robot.mesegRobot)
    robot.fprint(HUE_MIN, " ", SAT_MIN," ", VAL_MIN, " ", HUE_MAX, " ", SAT_MAX, " ", VAL_MAX) #зеленый: 63 173 0 180 256 256__________54 163 0 113 256 256
    # красный: 0 18 22 4 195 218_____________0 95 22 9 256 227
    n -= 1
    n = n%1000

