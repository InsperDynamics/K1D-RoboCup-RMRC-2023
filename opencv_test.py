import cv2
import numpy as np
resolution_horizontal = 800
resolution_vertical = 450
img1 = np.zeros((resolution_vertical, resolution_horizontal, 3), np.uint8)
img2 = np.zeros((resolution_vertical, resolution_horizontal, 3), np.uint8)

def list_ports():
    is_working = True
    dev_port = 0
    working_ports = []
    available_ports = []
    while is_working:
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            is_working = False
            print("Port %s is not working." %dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
        dev_port +=1
    return available_ports,working_ports

list_ports()

indexA = 0
indexB = 2

cap1 = cv2.VideoCapture(indexA)
cap1.set(cv2.CAP_PROP_FPS, 30)
print("Opened camera A: ", cap1.isOpened())
cap2 = cv2.VideoCapture(indexB)
cap2.set(cv2.CAP_PROP_FPS, 30)
print("Opened camera B: ", cap2.isOpened())

while True:
    ret, img1_local = cap1.read()
    ret2, img2_local = cap2.read()
    if ret:
        img1 = img1_local
    else:
        try:
            cap1.release()
            cap1 = cv2.VideoCapture(indexA)
            cap1.set(cv2.CAP_PROP_FPS, 30)
        except:
            print("Camera A disconnected")
    if ret2:
        img2 = img2_local
    else:
        try:
            cap2.release()
            cap2 = cv2.VideoCapture(indexB)
            cap2.set(cv2.CAP_PROP_FPS, 30)
        except:
            print("Camera B disconnected")
    resized = cv2.resize(img1, (resolution_horizontal, resolution_vertical))
    resized2 = cv2.resize(img2, (resolution_horizontal, resolution_vertical))
    concated = cv2.hconcat([resized, resized2])
    cv2.imshow('Camera A', resized)
    cv2.imshow('Camera B', resized2)
    cv2.imshow('Concat', concated)
    cv2.waitKey(1)