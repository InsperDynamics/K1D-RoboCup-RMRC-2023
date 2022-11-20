
import cv2
resolution_horizontal = 800
resolution_vertical = 450

cap = cv2.VideoCapture(2)
cap2 = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FPS, 30)
cap2.set(cv2.CAP_PROP_FPS, 30)
  
while True:
    ret, img = cap.read()
    ret2, img2 = cap2.read()
    #resized = cv2.resize(img, (resolution_horizontal, resolution_vertical))
    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cv2.imshow('test', img)
    cv2.imshow('test2', img2)
    cv2.waitKey(30)