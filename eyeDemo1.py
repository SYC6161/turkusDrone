import cv2
import numpy as np


def nothing():
    pass

cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_COMPLEX

while True:
    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color
    low_red = np.array([175, 50, 20])
    high_red = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, low_red, high_red)
    kernel = np.ones((5, 5), np.uint8)
    mask2 = cv2.erode(mask1, kernel)
    red = cv2.bitwise_and(frame, frame, mask=mask2)

    contours, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        hull = cv2.convexHull(cnt)
        x = approx.ravel()[0]
        y = approx.ravel()[1]
        if area > 400:
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            if len(approx) == 4:
                cv2.putText(frame, "Rectangle", (x, y), font, 1, (0, 0, 0))

    edges = cv2.Canny(frame, 100, 200)

    _, threshold_binary = cv2.threshold(frame, 128, 255, cv2.THRESH_BINARY)
    _, threshold_binary_inv = cv2.threshold(frame, 128, 255, cv2.THRESH_BINARY_INV)
    _, threshold_trunc = cv2.threshold(frame, 128, 255, cv2.THRESH_TRUNC)
    _, threshold_to_zero = cv2.threshold(frame, 12, 255, cv2.THRESH_TOZERO)

    cv2.imshow("Frame", frame)
    cv2.imshow('edges', edges)
    cv2.imshow('red', red)
    cv2.imshow("mask", mask1)

    key = cv2.waitKey(1)
    if key == 27:
        cap.release()
        cv2.destroyAllWindows()

        break