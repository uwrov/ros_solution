import cv2
import time

if __name__ == "__main__":
    cp = cv2.VideoCapture(0)
    #cp.set(cv2.CV_CAP_PROP_FRAMES, 5) 
    while (True):
        ret, frame = cp.read()
        cv2.imshow('frame', frame)
        if cv2.waitKey(27) == 1:
            break
