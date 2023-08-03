import cv2

# initialize the camera
cam = cv2.VideoCapture(2)   # 0 -> index of camera


while True:    # frame captured without any errors
    s, img = cam.read()

    print("Status: {}".format(s))

    cv2.imshow("ZED2 Left",img)
    code = cv2.waitKeyEx(1)

    if code == 113:
        exit()
