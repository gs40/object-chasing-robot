import cv2
import numpy as np
import time
from adafruit_motorkit import MotorKit


class Robot:
    def __init__(self):
        self.kit = MotorKit()  # defining the motor object

        self.distThreshold = 12.0  # robot will stop before 12 cm after detecting any object
        self.turnTime = 0.3  # time take to take a turn
        self.haltTime = 0.3  # for how much long to stay halt
        self.forSpeed = 0.3  # forward speed
        self.turningSpeed = 0.3
        self.cap = cv2.VideoCapture(0)  # defining the camera object
        time.sleep(1)

        self.kp = 1.0  # proportional gain
        self.kd = 1.0  # derivative gain
        self.ki = 1.0  # integral gain

        # the ball should be kept in the center of the frame
        self.ballx_axis = 0  # for moving the robot in right or left direction
        self.bally_axis = 0  # for moving the robot forward or backward
        self.framex_axis = 0  # for moving the robot in right or left direction
        self.framey_axis = 0  # for moving the robot forward or backward


        self.params = cv2.SimpleBlobDetector_params()

        # setting the parameters for blob detection

        self.params.filterByColor = False
        self.params.filterByArea = True
        self.params.minArea = 5000
        self.params.maxArea = 15000
        self.params.filterByInertia = False
        self.params.filterByConvexity = False
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.5
        self.params.maxCircularity = 1

        # creating the blob detector object
        self.det = cv2.SimpleBlobDetector_create(self.params)

        # defining lower bound hues for blue ball
        self.lower_blue = np.array([80, 60, 20])
        # dining upper bound hues for blue ball
        self.upper_blue = np.array([130, 255, 255])

    # for moving forward, backward,left,right or for halt
    # motor1 and motor2 are on left side and 3 and 4 are on right side
    def FBHT(self, leftSide, rightSide):
        self.kit.motor1.throttle = leftSide
        self.kit.motor2.throttle = leftSide
        self.kit.motor3.throttle = rightSide
        self.kit.motor4.throttle = rightSide
        time.sleep(self.turnTime)


    def maneuver(self):
        while True:
            # ret is and integer which is 1 if the frame is captured properly
            # and 0 if the frame is not captured properly
            # frame is the captured frame
            ret, frame = self.cap.read()

            # calculate center of the frame
            height, width, channels = np.shape(frame)
            center = (width/2, height/2)  # xmid and ymid respectively

            # converting the frame to hsv color space
            imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            blueMask = cv2.inRange(imgHSV, self.lower_blue, self.upper_blue)
            blur = cv2.blur(blueMask, (11, 11))
            #blue_ball = cv2.bitwise_and(frame, frame, mask = blueMask)

            # get the center and size of the blue ball
            keypoints = self.det.detect(blur)
            # measured center of the ball in x-axis
            try:
                self.ballx_axis = keypoints[0].pt[0]
                # measured center of the ball in y-axis
                self.bally_axis = keypoints[0].pt[1]
                for k in keypoints:
                    size = k.size
            except:
                pass

            # draw a red circle around the blue ball
            cv2.drawKeypoints(frame, keypoints, frame, (0, 0, 255),
                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            if (size >0):
                if (center[0]-self.ballx_axis) < 0:  # turn right
                    self.FBHT(self.turningSpeed, -self.turningSpeed)

                if (center[0]-self.ballx_axis) > 0:  # turn left
                    self.FBHT(-self.turningSpeed, self.turningSpeed)

                if (center[1]-self.bally_axis) < 0:  # go forward
                    self.FBHT(self.forSpeed, self.forSpeed)

                if (center[1]-self.bally_axis) > 0:  # go backward
                    self.FBHT(-self.forSpeed, -self.forSpeed)

            cv2.imshow('frame', frame)
            cv2.waitKey(1)


if __name__ == '__main__':
    # creating the object of the class
    obj = Robot()
    try:
        obj.maneuver()
    except KeyboardInterrupt:
        obj.FBH(0)
        obj.cap.release()
        cv2.destroyAllWindows()
