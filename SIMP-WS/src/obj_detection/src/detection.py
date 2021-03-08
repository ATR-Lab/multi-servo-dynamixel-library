import os
from os.path import isfile, join
import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
import threading
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

IMG_SUB_TOPIC = "/usb_cam/image_raw"
IMG_PUB_TOPIC = "/image/rgb/detection"
IMG_DIR = "../labels/"

STD_HEIGHT = 200.0
MIN_AREA = .75 #Factor
MATCH_DISTANCE = 50
MATCH_COUNT = 4
TEMPLATE_THRESHOLD = .4

labels = []
orb = None
bf = None
imgData = None
bridge = None

class colorMask:
    def __init__(self, ranges):
        self.ranges = ranges
    
    def filterImage(self, img):
        mask = None
        for range in self.ranges:
            if mask is None:
                mask = cv2.inRange(img, range[0], range[1])
            else:
                mask = mask + cv2.inRange(img, range[0], range[1])
        filtedImg = cv2.bitwise_and(img, img, mask=mask)
        filtedImg = cv2.cvtColor(filtedImg, cv2.COLOR_HSV2BGR)
        filtedImg = cv2.cvtColor(filtedImg, cv2.COLOR_BGR2GRAY)
        T, filtedImg = cv2.threshold(filtedImg, 1, 255, cv2.THRESH_BINARY)
        return filtedImg

colors = {
    "red": colorMask([((0, 150, 85), (10, 255, 255)), ((170, 150, 85), (180, 255, 255))]),
    "green": colorMask([((37, 100, 50), (72, 255, 255))]),
    "yellow": colorMask([((20, 150, 125), (35, 255, 255))]),
    "orange": colorMask([((13, 140, 150), (20, 255, 255))]),
    "white": colorMask([((0, 0, 175), (180, 100, 255))])
    }

class imageProcessor(threading.Thread):
    def __init__(self, publishTopic):
        self.imgPub = rospy.Publisher(publishTopic, Image, queue_size = 1)
        self.running = True
        threading.Thread.__init__(self)

    def stop(self):
        self.running = False

    def run(self):
        global labels
        global orb
        global bf
        global imgData
        global colors
        global bridge
        while self.running:
            if not imgData is None:
                img = bridge.imgmsg_to_cv2(imgData, "bgr8")
                height, width = img.shape[:2]
                hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                testColors = {}
                for key, color in colors.iteritems():
                    filtedImg = color.filterImage(hsvImg)
                    filtedImg = cv2.GaussianBlur(filtedImg, (7, 7), 0)
                    img2, contours, hierarchy = cv2.findContours(filtedImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    objects = []
                    for contour in contours:
                        if cv2.contourArea(contour) > MIN_AREA * STD_HEIGHT:
                            x, y, width, height = cv2.boundingRect(contour)
                            objects.append((img[y:y+height, x:x+width], x, y, width, height))

                    if len(objects) > 0:
                        testColors[key] = objects

                labelAreas = []

                for colorKey, areas in testColors.iteritems():
                    for area in areas:
                        templateMatches = []
                        match = None
                        for label in labels:
                            if colorKey in label[3]:                   
                                gLabel = cv2.cvtColor(label[2], cv2.COLOR_BGR2GRAY)
                                lHeight, lWidth = gLabel.shape[:2]
                                height, width = area[0].shape[:2]
                                resizedArea = cv2.resize(area[0], (min(lWidth, width), min(lWidth, width)))
                                gArea = cv2.cvtColor(resizedArea, cv2.COLOR_RGB2GRAY)
                                height, width = gArea.shape
                                featureMatch = False
                                kp2, des2 = orb.detectAndCompute(gArea, None)
                                kp1, des1 = orb.detectAndCompute(gLabel, None)
                                if not des1 is None and not des2 is None:
                                    matches = bf.match(label[1], des2)
                                    matches = sorted(matches, key = lambda x:x.distance)
                                    for i in range(0, len(matches)):
                                        if matches[i].distance > MATCH_DISTANCE:
                                            matches = matches[:i]
                                            break
                                    if len(matches) >= MATCH_COUNT:
                                        featureMatch = True
                                if featureMatch:
                                    for labelTemp in [gLabel, cv2.resize(gLabel, (int(0.8 * lWidth * STD_HEIGHT / lHeight), int(0.8 * STD_HEIGHT)))]:
                                        for i in range(0, 8):
                                            M = cv2.getRotationMatrix2D((width/2,height/2),i * 45,1)
                                            rot = cv2.warpAffine(gArea,M,(width,height))
                                            rot = cv2.resize(rot, (width, height))
                                            res = cv2.matchTemplate(rot, labelTemp, cv2.TM_CCOEFF_NORMED)
                                            value = np.max(res)
                                            if value > TEMPLATE_THRESHOLD:
                                                img = cv2.rectangle(img, (area[1], area[2]), (area[1] + area[3], area[2] + area[4]), (0, 255, 0), 3)
                                                img = cv2.putText(img, label[4], (area[1], area[2]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
                                                match = label
                                                break
                                        if not match is None:
                                            break
                            if not match is None:
                                break
                        if not match is None:
                            break
                print("send")
                self.imgPub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
                imgData = None
            else:
                time.sleep(.1)
                
def camera_callback(data):
    global imgData
    if imgData is None:
        imgData = data

def main(args):
    global labels
    global orb
    global bf
    global colors
    global bridge

    rospy.init_node('objectDetector', anonymous=True)

    bridge = CvBridge()

    imgFiles = [f for f in os.listdir(IMG_DIR) if isfile(join(IMG_DIR, f))]
    labels = []
    orb = cv2.ORB_create()
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    for imgFile in imgFiles:
        img = cv2.imread(IMG_DIR + imgFile)
        height, width = img.shape[:2]
        img = cv2.resize(img, (int(width * STD_HEIGHT / height), int(STD_HEIGHT)))
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = orb.detectAndCompute(grayImg, None)
        fitColors = []
        for key, color in colors.iteritems():
            filtedImg = color.filterImage(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
            height, width = filtedImg.shape
            ratio = np.count_nonzero(filtedImg) / (float(height) * width)
            if ratio > .2:
                fitColors.append(key)
        labels.append((kp, des, img, fitColors, imgFile))

    imgProcessor = imageProcessor(IMG_PUB_TOPIC)
    imgProcessor.start()

    imgSub = rospy.Subscriber(IMG_SUB_TOPIC, Image, camera_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    imgProcessor.stop()
    print("shutting down object detection")

if __name__ == '__main__':
    main(sys.argv)
