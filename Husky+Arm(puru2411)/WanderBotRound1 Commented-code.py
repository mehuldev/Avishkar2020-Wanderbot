# code by Team !ABHIMANYU  MNNIT allahabad { git ---->  PURU2411 }
# code of round 1 Wanderbot event AVISHKAR 20'
"""------------------------Libraries------------------------------------"""

from __future__ import division  # we need floating division
import pybullet as p
import pybullet_data
# for mathematical and scientific computaions
import time
import math
import numpy as np
from datetime import datetime
# for Image Processing
from PIL import Image
from cv2 import cv2
# For plotting graphs and simulations
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
# for path planning
from svg.path import Path, Line, Arc, CubicBezier, QuadraticBezier, parse_path


# Connecting to pybullet simulator in Shared Memory
clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


################################################################################################################
################################################################################################################
"""-------------------Loading and Setting URDFs------------------------"""

p.loadURDF("plane.urdf", [0, 0, 0.0])
p.setGravity(0, 0, -10)


# placing patchs to place the cube
cubeRPosition = [7, -2, 0.00025]
cubeR = p.loadURDF("cubes/urdf/placeingSquare.urdf",
                   basePosition=cubeRPosition, useFixedBase=1)
p.changeVisualShape(cubeR, -1, rgbaColor=[1, 0, 0, 1])

# importing cubes
cube1Position = [2, 5, 0.025]
cube1 = p.loadURDF("cubes/urdf/cube_small.urdf",
                   basePosition=cube1Position, globalScaling=0.8)
p.changeVisualShape(cube1, -1, rgbaColor=[1, 0, 0, 1])

# impoting the husky bot at
huskyCenter = [0.0, 0.0, 0.0]
huskyOrientation = p.getQuaternionFromEuler([0, 0, 0])
husky = p.loadURDF("husky/husky.urdf", huskyCenter, huskyOrientation)


# importing kuka arm at
kukaCenter = [0.0, 0.0, 0.24023]
kukaOrientation = p.getQuaternionFromEuler([0, 0, 0])
scale = 0.4
kukaId = p.loadURDF("kuka_experimental-indigo-devel/kuka_kr210_support/urdf/kr210l150.urdf",
                    kukaCenter, kukaOrientation, globalScaling=scale)


# setting kuka initialy 0
curr_joint_value = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
p.setJointMotorControlArray(kukaId, range(
    11), p.POSITION_CONTROL, targetPositions=curr_joint_value)


# puting kuka on husky
cid = p.createConstraint(husky, 1, kukaId, -1, p.JOINT_FIXED,
                         [0, 0, 0], [0.0, 0.0, 0.14023], [0., 0., 0], [0.0, 0.0, 0.0])


# activating real time simulation
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)


# printing the basic info of robot
# print("the joint info of husky : ")
# for i in range(p.getNumJoints(husky)):
#   print(p.getJointInfo(husky, i))

# print("the joint info of kuka : ")
# for i in range(p.getNumJoints(kukaId)):
#   print(p.getJointInfo(kukaId, i))


# print("the joint info of cube : ")
# for i in range(p.getNumJoints(cube)):
#   print(p.getJointInfo(cube, i))

# print('cube ground state', p.getLinkStates(cube, [0])[0][0])

# giving a cool off time of 2 sec
# time.sleep(2)


################################################################################################################
################################################################################################################


################################################################################################################
################################################################################################################
# image captureing and processing

"""-------------Image Processing and Object Detection---------------------"""


def boxDetection2411(rgbaImg, width, height):
	"""
	Takes RGBA image along with height and width and returns positions of cubes in reference frame
	"""
	rgba = bytes(rgbaImg)
	# Make a new image object from the bytes
	img = Image.frombytes('RGBA', (width, height), rgba)
	opencv_img = np.array(img)

	# Converting RGBA image to RGB(dropping alpha channel)
	rgbImage = cv2.cvtColor(opencv_img, cv2.COLOR_RGBA2RGB)
	# Converting RGB image to HSV(For Color detection)
	hsvFrame = cv2.cvtColor(rgbImage, cv2.COLOR_RGB2HSV)
	# Converting RGB image to BGR
	imageFrame = cv2.cvtColor(rgbImage, cv2.COLOR_RGB2BGR)

    # Set range for red color and define mask
	red_lower = np.array([0, 70, 50], np.uint8)
	red_upper = np.array([10, 255, 255], np.uint8)
	red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Set range for green color and define mask
	green_lower = np.array([40, 52, 72], np.uint8)
	green_upper = np.array([70, 255, 255], np.uint8)
	green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # Set range for blue color and define mask
    blue_lower = np.array([110, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    # Set range for orange color and define mask
    orange_lower = np.array([10, 70, 50], np.uint8)
    orange_upper = np.array([20, 255, 255], np.uint8)
    orange_mask = cv2.inRange(hsvFrame, orange_lower, orange_upper)

    # Set range for yellow color and define mask
    yellow_lower = np.array([30, 200, 200], np.uint8)
    yellow_upper = np.array([40, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

    # Set range for purple color and define mask
    purple_lower = np.array([150, 230, 200], np.uint8)
    purple_upper = np.array([220, 255, 255], np.uint8)
    purple_mask = cv2.inRange(hsvFrame, purple_lower, purple_upper)

	# initializing kernel for convolution over image frame
    kernal = np.ones((5, 5), "uint8")

    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)

    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)

    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)

    # For blue color
    orange_mask = cv2.dilate(orange_mask, kernal)
    res_orange = cv2.bitwise_and(imageFrame, imageFrame, mask=orange_mask)

    # For blue color
    yellow_mask = cv2.dilate(yellow_mask, kernal)
    res_yellow = cv2.bitwise_and(imageFrame, imageFrame, mask=yellow_mask)

    # For purple color
    purple_mask = cv2.dilate(purple_mask, kernal)
    res_purple = cv2.bitwise_and(imageFrame, imageFrame, mask=purple_mask)

    positionsCube = []
    positionsPatch = []
    a = 120
    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(
        red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area < 300):
            x, y, w, h = cv2.boundingRect(contour)
            positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Reality Stone", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 0, 200))
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area < 5000 and area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Reality Stone Holder", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 0, 200))

    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(
        green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area < 300):
            x, y, w, h = cv2.boundingRect(contour)
            positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Time Stone", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 130, 18))
        elif(area < 1000):
            x, y, w, h = cv2.boundingRect(contour)
            positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Time Stone Holder", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 130, 18))

    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(
        blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area < 300):
            x, y, w, h = cv2.boundingRect(contour)
            positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Space Stone", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (180, 0, 0))
        elif(area < 5000):
            x, y, w, h = cv2.boundingRect(contour)
            positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Space Stone Holder", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (180, 0, 0))

    # Creating contour to track orange color
    contours, hierarchy = cv2.findContours(
        orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area < 300):
            x, y, w, h = cv2.boundingRect(contour)
            positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Soul Stone", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 106, 225))
        elif(area < 5000):
            x, y, w, h = cv2.boundingRect(contour)
            positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Soul Stone Holder", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 106, 225))

    # Creating contour to track yellow color
    contours, hierarchy = cv2.findContours(
        yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area < 300):
            x, y, w, h = cv2.boundingRect(contour)
            positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Mind Stone", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 200, 130))
        elif(area < 2000):
            x, y, w, h = cv2.boundingRect(contour)
            positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Mind Stone Holder", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 200, 130))

    # Creating contour to track purple color
    contours, hierarchy = cv2.findContours(
        purple_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area < 300):
            x, y, w, h = cv2.boundingRect(contour)
            positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Power Stone", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (255, 0, 225))
        elif(area < 2000):
            x, y, w, h = cv2.boundingRect(contour)
            positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
            cv2.putText(imageFrame, "Power Stone Holder", (x, y),
                        cv2.FONT_HERSHEY_TRIPLEX, 2, (255, 0, 225))

    return imageFrame, positionsCube, positionsPatch


# setting camera parameters to take 1 image
def take_1photo():
    imageFrame = []
    width = 3000
    height = 3000

    view_matrix = p.computeViewMatrix([0, 0, 50], [0, 0, 0.0], [1, 0, 0])
    projection_matrix = p.computeProjectionMatrix(12, -12, 12, -12, 48.0, 51.1)
    width, height, rgbaImg, depthImg, segImg = p.getCameraImage(
        width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img, positionsCube, positionsPatch = boxDetection2411(
        rgbaImg, width, height)
    imageFrame.append(img)
    return imageFrame, positionsCube, positionsPatch


# setting camera parameters to take 4 images
# def take_4photo():
# 	imageFrame = []
# 	width = 3000
# 	height = 3000

# 	for j in range(2):
# 		for k in range(2):
# 			view_matrix = p.computeViewMatrix([j*12-6, k*12-6, 50], [j*12-6, k*12-6, 0.0], [1, 0, 0])
# 			projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 48.5, 51.1)
# 			width, height, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
# 			img, positionsCube, positionsPatch = boxDetection2411(rgbaImg,width,height)
# 			imageFrame.append(img)
# 	return imageFrame, positionsCube, positionsPatch


# capturing images
imageFrame, positionsCube, positionsPatch = take_1photo()
# imageFrame = take_4photo()

# saving captured and processed images
for i in range(len(imageFrame)):
    cv2.imwrite('top_view'+str(i)+'.jpg', imageFrame[i])

print("\nposition of cubes (R, G, B, O, Y, P) : ", positionsCube, "\n")

print("\nposition of cube holders (R, G, B, O, Y, P) : ", positionsPatch, '\n')

# pos = []
# for i in range(len(positions)):
# 	pos.append([])

################################################################################################################
################################################################################################################


################################################################################################################
################################################################################################################

"""-----------------------Inverse Kinematics for kuka arm and Bot Manipulation-------------------------"""

a1 = 0.75*scale
a2 = 0.35*scale
a3 = 1.25*scale
a4 = 0.054*scale
a5 = 1.5*scale
a6 = 0.303*scale


def get_hypotenuse(a, b):
    return np.sqrt(a*a + b*b)


def get_cosine_law_angle(a, b, c):
    """given all sides of a triangle a, b, c
    calculate angle gamma between sides a and b using cosine law"""
    gamma = np.arccos((a*a + b*b - c*c) / (2*a*b))
    return gamma


def griperCenter(px, py, pz, R06):
    # calculating griper center
    Xc = px - a6*R06[0, 2]
    Yc = py - a6*R06[1, 2]
    Zc = pz - a6*R06[2, 2]
    return Xc, Yc, Zc


def get_first3Angles(Xc, Yc, Zc):

    l = np.sqrt(a4**2 + a5**2)
    phi = np.arctan2(a4, a5)

    r1 = get_hypotenuse(Xc, Yc)
    r2 = get_hypotenuse(r1-a2, Zc-a1)

    phi1 = np.arctan2(Zc-a1, r1-a2)
    phi2 = get_cosine_law_angle(r2, a3, l)

    phi3 = get_cosine_law_angle(l, a3, r2)

    q1 = np.arctan2(Yc, Xc)
    q2 = np.pi/2 - phi1 - phi2
    q3 = np.pi/2 - phi3 - phi

    return q1, q2, q3


def get_last3Angles(R36):

    # R3_6 =  Matrix([[-np.sin(q4)*np.sin(q6) + np.cos(q4)*np.cos(q5)*np.cos(q6), -np.sin(q4)*np.cos(q6) - np.sin(q6)*np.cos(q4)*np.cos(q5), -np.sin(q5)*np.cos(q4)],
    #                 [ np.sin(q4)*np.cos(q5)*np.cos(q6) + np.sin(q6)*np.cos(q4), -np.sin(q4)*np.sin(q6)*np.cos(q5) + np.cos(q4)*np.cos(q6), -np.sin(q4)*np.sin(q5)],
    #                 [ np.sin(q5)*np.cos(q6)                                   , -np.sin(q5)*np.sin(q6)                                   ,  np.cos(q5)           ]])

    if(R36[2, 2] >= 1):
        R36[2, 2] = 1
    elif(R36[2, 2] <= -1):
        R36[2, 2] = -1

    q5 = np.arccos(R36[2, 2])

    q6 = np.arctan2(-R36[2, 1], R36[2, 0])

    q4 = np.arctan2(-R36[1, 2], -R36[0, 2])

    return q4, q5, q6


def get_kuka_angles(basePosition, baseOrientation, point, orientation):
    a = 0.015772399999437497
    b = 0.009488456000838417
    theta = baseOrientation[2]

    xB = basePosition[0] + a*np.cos(theta) - b*np.sin(theta)
    yB = basePosition[1] + a*np.sin(theta) + b*np.cos(theta)
    zB = basePosition[2] - 0.3587040000000001

    alphaB = baseOrientation[0]
    betaB = baseOrientation[1]
    gamaB = baseOrientation[2]

    xP = point[0]
    yP = point[1]
    zP = point[2]

    alphaP = orientation[0]
    betaP = orientation[1]
    gamaP = orientation[2]

    Hgb = np.array([[np.cos(betaB)*np.cos(gamaB), np.sin(alphaB)*np.sin(betaB)*np.cos(gamaB) - np.sin(gamaB)*np.cos(alphaB), np.sin(alphaB)*np.sin(gamaB) + np.sin(betaB)*np.cos(alphaB)*np.cos(gamaB), xB],
                    [np.sin(gamaB)*np.cos(betaB), np.sin(alphaB)*np.sin(betaB)*np.sin(gamaB) + np.cos(alphaB)*np.cos(
                        gamaB), -np.sin(alphaB)*np.cos(gamaB) + np.sin(betaB)*np.sin(gamaB)*np.cos(alphaB), yB],
                    [-np.sin(betaB), np.sin(alphaB)*np.cos(betaB),
                     np.cos(alphaB)*np.cos(betaB), zB],
                    [0, 0, 0, 1]])

    Hgp = np.array([[np.cos(betaP)*np.cos(gamaP), np.sin(alphaP)*np.sin(betaP)*np.cos(gamaP) - np.sin(gamaP)*np.cos(alphaP), np.sin(alphaP)*np.sin(gamaP) + np.sin(betaP)*np.cos(alphaP)*np.cos(gamaP), xP],
                    [np.sin(gamaP)*np.cos(betaP), np.sin(alphaP)*np.sin(betaP)*np.sin(gamaP) + np.cos(alphaP)*np.cos(
                        gamaP), -np.sin(alphaP)*np.cos(gamaP) + np.sin(betaP)*np.sin(gamaP)*np.cos(alphaP), yP],
                    [-np.sin(betaP), np.sin(alphaP)*np.cos(betaP),
                     np.cos(alphaP)*np.cos(betaP), zP],
                    [0, 0, 0, 1]])

    IHgb = np.linalg.inv(Hgb)
    Hbp = np.dot(IHgb, Hgp)

    R6a = Hbp[:3, :3]
    R6b = np.array([[0, 0, 1.0],
                    [0, -1.0, 0],
                    [1.0, 0, 0]])

    R06 = np.dot(R6a, R6b)
    [Px, Py, Pz] = Hbp[:3, 3]

    Xc, Yc, Zc = griperCenter(Px, Py, Pz, R06)

    q1, q2, q3 = get_first3Angles(Xc, Yc, Zc)
    R03 = [[np.sin(q2)*np.cos(q1)*np.cos(q3) + np.sin(q3)*np.cos(q1)*np.cos(q2), np.sin(q1), -np.sin(q2)*np.sin(q3)*np.cos(q1) + np.cos(q1)*np.cos(q2)*np.cos(q3)],
           [np.sin(q1)*np.sin(q2)*np.cos(q3) + np.sin(q1)*np.sin(q3)*np.cos(q2), -np.cos(
               q1), -np.sin(q1)*np.sin(q2)*np.sin(q3) + np.sin(q1)*np.cos(q2)*np.cos(q3)],
           [-np.sin(q2)*np.sin(q3) + np.cos(q2)*np.cos(q3), 0, -np.sin(q2)*np.cos(q3) - np.sin(q3)*np.cos(q2)]]

    IR03 = np.transpose(R03)

    R36 = np.dot(IR03, R06)

    q4, q5, q6 = get_last3Angles(R36)

    return q1, q2, q3, q4, q5, q6


################################################################################################################
################################################################################################################


################################################################################################################
################################################################################################################

"""------------------------Motion Control of Husky----------------------"""


# spiting out the rotation speed according to proportionality
def speed_for_rotation(rotation, currOrientation):
    kp = 7
    Vmax = 5
    v = kp*(currOrientation - rotation)

    if(v > Vmax):
        v = Vmax
    elif(v < -Vmax):
        v = -Vmax

    # print("currOrientation, rotation, v : ", currOrientation, rotation, v)
    return v


# spiting out the forward speed according to proportionality
def speed_for_forward(delX, delY):
    kp = 10
    Vmax = 5
    v = kp*(abs(delX)+abs(delY))/2

    if(v > Vmax):
        v = Vmax
    elif(v < -Vmax):
        v = -Vmax

    return v


def get_target_parameters(x, y, z):

    currPosition = p.getLinkStates(husky, [0])[0][0]
    currOrientation = p.getEulerFromQuaternion(
        p.getLinkStates(husky, [0])[0][1])

    deltaX = x - currPosition[0]
    deltaY = y - currPosition[1]

    # if abs(deltaY)<0.4:
    # 	deltaY = abs(deltaY)

    rotation = [0, 0, np.arctan2(deltaY, deltaX)]
    # print("deltaY, deltaX : ", deltaY, deltaX)
    return deltaX, deltaY, rotation, currOrientation, currPosition


def move_husky_to_point(x, y, z):
    # print('moving husky to ', x, y, z)
    wheels = [2, 3, 4, 5]
    # (2, b'front_left_wheel', 0, 7, 6, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_left_wheel_link')
    # (3, b'front_right_wheel', 0, 8, 7, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_right_wheel_link')
    # (4, b'rear_left_wheel', 0, 9, 8, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_left_wheel_link')
    # (5, b'rear_right_wheel', 0, 10, 9, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_right_wheel_link')

    wheelVelocities = [0, 0, 0, 0]
    wheelDeltasTurn = [1, -1, 1, -1]
    wheelDeltasFwd = [1, 1, 1, 1]
    deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(
        x, y, z)

    # rotate the bot toward the destination
    while abs(rotation[2]-currOrientation[2]) >= 0.005:
        # images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

        deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(
            x, y, z)

        wheelVelocities = [0, 0, 0, 0]

        vr = speed_for_rotation(rotation[2], currOrientation[2])

        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i]

        for i in range(len(wheels)):
            p.setJointMotorControl2(
                husky, wheels[i], p.VELOCITY_CONTROL, targetVelocity=wheelVelocities[i], force=1000)

    # move kukaId forward to destination
    while abs(deltaX) >= 0.005 or abs(deltaY) >= 0.005:
        # images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

        deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(
            x, y, z)

        wheelVelocities = [0, 0, 0, 0]

        vr = speed_for_rotation(rotation[2], currOrientation[2])
        vf = speed_for_forward(deltaX, deltaY)

        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i]

        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] + vf * wheelDeltasFwd[i]

        for i in range(len(wheels)):
            p.setJointMotorControl2(
                husky, wheels[i], p.VELOCITY_CONTROL, targetVelocity=wheelVelocities[i], force=1000)

    # stoping the motion
    wheelVelocities = [0, 0, 0, 0]
    for i in range(len(wheels)):
        p.setJointMotorControl2(
            husky, wheels[i], p.VELOCITY_CONTROL, targetVelocity=wheelVelocities[i], force=1000)


################################################################################################################
################################################################################################################


################################################################################################################
################################################################################################################

"""-------------------------Motion Control of end effector-------------------------"""


def get_point_parameters(curr_joint_value, final_joint_value, t):

    inst_joint_value = np.array(
        curr_joint_value[:6]) + t*(np.array(final_joint_value) - np.array(curr_joint_value[:6]))
    return inst_joint_value


def move_endeffector_to_point(finalPoint, finalOrientation):

    kukaBasePosition = p.getLinkStates(kukaId, [0])[0][0]
    kukaBaseOrientation = p.getEulerFromQuaternion(
        p.getLinkStates(husky, [0])[0][1])
    final_joint_value = get_kuka_angles(
        kukaBasePosition, kukaBaseOrientation, finalPoint, finalOrientation)

    t = 0
    while t <= 1:
        q1, q2, q3, q4, q5, q6 = get_point_parameters(
            curr_joint_value, final_joint_value, t)
        p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, targetPositions=[
                                    q1, q2, q3, q4, q5, q6, curr_joint_value[6], curr_joint_value[7], curr_joint_value[8], curr_joint_value[9], curr_joint_value[10]])
        t += .00003

    curr_joint_value[0] = final_joint_value[0]
    curr_joint_value[1] = final_joint_value[1]
    curr_joint_value[2] = final_joint_value[2]
    curr_joint_value[3] = final_joint_value[3]
    curr_joint_value[4] = final_joint_value[4]
    curr_joint_value[5] = final_joint_value[5]


def hold(flag):
    if flag:
        curr_joint_value[7] = 1
        curr_joint_value[8] = 1
    else:
        curr_joint_value[7] = 0
        curr_joint_value[8] = 0

    p.setJointMotorControlArray(kukaId, range(
        11), p.POSITION_CONTROL, targetPositions=curr_joint_value)
    time.sleep(1)


################################################################################################################
################################################################################################################


################################################################################################################
################################################################################################################

"""-------------------------Picking object from its position-------------------"""


def calculate_position_for_husky(x2, y2, z2):
    x1, y1, z1 = p.getLinkStates(husky, [0])[0][0]

    t = 0.7 / np.sqrt((x1-x2)**2 + (y1-y2)**2)
    x = x2 + t*(x1-x2)
    y = y2 + t*(y1-y2)

    return x, y, z2


def pick_cube_from(cubePosition):
    [x2, y2, z2] = cubePosition
    x, y, z = calculate_position_for_husky(x2, y2, z2)
    # print('move husky to ',x, y, z)
    move_husky_to_point(x, y, z)

    initialOrientation = p.getLinkStates(kukaId, [6])[0][1]
    initialOrientation = p.getEulerFromQuaternion(initialOrientation)

    # print("husky orientation ", initialOrientation)
    time.sleep(.5)
    # move_endeffector_to_point([x2, y2, 1], [0, np.pi/2, 0])
    move_endeffector_to_point([x2, y2, .2], [0, np.pi/2, 0])
    move_endeffector_to_point([x2, y2, .1], [0, np.pi/2, 0])
    time.sleep(1)
    hold(1)

    move_endeffector_to_point([x2, y2, .4], [0, np.pi/2, 0])
    # move_endeffector_to_point([x2, y2, 1], [0, np.pi/2, 0])
    move_endeffector_to_point([x2, y2, 1], initialOrientation)
    time.sleep(1)


def place_cube_to(cubePosition):
    [x2, y2, z2] = cubePosition
    x, y, z = calculate_position_for_husky(x2, y2, z2)
    # print('move husky to ',x, y, z)
    move_husky_to_point(x, y, z)

    initialOrientation = p.getLinkStates(kukaId, [6])[0][1]
    initialOrientation = p.getEulerFromQuaternion(initialOrientation)

    # print("husky orientation ", initialOrientation)
    time.sleep(.5)
    # move_endeffector_to_point([x2, y2, 1], [0, np.pi/2, 0])
    move_endeffector_to_point([x2, y2, .2], [0, np.pi/2, 0])
    move_endeffector_to_point([x2, y2, .11], [0, np.pi/2, 0])
    time.sleep(1)
    hold(0)

    move_endeffector_to_point([x2, y2, .4], [0, np.pi/2, 0])
    # move_endeffector_to_point([x2, y2, 1], [0, np.pi/2, 0])
    move_endeffector_to_point([x2, y2, 1], initialOrientation)
    time.sleep(1)


################################################################################################################
################################################################################################################


################################################################################################################
################################################################################################################

"""---------------Creating and Following the Trajectory of the Path---------"""

def get_trajectory(t, id):
    # define the trajectory
    if(id == 1):  # trjectory of eight
        px = .6
        py = 1/6*np.sin(2 * t)
        pz = 1 + 1/2*np.sin(t)

    return px, py, pz


def follow_trajectory(id):
    # getting initial point of tregectory
    px, py, pz = get_trajectory(0, id)

    kukaBasePosition = p.getLinkStates(kukaId, [0])[0][0]
    kukaBaseOrientation = p.getEulerFromQuaternion(
        p.getLinkStates(husky, [0])[0][1])

    prevPose = [0, 0, 0]
    prevPose1 = [0, 0, 0]
    hasPrevPose = 0
    trailDuration = 20
    t = 0  # parametric variable

    while t <= 2*np.pi+.1:
        # getting the value of speed inf position after each iteration according to specified trajectory
        px, py, pz = get_trajectory(t, id)

        # getting values of each joints according to points
        q1, q2, q3, q4, q5, q6 = get_kuka_angles(
            kukaBasePosition, kukaBaseOrientation, [px, py, pz], [0, 0, 0])

        p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, targetPositions=[
                                    q1, q2, q3, q4, q5, q6, curr_joint_value[6], curr_joint_value[7], curr_joint_value[8], curr_joint_value[9], curr_joint_value[10]])
        t += .007

        ls = p.getLinkState(kukaId, 5)
        if (hasPrevPose):
            p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 4, trailDuration)
        # prevPose = pos
        prevPose1 = ls[4]
        hasPrevPose = 1


def create_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    ax.set_autoscale_on(False)
    return fig, ax


def update_plot(X, Y, Z,  fig, ax, n):
    X = np.reshape(X, (1, n))
    Y = np.reshape(Y, (1, n))
    Z = np.reshape(Z, (1, n))
    ax.cla()
    ax.plot_wireframe(X, Y, Z)
    plt.draw()
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    ax.set_autoscale_on(False)
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(.001)


def get_path(id):

    if(id == 1):   # set of points for "8"
        svgpath = "m 185.34925,203.60654 q 0,30.14579 -23.58557,50.13885 -23.42937,19.99306 -59.04201,19.99306 -37.799387,0 -60.291583,-19.52447 -22.336,-19.52448 -22.336,-49.98266 0,-19.36828 11.246098,-34.98786 11.246098,-15.77578 31.707748,-24.99133 v -0.93717 Q 44.304437,133.31842 35.24508,121.44754 26.341919,109.57666 26.341919,91.770341 q 0,-26.240896 21.555021,-43.734826 21.555021,-17.49393 54.82473,-17.49393 34.83166,0 55.6057,16.712951 20.77405,16.712951 20.77405,42.485259 0,15.775775 -9.84034,31.082965 -9.84034,15.15099 -28.89622,23.74176 v 0.93718 q 21.86741,9.37175 33.4259,23.11698 11.55849,13.74523 11.55849,34.98786 z M 148.33084,90.052187 q 0,-16.712951 -12.96425,-26.553287 -12.80806,-9.996532 -32.80112,-9.996532 -19.68067,0 -32.33253,9.371749 -12.495664,9.371748 -12.495664,25.30372 0,11.246098 6.247832,19.524473 6.404028,8.12218 19.212084,14.52621 5.779245,2.81153 16.556755,7.34121 10.933703,4.52967 21.242633,7.49739 15.46338,-10.30892 21.39882,-21.39882 5.93544,-11.0899 5.93544,-25.616113 z m 4.84207,116.522073 q 0,-14.37002 -6.40403,-22.96079 -6.24783,-8.74696 -24.67893,-17.49393 -7.34121,-3.4363 -16.08817,-6.40402 -8.746966,-2.96772 -23.273175,-8.27838 -14.057623,7.65359 -22.648392,20.77404 -8.434574,13.12045 -8.434574,29.6772 0,21.08644 14.52621,34.83167 14.52621,13.74523 36.862211,13.74523 22.80459,0 36.39362,-11.71469 13.74523,-11.71468 13.74523,-32.17633 z"

    elif(id == 2):  # set of points for "6"
        svgpath = """m 192.89809,187.54732 q 0,36.11694 -23.86582,59.02813 -23.70671,22.75208 -58.2326,22.75208 -17.501599,0 -31.821091,-5.40958 Q 64.659087,258.50836 53.68081,247.8483 39.99774,234.64254 32.519783,212.84509 q -7.318851,-21.79745 -7.318851,-52.5048 0,-31.50288 6.682429,-55.84602 6.841535,-24.343135 21.638344,-43.276685 14.00128,-17.978917 36.11694,-28.002562 22.115655,-10.182749 51.550175,-10.182749 9.38722,0 15.75144,0.795527 6.36422,0.795527 12.88754,2.863898 v 30.389144 h -1.59106 q -4.45495,-2.386582 -13.52396,-4.454953 -8.90991,-2.227477 -18.29713,-2.227477 -34.20767,0 -54.573173,21.479238 -20.365499,21.320133 -23.706714,57.755279 13.364859,-8.11437 26.252402,-12.25112 13.046648,-4.29584 30.070935,-4.29584 15.11502,0 26.57061,2.86389 11.6147,2.7048 23.70671,11.13739 14.00129,9.70543 21.00193,24.50224 7.15974,14.79681 7.15974,35.95783 z m -32.29841,1.27285 q 0,-14.79681 -4.45495,-24.50224 -4.29585,-9.70544 -14.31949,-16.86518 -7.31885,-5.09138 -16.22876,-6.68243 -8.90991,-1.59106 -18.61534,-1.59106 -13.523964,0 -25.138663,3.18211 -11.614699,3.18211 -23.865819,9.86454 -0.318211,3.50032 -0.477317,6.84154 -0.159105,3.18211 -0.159105,8.11437 0,25.13867 5.091375,39.77637 5.25048,14.4786 14.319491,22.91119 7.318852,7.00064 15.751441,10.34185 8.591697,3.18211 18.615337,3.18211 23.0703,0 36.27605,-14.00128 13.20575,-14.16038 13.20575,-40.57189 z"""

    elif(id == 3):  # set of points for "9"
        svgpath = "m 194.00949,132.78458 q 0,32.35895 -7.46745,58.74395 -7.3015,26.38499 -22.23641,44.97064 -15.10084,18.91755 -37.83508,29.20603 -22.73424,10.28849 -53.433761,10.28849 -8.629055,0 -16.262449,-0.99566 -7.633394,-0.82971 -13.607355,-2.82103 v -31.69519 h 1.659433 q 4.812358,2.48916 13.607355,4.81236 8.794998,2.15727 19.581316,2.15727 36.673481,0 57.416401,-21.90453 20.90886,-22.07046 24.22773,-60.73526 -15.43273,9.29282 -29.04009,13.27546 -13.60735,3.98264 -29.70386,3.98264 -15.266786,0 -27.712538,-2.98698 -12.279808,-2.98698 -24.72556,-11.61603 -14.603015,-10.12255 -22.070466,-25.72122 -7.301508,-15.59868 -7.301508,-37.33726 0,-37.83508 24.891504,-61.56498 24.891503,-23.7299 60.735268,-23.7299 17.92188,0 33.18867,5.642074 15.26679,5.476131 26.71688,16.594336 14.10519,13.773298 21.73858,35.511877 7.63339,21.572637 7.63339,55.922913 z m -33.52055,-7.13557 q 0,-25.721216 -5.31019,-41.319892 -5.31019,-15.598675 -14.60302,-24.227729 -7.79933,-7.467451 -16.76027,-10.620375 -8.96095,-3.318867 -19.41538,-3.318867 -23.89584,0 -37.835082,14.934902 -13.773299,14.934902 -13.773299,41.983671 0,15.76462 4.480471,25.72122 4.48047,9.9566 15.100845,17.42405 7.467451,5.14424 16.428392,6.96962 8.960941,1.65943 19.913203,1.65943 12.94358,0 26.21905,-3.48481 13.27547,-3.48481 24.8915,-10.12254 0.16595,-3.48481 0.33189,-6.80368 0.33189,-3.48481 0.33189,-8.795 z"

    elif(id == 4):  # set of points for "&"
        svgpath = "m 109.99677,74.649937 q 0,-12.730557 -7.63833,-19.966873 -7.50433,-7.370322 -19.296846,-7.370322 -12.328539,0 -20.100879,8.308363 -7.77234,8.174357 -7.77234,20.100879 0,10.050439 5.226229,17.822779 5.360234,7.772337 22.915002,18.358807 13.132574,-4.69021 19.832864,-13.668602 6.8343,-9.112398 6.8343,-23.585031 z M 135.99391,192.03907 71.939108,129.59234 q -4.154182,2.01009 -8.308364,5.36023 -4.154181,3.21614 -8.308363,8.84439 -3.752164,5.22623 -6.16427,12.59655 -2.412105,7.37032 -2.412105,16.61673 0,19.56485 11.390498,31.62538 11.524504,11.92652 32.563424,11.92652 12.462542,0 24.657082,-6.03026 12.32854,-6.16427 20.6369,-18.49281 z m 40.33576,-79.59948 v 12.86456 q 0,12.86456 -3.35014,28.94527 -3.35015,16.0807 -11.3905,30.68734 l 50.65421,49.31416 H 179.2778 L 148.59046,204.2336 q -15.41067,19.02884 -31.49138,26.66717 -16.0807,7.50433 -33.099445,7.50433 -27.873219,0 -46.366027,-16.21471 -18.358803,-16.34872 -18.358803,-42.74787 0,-12.32854 3.484152,-21.30693 3.484153,-8.9784 8.174358,-15.54468 4.690205,-6.29828 11.658509,-11.79252 6.968305,-5.62824 14.070616,-9.78243 -14.740645,-9.64842 -21.306932,-19.43085 -6.432281,-9.782423 -6.432281,-24.657074 0,-8.978392 3.484152,-17.018744 3.618158,-8.174357 10.720469,-14.87465 6.700293,-6.432281 17.420762,-10.452457 10.854474,-4.020176 23.853043,-4.020176 23.183017,0 37.521637,11.792516 14.33863,11.65851 14.33863,29.615295 0,5.896257 -1.60807,13.400586 -1.60807,7.370322 -5.49424,13.26658 -4.28819,6.566284 -12.19453,12.596554 -7.90635,6.03026 -20.5029,10.31845 l 49.71617,48.51012 q 1.87609,-5.36024 2.81413,-11.79252 0.93804,-6.43228 1.07204,-13.40058 0.26802,-7.50433 0.13401,-16.75074 0,-9.2464 0,-15.67868 z"

    path = parse_path(svgpath)

    # svg.path point method returns a complex number p, p.real and p.imag can pull the x, and y
    # # on 0.0 to 1.0 along path, represent percent of distance along path
    n = 1000  # number of line segments to draw

    # pts = []
    # for i in range(0,n+1):
    #     f = i/n  # will go from 0.0 to 1.0
    #     complex_point = path.point(f)  # path.point(t) returns point at 0.0 <= f <= 1.0
    #     pts.append((complex_point.real, complex_point.imag))

    # list comprehension version or loop above
    pts = [((p.real-100)/200, (300-p.imag)/200)
           for p in (path.point(i/n) for i in range(0, n+1))]

    return pts


def follow_path(id):

    X = []
    Y = []
    Z = []

    kukaBasePosition = p.getLinkStates(kukaId, [0])[0][0]
    kukaBaseOrientation = p.getEulerFromQuaternion(
        p.getLinkStates(husky, [0])[0][1])

    pts = get_path(id)

    (py, pz) = pts[0]
    px = 0.7
    q1, q2, q3, q4, q5, q6 = get_kuka_angles(
        kukaBasePosition, kukaBaseOrientation, [px, py, pz], [0, 0, 0])
    p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, targetPositions=[
                                q1, q2, q3, q4, q5, q6, curr_joint_value[6], curr_joint_value[7], curr_joint_value[8], curr_joint_value[9], curr_joint_value[10]])

    time.sleep(1)

    prevPose1 = [px, py, pz]
    hasPrevPose = 0
    trailDuration = 20
    t = 0  # parametric variable
    fig, ax = create_plot()

    for i in range(len(pts)):

        (py, pz) = pts[i]
        px = 0.7
        # print(px, py, pz)
        X.append(px)
        Y.append(py)
        Z.append(pz)

        # getting values of each joints according to points
        q1, q2, q3, q4, q5, q6 = get_kuka_angles(
            kukaBasePosition, kukaBaseOrientation, [px, py, pz], [0, 0, 0])

        p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, targetPositions=[
                                    q1, q2, q3, q4, q5, q6, curr_joint_value[6], curr_joint_value[7], curr_joint_value[8], curr_joint_value[9], curr_joint_value[10]])
        t += 1

        ls = p.getLinkState(kukaId, 5)
        if (hasPrevPose):
            p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 4, trailDuration)
        # prevPose = pos
        prevPose1 = ls[4]
        hasPrevPose = 1
        time.sleep(.015)

    update_plot(X, Y, Z, fig, ax, 1001)
    plt.show()


def write():		# will write "ROBOTICS"

    arr = [[-6, 0], [-6, 2], [-5, 2], [-5, 1], [-6, 1], [-5, 0], [-3, 0], [-3, 2], [-4, 2], [-4, 0], [-1, 0], [-1, 2], [-2, 2], [-2, 1], [-1, 1], [-2, 1], [-2, 0], [1, 0], [1, 2],
           [0, 2], [0, 0], [3, 0], [3, 2], [2, 2], [4, 2], [3, 2], [3, 0], [5, 0], [5, 2], [5, 0], [6, 0], [6, 2], [7, 2], [6, 2], [6, 0], [8, 0], [9, 0], [9, 1], [8, 1], [8, 2], [9.5, 2]]

    X = []
    Y = []
    Z = []

    kukaBasePosition = p.getLinkStates(kukaId, [0])[0][0]
    kukaBaseOrientation = p.getEulerFromQuaternion(
        p.getLinkStates(husky, [0])[0][1])

    py, pz = arr[0]
    py = py/10
    pz = 1 + pz/10
    px = 0.7
    q1, q2, q3, q4, q5, q6 = get_kuka_angles(
        kukaBasePosition, kukaBaseOrientation, [px, py, pz], [0, 0, 0])
    p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, targetPositions=[
                                q1, q2, q3, q4, q5, q6, curr_joint_value[6], curr_joint_value[7], curr_joint_value[8], curr_joint_value[9], curr_joint_value[10]])

    time.sleep(1)

    prevPose1 = [px, py, pz]
    hasPrevPose = 0
    trailDuration = 30
    t = 0  # parametric variable
    fig, ax = create_plot()

    for i in range(len(arr)):

        py, pz = arr[i]
        py = py/10
        pz = 1 + pz/10
        px = 0.7
        # print(px, py, pz)
        X.append(px)
        Y.append(py)
        Z.append(pz)

        # getting values of each joints according to points
        q1, q2, q3, q4, q5, q6 = get_kuka_angles(
            kukaBasePosition, kukaBaseOrientation, [px, py, pz], [0, 0, 0])

        p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, targetPositions=[
                                    q1, q2, q3, q4, q5, q6, curr_joint_value[6], curr_joint_value[7], curr_joint_value[8], curr_joint_value[9], curr_joint_value[10]])
        t += 1

        ls = p.getLinkState(kukaId, 5)
        if (hasPrevPose):
            p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 4, trailDuration)
        # prevPose = pos
        prevPose1 = ls[4]
        hasPrevPose = 1
        time.sleep(.2)

    update_plot(X, Y, Z, fig, ax, 41)
    plt.show()


################################################################################################################
################################################################################################################


start = time.time()

################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

# what you want from husky to do for you!
# write you code bellow :-)
print("writing 'ROBOTICS'\n")
write()
time.sleep(20)


print("Drawing '8'\n")
follow_path(1)
time.sleep(20)

print("Drawing '6'\n")
follow_path(2)
time.sleep(20)

print("Drawing '9'\n")
follow_path(3)
time.sleep(20)

print("Drawing '&'\n")
follow_path(4)
time.sleep(20)

for i in range(len(positionsCube)):
    print("going to pick the cube from : ",
          positionsCube[i][0], ", ", positionsCube[i][1])
    pick_cube_from([positionsCube[i][0], positionsCube[i][1], 0])
    print("going to place the cube from : ",
          positionsPatch[i][0], ", ", positionsPatch[i][1])
    place_cube_to([positionsPatch[i][0], positionsPatch[i][1], 0])

print("ging back to center\n")
move_husky_to_point(0, 0, 0)


################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

end = time.time()
print("total time taken : ", end-start)


time.sleep(10)
p.disconnect()
