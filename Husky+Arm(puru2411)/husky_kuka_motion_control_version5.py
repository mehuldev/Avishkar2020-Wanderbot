# code by Team !ABHIMANYU  MNNIT allahabad { git ---->  PURU2411 }
# this code uses the external inverse kinematics function
# image processing iplemented

import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import numpy as np
from cv2 import cv2
from PIL import Image
import matplotlib.pyplot as plt



clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


################################################################################################################
################################################################################################################
# loading and setting urdfs

p.loadURDF("plane.urdf", [0, 0, 0.0])
p.setGravity(0, 0, -10)



# making boundary 
cubea = p.loadURDF("cubes/urdf/cube_small_extended_in_Y_direction.urdf", basePosition= [11.25, 0, 0.1], useFixedBase=1 )
p.changeVisualShape(cubea , -1, rgbaColor=[0, 0, 0, 1])
cubeb = p.loadURDF("cubes/urdf/cube_small_extended_in_Y_direction.urdf", basePosition= [-11.25, 0, 0.1] , useFixedBase=1)
p.changeVisualShape(cubeb , -1, rgbaColor=[0, 0, 0, 1])
cubec = p.loadURDF("cubes/urdf/cube_small_extended_in_X_direction.urdf", basePosition= [0, 11.25, 0.1] , useFixedBase=1)
p.changeVisualShape(cubec , -1, rgbaColor=[0, 0, 0, 1])
cubed = p.loadURDF("cubes/urdf/cube_small_extended_in_X_direction.urdf", basePosition= [0, -11.25, 0.1], useFixedBase=1 )
p.changeVisualShape(cubed , -1, rgbaColor=[0, 0, 0, 1])



# making the maze	

y1 = 3
y2 = -3
cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [4, y1, 0.2], useFixedBase=1 )
p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [4, y2, 0.2], useFixedBase=1 )
p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
for _ in range(6):

	cube = p.loadURDF("cubes/urdf/y_segment.urdf", basePosition= [4, y1+0.5, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	y1+=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [4, y1, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	
	cube = p.loadURDF("cubes/urdf/y_segment.urdf", basePosition= [4, y2-0.5, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	y2-=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [4, y2, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	
y1=0
y2=0
cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [8,y1 ,0.2], useFixedBase=1 )
p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
for _ in range(5):

	cube = p.loadURDF("cubes/urdf/y_segment.urdf", basePosition= [8, y1+0.5, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	y1+=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [8, y1, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	
	cube = p.loadURDF("cubes/urdf/y_segment.urdf", basePosition= [8, y2-0.5, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	y2-=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [8, y2, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])

y1 = 3
y2 = -3
cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [-4, y1+0.5, 0.2], useFixedBase=1 )
p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [-4, y2, 0.2], useFixedBase=1 )
p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
for _ in range(6):

	cube = p.loadURDF("cubes/urdf/y_segment.urdf", basePosition= [-4, y1+0.5, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	y1+=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [-4, y1, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	
	cube = p.loadURDF("cubes/urdf/y_segment.urdf", basePosition= [-4, y2-0.5, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	y2-=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [-4, y2, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	
y1=0
y2=0
cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [-8,y1 ,0.2], useFixedBase=1 )
p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
for _ in range(5):

	cube = p.loadURDF("cubes/urdf/y_segment.urdf", basePosition= [-8, y1+0.5, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	y1+=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [-8, y1, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	
	cube = p.loadURDF("cubes/urdf/y_segment.urdf", basePosition= [-8, y2-0.5, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	y2-=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [-8, y2, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])

x1=-2
x2=-2
cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [x1, 6, 0.2], useFixedBase=1 )
p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [x2, -6, 0.2], useFixedBase=1 )
p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
for _ in range(4):
	cube = p.loadURDF("cubes/urdf/x_segment.urdf", basePosition= [x1+0.5, 6, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	x1+=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [x1, 6, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	
	cube = p.loadURDF("cubes/urdf/x_segment.urdf", basePosition= [x2+0.5, -6, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])
	x2+=1
	cube = p.loadURDF("cubes/urdf/xy_segment.urdf", basePosition= [x2, -6, 0.2], useFixedBase=1 )
	p.changeVisualShape(cube , -1, rgbaColor=[0, 0, 0, 1])


# placing patchs to place the cube
cubeRPosition = [1.5, 0, 0.00025]
cubeR = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= cubeRPosition, useFixedBase=1)
p.changeVisualShape(cubeR , -1, rgbaColor=[1, 0, 0, 1])

cubeOPosition = [-6, -2, 0.00025]
cubeO = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= cubeOPosition, useFixedBase=1)
p.changeVisualShape(cubeO , -1, rgbaColor=[1, 130/255, 0, 1])

cubeGPosition = [-8, 8, 0.00025]
cubeG = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= cubeGPosition, useFixedBase=1)
p.changeVisualShape(cubeG , -1, rgbaColor=[0, 1, 0, 1])

cubeYPosition = [4, -2, 0.00025]
cubeY = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= cubeYPosition, useFixedBase=1)
p.changeVisualShape(cubeY , -1, rgbaColor=[1, 1, 0, 1])

cubeBPosition = [10, 0, 0.00025]
cubeB = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= cubeBPosition, useFixedBase=1)
p.changeVisualShape(cubeB , -1, rgbaColor=[0, 0, 1, 1])

cubePPosition = [5, -7, 0.00025]
cubeP = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= cubePPosition, useFixedBase=1)
p.changeVisualShape(cubeP , -1, rgbaColor=[1, 0, 1, 1])

# importing cubes
cube1Position = [1, 1, 0.025]
cube1 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube1Position, globalScaling=0.8)
p.changeVisualShape(cube1 , -1, rgbaColor=[1, 0, 0, 1])

cube2Position = [-9, -2, 0.025]
cube2 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube2Position, globalScaling=0.8)
p.changeVisualShape(cube2 , -1, rgbaColor=[1, 130.0/225.0, 0, 1])

cube3Position = [-2, 5, 0.025]
cube3 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube3Position, globalScaling=0.8)
p.changeVisualShape(cube3 , -1, rgbaColor=[0, 1, 0, 1])

cube4Position = [8, -7, 0.025]
cube4 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube4Position, globalScaling=0.8)
p.changeVisualShape(cube4 , -1, rgbaColor=[1, 1, 0, 1])

cube5Position = [-3, 7, 0.025]
cube5 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube5Position, globalScaling=0.8)
p.changeVisualShape(cube5 , -1, rgbaColor=[0, 0, 1, 1])

cube6Position = [3, -7, 0.025]
cube6 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube6Position, globalScaling=0.8)
p.changeVisualShape(cube6 , -1, rgbaColor=[1, 0, 1, 1])


# impoting the husky bot at 
huskyCenter = [0.0, 0.0, 0.0]
huskyOrientation = p.getQuaternionFromEuler([0,0,0])
husky = p.loadURDF("husky/husky.urdf", huskyCenter, huskyOrientation)


#importing kuka arm at
kukaCenter = [0.0, 0.0, 0.24023]
kukaOrientation = p.getQuaternionFromEuler([0,0,0])
scale = 0.4
kukaId = p.loadURDF("kuka_experimental-indigo-devel/kuka_kr210_support/urdf/kr210l150.urdf", kukaCenter, kukaOrientation, globalScaling= scale)


# setting kuka initialy 0 
curr_joint_value = [0,0,0,0,0,0,0,0,0,0,0]
p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions=curr_joint_value)


# puting kuka on husky
cid = p.createConstraint(husky, 1, kukaId, -1, p.JOINT_FIXED, [0, 0, 0], [0.0, 0.0, 0.14023], [0., 0., 0], [0.0, 0.0, 0.0])


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

# image processing
def boxDetection2411(rgbaImg,width,height):
	rgba = bytes(rgbaImg)

	# Make a new image object from the bytes
	img = Image.frombytes('RGBA', (width, height), rgba)
	# img.show()
	# img.save('test2.png')
	opencv_img = np.array(img)
	# print(opencv_img)

	rgbImage = cv2.cvtColor(opencv_img, cv2.COLOR_RGBA2RGB)
	hsvFrame = cv2.cvtColor(rgbImage, cv2.COLOR_RGB2HSV)
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
	purple_lower = np.array([150,230, 200], np.uint8) 
	purple_upper = np.array([220, 255, 255], np.uint8) 
	purple_mask = cv2.inRange(hsvFrame, purple_lower, purple_upper)

	kernal = np.ones((5, 5), "uint8")

	# For red color
	red_mask = cv2.dilate(red_mask, kernal)
	res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)

	# For green color 
	green_mask = cv2.dilate(green_mask, kernal) 
	res_green = cv2.bitwise_and(imageFrame, imageFrame, mask = green_mask) 
	      
	# For blue color 
	blue_mask = cv2.dilate(blue_mask, kernal) 
	res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask)

	# For blue color 
	orange_mask = cv2.dilate(orange_mask, kernal) 
	res_orange = cv2.bitwise_and(imageFrame, imageFrame, mask = orange_mask)

	# For blue color 
	yellow_mask = cv2.dilate(yellow_mask, kernal) 
	res_yellow = cv2.bitwise_and(imageFrame, imageFrame, mask = yellow_mask)

	# For purple color 
	purple_mask = cv2.dilate(purple_mask, kernal) 
	res_purple = cv2.bitwise_and(imageFrame, imageFrame, mask = purple_mask)

	positionsCube = []
	positionsPatch = []
	a = 120
	# Creating contour to track red color
	contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area < 300):
			x, y, w, h = cv2.boundingRect(contour)
			positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Reality Stone", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 0, 200))
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area < 5000 and area >300):
			x, y, w, h = cv2.boundingRect(contour)
			positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Reality Stone Holder", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 0, 200))

	# Creating contour to track green color 
	contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area < 300):
			x, y, w, h = cv2.boundingRect(contour)
			positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Time Stone", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 130, 18))
		elif(area < 1000):
			x, y, w, h = cv2.boundingRect(contour)
			positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Time Stone Holder", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 130, 18))


	# Creating contour to track blue color 
	contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area < 300):
			x, y, w, h = cv2.boundingRect(contour)
			positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Space Stone", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (180, 0, 0))
		elif(area < 5000):
			x, y, w, h = cv2.boundingRect(contour)
			positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Space Stone Holder", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (180, 0, 0))


	# Creating contour to track orange color 
	contours, hierarchy = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area < 300):
			x, y, w, h = cv2.boundingRect(contour)
			positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Soul Stone", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 106, 225))
		elif(area < 5000):
			x, y, w, h = cv2.boundingRect(contour)
			positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Soul Stone Holder", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 106, 225))


	# Creating contour to track yellow color 
	contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area < 300):
			x, y, w, h = cv2.boundingRect(contour)
			positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Mind Stone", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 200, 130))
		elif(area < 2000):
			x, y, w, h = cv2.boundingRect(contour)
			positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Mind Stone Holder", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 200, 130))
	
	# Creating contour to track purple color 
	contours, hierarchy = cv2.findContours(purple_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area < 300):
			x, y, w, h = cv2.boundingRect(contour)
			positionsCube.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Power Stone", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (255, 0, 225))
		elif(area < 2000):
			x, y, w, h = cv2.boundingRect(contour)
			positionsPatch.append([(y+h/2-1500)/a, (x+w/2-1500)/a])
			cv2.putText(imageFrame, "Power Stone Holder", (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (255, 0, 225))

	return imageFrame, positionsCube, positionsPatch


# setting camera parameters to take 1 image
def take_1photo():
	imageFrame = [] 
	width = 3000
	height = 3000

	view_matrix = p.computeViewMatrix([0, 0, 50], [0, 0, 0.0], [1, 0, 0])
	projection_matrix = p.computeProjectionMatrix(12, -12, 12, -12, 48.0, 51.1)
	width, height, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	img, positionsCube, positionsPatch = boxDetection2411(rgbaImg,width,height)
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

# inverse kinematics for kuka arm

a1 = 0.75*scale
a2 = 0.35*scale
a3 = 1.25*scale
a4 = 0.054*scale
a5 = 1.5*scale
a6 = 0.303*scale


def get_hypotenuse(a, b):
  return np.sqrt(a*a + b*b)


def get_cosine_law_angle(a, b, c):
    # given all sides of a triangle a, b, c
    # calculate angle gamma between sides a and b using cosine law
    gamma = np.arccos((a*a + b*b - c*c) / (2*a*b))
    return gamma


def griperCenter(px, py, pz, R06): 
    # calculating griper center
    Xc = px - a6*R06[0,2]
    Yc = py - a6*R06[1,2]
    Zc = pz - a6*R06[2,2]
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
	q3 = np.pi/2 -phi3 - phi


	return q1, q2, q3


def get_last3Angles(R36):

    # R3_6 =  Matrix([[-np.sin(q4)*np.sin(q6) + np.cos(q4)*np.cos(q5)*np.cos(q6), -np.sin(q4)*np.cos(q6) - np.sin(q6)*np.cos(q4)*np.cos(q5), -np.sin(q5)*np.cos(q4)], 
    #                 [ np.sin(q4)*np.cos(q5)*np.cos(q6) + np.sin(q6)*np.cos(q4), -np.sin(q4)*np.sin(q6)*np.cos(q5) + np.cos(q4)*np.cos(q6), -np.sin(q4)*np.sin(q5)], 
    #                 [ np.sin(q5)*np.cos(q6)                                   , -np.sin(q5)*np.sin(q6)                                   ,  np.cos(q5)           ]])

    if(R36[2, 2]>=1):
    	R36[2, 2] = 1
    elif(R36[2, 2]<=-1):
    	R36[2, 2] = -1

    q5 = np.arccos(R36[2, 2])

    q6 = np.arctan2(-R36[2, 1], R36[2, 0])

    q4 = np.arctan2(-R36[1,2], -R36[0,2])

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
					[np.sin(gamaB)*np.cos(betaB), np.sin(alphaB)*np.sin(betaB)*np.sin(gamaB) + np.cos(alphaB)*np.cos(gamaB), -np.sin(alphaB)*np.cos(gamaB) + np.sin(betaB)*np.sin(gamaB)*np.cos(alphaB), yB], 
					[-np.sin(betaB), np.sin(alphaB)*np.cos(betaB), np.cos(alphaB)*np.cos(betaB), zB],
					[0, 0, 0, 1]])

	Hgp = np.array([[np.cos(betaP)*np.cos(gamaP), np.sin(alphaP)*np.sin(betaP)*np.cos(gamaP) - np.sin(gamaP)*np.cos(alphaP), np.sin(alphaP)*np.sin(gamaP) + np.sin(betaP)*np.cos(alphaP)*np.cos(gamaP), xP], 
					[np.sin(gamaP)*np.cos(betaP), np.sin(alphaP)*np.sin(betaP)*np.sin(gamaP) + np.cos(alphaP)*np.cos(gamaP), -np.sin(alphaP)*np.cos(gamaP) + np.sin(betaP)*np.sin(gamaP)*np.cos(alphaP), yP], 
					[-np.sin(betaP), np.sin(alphaP)*np.cos(betaP), np.cos(alphaP)*np.cos(betaP), zP],
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
	R03 =  [[np.sin(q2)*np.cos(q1)*np.cos(q3) + np.sin(q3)*np.cos(q1)*np.cos(q2), np.sin(q1), -np.sin(q2)*np.sin(q3)*np.cos(q1) + np.cos(q1)*np.cos(q2)*np.cos(q3)], 
			[np.sin(q1)*np.sin(q2)*np.cos(q3) + np.sin(q1)*np.sin(q3)*np.cos(q2), -np.cos(q1), -np.sin(q1)*np.sin(q2)*np.sin(q3) + np.sin(q1)*np.cos(q2)*np.cos(q3)], 
			[-np.sin(q2)*np.sin(q3) + np.cos(q2)*np.cos(q3)                     , 0          , -np.sin(q2)*np.cos(q3) - np.sin(q3)*np.cos(q2)]]

	IR03 = np.transpose(R03)

	R36 = np.dot(IR03, R06)

	q4, q5, q6 = get_last3Angles(R36)

	return q1, q2, q3, q4, q5, q6



################################################################################################################
################################################################################################################





################################################################################################################
################################################################################################################

# motion control of kusky


def speed_for_rotation(rotation, currOrientation):  # spiting out the rotation speed according to proportionality
	kp = 7
	Vmax = 5
	v = kp*(currOrientation - rotation)

	if(v>Vmax):
		v = Vmax
	elif(v<-Vmax):
		v = -Vmax

	# print("currOrientation, rotation, v : ", currOrientation, rotation, v)
	return v


def speed_for_forward(delX, delY):  # spiting out the forward speed according to proportionality
	kp = 10
	Vmax = 5
	v = kp*(abs(delX)+abs(delY))/2

	if(v>Vmax):
		v = Vmax
	elif(v<-Vmax):
		v = -Vmax

	return v


def get_target_parameters(x, y, z):

	currPosition = p.getLinkStates(husky, [0])[0][0]
	currOrientation = p.getEulerFromQuaternion(p.getLinkStates(husky, [0])[0][1])

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
	#(2, b'front_left_wheel', 0, 7, 6, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_left_wheel_link')
	#(3, b'front_right_wheel', 0, 8, 7, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_right_wheel_link')
	#(4, b'rear_left_wheel', 0, 9, 8, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_left_wheel_link')
	#(5, b'rear_right_wheel', 0, 10, 9, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_right_wheel_link')

	wheelVelocities = [0, 0, 0, 0]
	wheelDeltasTurn = [1, -1, 1, -1]
	wheelDeltasFwd = [1, 1, 1, 1]
	deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(x, y, z)


	# rotate the bot toward the destination
	while abs(rotation[2]-currOrientation[2])>= 0.005:
		# images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

		deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(x, y, z)

		wheelVelocities = [0, 0, 0, 0]

		vr = speed_for_rotation(rotation[2], currOrientation[2])

		for i in range(len(wheels)):
			wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i]

		for i in range(len(wheels)):
			p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)


	# move kukaId forward to destination 
	while abs(deltaX)>= 0.005 or abs(deltaY)>=0.005 :
		# images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

		deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(x, y, z)

		wheelVelocities = [0, 0, 0, 0]

		vr = speed_for_rotation(rotation[2], currOrientation[2])
		vf = speed_for_forward(deltaX, deltaY)

		for i in range(len(wheels)):
			wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i]

		for i in range(len(wheels)):
			wheelVelocities[i] = wheelVelocities[i] + vf * wheelDeltasFwd[i]

		for i in range(len(wheels)):
			p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)


	# stoping the motion 
	wheelVelocities = [0, 0, 0, 0]
	for i in range(len(wheels)):
		p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)


################################################################################################################
################################################################################################################





################################################################################################################
################################################################################################################

# motion control of end effector


def get_point_parameters(curr_joint_value, final_joint_value, t):

    inst_joint_value = np.array(curr_joint_value[:6]) + t*(np.array(final_joint_value) - np.array(curr_joint_value[:6]))
    return inst_joint_value


def move_endeffector_to_point(finalPoint, finalOrientation):

	kukaBasePosition = p.getLinkStates(kukaId, [0])[0][0]
	kukaBaseOrientation = p.getEulerFromQuaternion(p.getLinkStates(husky, [0])[0][1])
	final_joint_value = get_kuka_angles(kukaBasePosition, kukaBaseOrientation, finalPoint, finalOrientation)
	
	t = 0
	while t<=1:
		q1, q2, q3, q4, q5, q6 = get_point_parameters(curr_joint_value, final_joint_value, t)
		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= [q1, q2, q3, q4, q5, q6, curr_joint_value[6], curr_joint_value[7], curr_joint_value[8], curr_joint_value[9], curr_joint_value[10]])
		t += .00003   

	curr_joint_value[0]= final_joint_value[0]; curr_joint_value[1]= final_joint_value[1]; curr_joint_value[2]= final_joint_value[2] 
	curr_joint_value[3]= final_joint_value[3]; curr_joint_value[4]= final_joint_value[4]; curr_joint_value[5]= final_joint_value[5]



def hold(flag):
	if flag:
		curr_joint_value[7] = 1
		curr_joint_value[8] = 1
	else:
		curr_joint_value[7] = 0
		curr_joint_value[8] = 0

	p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)
	time.sleep(1)


################################################################################################################
################################################################################################################





################################################################################################################
################################################################################################################

# manual mode 


def manual_husky_kuka_control():
	time.sleep(2)
	sliderz1 = p.addUserDebugParameter("slider1", -np.pi, np.pi, curr_joint_value[0])
	sliderz2 = p.addUserDebugParameter("slider2", -np.pi, np.pi, curr_joint_value[1])
	sliderz3 = p.addUserDebugParameter("slider3", -np.pi, np.pi, curr_joint_value[2])
	sliderz4 = p.addUserDebugParameter("slider4", -np.pi, np.pi, curr_joint_value[3])
	sliderz5 = p.addUserDebugParameter("slider5", -np.pi, np.pi, curr_joint_value[4])
	sliderz6 = p.addUserDebugParameter("slider6", -np.pi, np.pi, curr_joint_value[5])
	sliderz7 = p.addUserDebugParameter("slider7", -np.pi, np.pi, curr_joint_value[7])
	q = []

	wheels = [2, 3, 4, 5]
	wheelVelocities = [0, 0, 0, 0]
	wheelDeltasTurn = [1, -1, 1, -1]
	wheelDeltasFwd = [1, 1, 1, 1]
	speed = 5

	while 1:

		keys = p.getKeyboardEvents()

		slider1 = p.readUserDebugParameter(sliderz1)
		slider2 = p.readUserDebugParameter(sliderz2)
		slider3 = p.readUserDebugParameter(sliderz3)
		slider4 = p.readUserDebugParameter(sliderz4)
		slider5 = p.readUserDebugParameter(sliderz5)
		slider6 = p.readUserDebugParameter(sliderz6)
		slider7 = p.readUserDebugParameter(sliderz7)

		curr_joint_value[0]= slider1; curr_joint_value[1]= slider2; curr_joint_value[2]= slider3; curr_joint_value[3]= slider4
		curr_joint_value[4]= slider5; curr_joint_value[5]= slider6; curr_joint_value[7]= slider7; curr_joint_value[8]= slider7

		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions=curr_joint_value)
		
		wheelVelocities = [0, 0, 0, 0]
		if p.B3G_LEFT_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasTurn[i]
		if p.B3G_RIGHT_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasTurn[i]
		if p.B3G_UP_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasFwd[i]
		if p.B3G_DOWN_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasFwd[i]

		for i in range(len(wheels)):
			p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)

		flag =0
		for k in keys:
			if ord('m') in keys:
				q = [slider1, slider2, slider3, slider4, slider5, slider6, 0, slider7, slider7, 0, 0]
				flag =1
				print("saving the valuse of joints")
				break
		if(flag):
			break

	print("the vanal valuse of joints are : ")
	print(q)


################################################################################################################
################################################################################################################





################################################################################################################
################################################################################################################

# picks the cube from given position


def calculate_position_for_husky(x2, y2, z2):
	x1, y1, z1 = p.getLinkStates(husky, [0])[0][0]

	t = 0.7 / np.sqrt((x1-x2)**2 + (y1-y2)**2 )
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





# pick_cube_from(cubePosition) will pick the cube from given point, wherever the robot is.

# place_cube_to(cubePosition)  will place the cube to the given point, wherever the robot is.

# move_husky_to_point(x, y, z) will thake the robot to the specified point using a st. line.

# move_endeffector_to_point(finalPoint, finalOrientation) will move the endeffector to the specified point and orientation

# manual_husky_kuka_control() will allow you to control the robot manualy, (move the husky by arrow keys, and move the arms by slider)

# take_photo() will return a top view of the plane

# imageFrame, positionsCube, positionsPatch = take_1photo()  --> will capture a top view imgae of arena, and return the processed image, and position of cubes and patchs in color order (R, G, B, O, Y, P).





start = time.time()

################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

# what you want from husky to do for you!
# write you code bellow :-)



for i in range(len(positionsCube)):
	pick_cube_from([positionsCube[i][0], positionsCube[i][1], 0])
	place_cube_to([positionsPatch[i][0], positionsPatch[i][1], 0])

move_husky_to_point(0,0,0)


################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

end = time.time()
print("total time taken : ", end-start)


time.sleep(10)
p.disconnect()
