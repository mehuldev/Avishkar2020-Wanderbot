# code by Purushotam Kumar Agrawal { git ---->  PURU2411 }
# this code uses the external inverse kinematics function
# simple pick and place without image proseccing

import pybullet as p
import time
import math
from datetime import datetime
from datetime import datetime
import pybullet_data
import numpy as np
import cv2



clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


################################################################################################################
################################################################################################################

# loading and setting urdfs


p.loadURDF("plane.urdf", [0, 0, 0.0])
p.setGravity(0, 0, -10)


# setting camera parameters
def take_1photo(): 
	width = 3000
	height = 3000

	fov = 200
	aspect = width / height
	near = 0.02
	far = 10

	view_matrix = p.computeViewMatrix([0, 0, 50], [0, 0, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(12, -12, 12, -12, 48.5, 51.1)
	images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	cv2.imwrite('top_view1.jpg', images[2])
	return images


# setting camera parameters for 4 camera takeup
def take_4photo(): 
	width = 2000
	height = 2000

	fov = 200
	aspect = width / height
	near = 0.02
	far = 10

	view_matrix = p.computeViewMatrix([6, 6, 50], [6, 6, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 48.5, 51.1)
	images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	cv2.imwrite('top_view1.jpg', images[2])

	view_matrix = p.computeViewMatrix([6, -6, 50], [6, -6, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 48.5, 51.1)
	images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	cv2.imwrite('top_view2.jpg', images[2])

	view_matrix = p.computeViewMatrix([-6, -6, 50], [-6, -6, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 48.5, 51.1)
	images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	cv2.imwrite('top_view3.jpg', images[2])

	view_matrix = p.computeViewMatrix([-6, 6, 50], [-6, 6, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 48.5, 51.1)
	images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	cv2.imwrite('top_view4.jpg', images[2])
	return images



# making boundary 
cubea = p.loadURDF("cubes/urdf/cube_small_extended_in_Y_direction.urdf", basePosition= [11.25, 0, 0.1], useFixedBase=1 )
p.changeVisualShape(cubea , -1, rgbaColor=[0, 0, 0, 1])
cubeb = p.loadURDF("cubes/urdf/cube_small_extended_in_Y_direction.urdf", basePosition= [-11.25, 0, 0.1] , useFixedBase=1)
p.changeVisualShape(cubeb , -1, rgbaColor=[0, 0, 0, 1])
cubec = p.loadURDF("cubes/urdf/cube_small_extended_in_X_direction.urdf", basePosition= [0, 11.25, 0.1] , useFixedBase=1)
p.changeVisualShape(cubec , -1, rgbaColor=[0, 0, 0, 1])
cubed = p.loadURDF("cubes/urdf/cube_small_extended_in_X_direction.urdf", basePosition= [0, -11.25, 0.1], useFixedBase=1 )
p.changeVisualShape(cubed , -1, rgbaColor=[0, 0, 0, 1])


# placing squares to place the cube
cubeRPosition = [2, 2, 0.00025]
cubeR = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= cubeRPosition, useFixedBase=1)
p.changeVisualShape(cubeR , -1, rgbaColor=[1, 0, 0, 1])

cubeGPosition = [-2, -2, 0.00025]
cubeG = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= cubeGPosition, useFixedBase=1)
p.changeVisualShape(cubeG , -1, rgbaColor=[0, 1, 0, 1])

# importing cube at
cube1Position = [6, 5, 0.025]
cube1 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube1Position, globalScaling=0.8)
p.changeVisualShape(cube1 , -1, rgbaColor=[1, 0, 0, 1])

cube2Position = [-6, -5, 0.025]
cube2 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube2Position, globalScaling=0.8)
p.changeVisualShape(cube2 , -1, rgbaColor=[0, 1, 0, 1])

# cube6 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= [2, 2, 0], globalScaling=0.8)
# p.changeVisualShape(cube6 , -1, rgbaColor=[0, 1, 0, 1])
# cube3 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= [2, -2, 0], globalScaling=0.8)
# p.changeVisualShape(cube3 , -1, rgbaColor=[0, 0, 1, 1])
# cube4 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= [-2, -2, 0], globalScaling=0.8)
# p.changeVisualShape(cube4 , -1, rgbaColor=[1, 0, 0, 1])
# cube5 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= [10, 10, 0], globalScaling=0.8)
# p.changeVisualShape(cube5 , -1, rgbaColor=[1, 0, 0, 1])




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
time.sleep(2)


take_4photo()
# take_1photo()


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
	kp = 10
	Vmax = 5
	v = kp*(currOrientation - rotation)

	if(v>Vmax):
		v = Vmax
	elif(v<-Vmax):
		v = -Vmax

	# print("currOrientation, rotation, v : ", currOrientation, rotation, v)
	return v


def speed_for_forward(delX, delY):  # spiting out the forward speed according to proportionality
	kp = 8
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
		t += .0001   

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




# pick_cube_from() will pick the cube from given point, wherever the robot is.
# move_husky_to_point() will thake the robot to the specified point
# move_endeffector_to_point(finalPoint, finalOrientation) will move the endeffector to the specified point and orientation
# manual_husky_kuka_control() will allow you to control the robot manualy, (move the husky by arrow keys, and move the arms by slider)
# take_photo() will return a top view of the plane




################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

# what you want from husky to do for you!
# write you code bellow :-)

# img = take_photo()

# manual_husky_kuka_control()
# move_husky_to_point(-10, -10, 0)

start = time.time()

print('initial position of RED cube is : ', cube1Position)
pick_cube_from(cube1Position)
place_cube_to(cubeRPosition)

end = time.time()

print('RED cube is placed at : ', p.getLinkStates(cube1, [0])[0][0])
print("total time taken : ", end-start)

print("\n")
start = time.time()
print('initial position of GREEN cube is : ', cube2Position)
pick_cube_from(cube2Position)
place_cube_to(cubeGPosition)

end = time.time()

print('red cube is placed at : ', p.getLinkStates(cube2, [0])[0][0])
print("total time taken : ", end-start)



# move_husky_to_point(-1,0.1,0)

# move_endeffector_to_point([.8, 0, .5], [0, np.pi/2, 0])
# move_endeffector_to_point([0.8, 0, .1], [0, np.pi/2, 0])
# print('cube ground state', p.getLinkStates(cube, [0])[0][0])


################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

time.sleep(10)
p.disconnect()
