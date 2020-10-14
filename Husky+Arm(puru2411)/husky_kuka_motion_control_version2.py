# code by Purushotam Kumar Agrawal { git ---->  PURU2411 }
# this code uses the inbuilt inverse kinematics function


import pybullet as p
import time
import math
from datetime import datetime
from datetime import datetime
import pybullet_data
import numpy as np
from invkin_puru import *
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

	view_matrix = p.computeViewMatrix([0, 0, 2], [0, 0, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(12, -12, 12, -12, 1.93, 1.98)
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

	view_matrix = p.computeViewMatrix([6, 6, 2], [6, 6, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 1.93, 1.98)
	images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	cv2.imwrite('top_view1.jpg', images[2])

	view_matrix = p.computeViewMatrix([6, -6, 2], [6, -6, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 1.93, 1.98)
	images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	cv2.imwrite('top_view2.jpg', images[2])

	view_matrix = p.computeViewMatrix([-6, -6, 2], [-6, -6, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 1.93, 1.98)
	images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	cv2.imwrite('top_view3.jpg', images[2])

	view_matrix = p.computeViewMatrix([-6, 6, 2], [-6, 6, 0.0], [1, 0, 0])
	# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	projection_matrix = p.computeProjectionMatrix(6, -6, 6, -6, 1.93, 1.98)
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


# importing cube at
cubePosition = [-2, 2, 0]
cube = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cubePosition, globalScaling=0.8)
cube2 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= [2, 2, 0], globalScaling=0.8)
p.changeVisualShape(cube2 , -1, rgbaColor=[0, 1, 0, 1])
cube3 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= [2, -2, 0], globalScaling=0.8)
p.changeVisualShape(cube3 , -1, rgbaColor=[0, 0, 1, 1])
cube4 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= [-2, -2, 0], globalScaling=0.8)
p.changeVisualShape(cube4 , -1, rgbaColor=[1, 0, 0, 1])
cube5 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= [10, 10, 0], globalScaling=0.8)
p.changeVisualShape(cube5 , -1, rgbaColor=[1, 0, 0, 1])


# take_4photo()



# impoting the husky bot at 
huskyCenter = [0.0, 0.0, 0.0]
huskyOrientation = p.getQuaternionFromEuler([0,0,0])
husky = p.loadURDF("husky/husky.urdf", huskyCenter, huskyOrientation)

#importing kuka arm at
kukaCenter = [0.0, 0.0, 0.24023]
kukaOrientation = p.getQuaternionFromEuler([0,0,0])
kukaId = p.loadURDF("kuka_experimental-indigo-devel/kuka_kr210_support/urdf/kr210l150.urdf", kukaCenter, kukaOrientation, globalScaling=0.4)

# setting kuka initialy 0 
curr_joint_value = [0,0,0,0,0,0,0,0,0,0,0]
p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions=curr_joint_value)

# puting kuka on husky
cid = p.createConstraint(husky, 1, kukaId, -1, p.JOINT_FIXED, [0, 0, 0], [0.0, 0.0, 0.14023], [0., 0., 0], [0.0, 0.0, 0.0])

# activating real time simulation
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)


print("the joint info of husky : ")
for i in range(p.getNumJoints(husky)):
  print(p.getJointInfo(husky, i))

print("the joint info of kuka : ")
for i in range(p.getNumJoints(kukaId)):
  print(p.getJointInfo(kukaId, i))


print("the joint info of cube : ")
for i in range(p.getNumJoints(cube)):
  print(p.getJointInfo(cube, i))

print('cube ground state', p.getLinkStates(cube, [-1]))

# giving a cool off time of 2 sec
time.sleep(2)

################################################################################################################
################################################################################################################





################################################################################################################
################################################################################################################

# motion control of kusky


def speed_for_rotation(rotation, initialOrientation):  # spiting out the rotation speed according to proportionality
	kp = 8

	v = kp*(initialOrientation - rotation)

	if(v>5):
		v = 5
	elif(v<-5):
		v = -5

	return v


def speed_for_forward(delX, delY):  # spiting out the forward speed according to proportionality
	kp = 8

	v = kp*(abs(delX)+abs(delY))/2

	if(v>5):
		v = 5
	elif(v<-5):
		v = -5

	return v


def get_target_parameters(x, y, z):

	initialPosition = p.getLinkStates(husky, [0])[0][0]
	initialOrientation = p.getLinkStates(husky, [0])[0][1]
	initialOrientation = p.getEulerFromQuaternion(initialOrientation)

	deltaX = x - initialPosition[0]
	deltaY = y - initialPosition[1]

	rotation = [0, 0, np.arctan2(deltaY, deltaX)]

	return deltaX, deltaY, rotation, initialOrientation, initialPosition


def move_husky_to_point(x, y, z):
	print('moving husky to ', x, y, z)
	wheels = [2, 3, 4, 5]
	#(2, b'front_left_wheel', 0, 7, 6, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_left_wheel_link')
	#(3, b'front_right_wheel', 0, 8, 7, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_right_wheel_link')
	#(4, b'rear_left_wheel', 0, 9, 8, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_left_wheel_link')
	#(5, b'rear_right_wheel', 0, 10, 9, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_right_wheel_link')

	wheelVelocities = [0, 0, 0, 0]
	wheelDeltasTurn = [1, -1, 1, -1]
	wheelDeltasFwd = [1, 1, 1, 1]
	deltaX, deltaY, rotation, initialOrientation, initialPosition = get_target_parameters(x, y, z)


	# rotate the bot toward the destination
	while abs(rotation[2]-initialOrientation[2])>= 0.005:
		# images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

		deltaX, deltaY, rotation, initialOrientation, initialPosition = get_target_parameters(x, y, z)

		wheelVelocities = [0, 0, 0, 0]

		vr = speed_for_rotation(rotation[2], initialOrientation[2])

		for i in range(len(wheels)):
			wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i]

		for i in range(len(wheels)):
			p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)


	# move kukaId forward to destination 
	while abs(deltaX)>= 0.005 or abs(deltaY)>=0.005 :
		# images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

		deltaX, deltaY, rotation, initialOrientation, initialPosition = get_target_parameters(x, y, z)

		wheelVelocities = [0, 0, 0, 0]

		vr = speed_for_rotation(rotation[2], initialOrientation[2])
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


def get_point_parameters(initialPoint, finalPoint, initialOrientation, finalOrientation, t):
    v = 5
    currentposition = np.array(initialPoint) + t*(np.array(finalPoint) - np.array(initialPoint))
    currentOrientation = np.array(initialOrientation) + t*(np.array(finalOrientation) - np.array(initialOrientation))

    return currentposition[0], currentposition[1], currentposition[2], currentOrientation[0], currentOrientation[1], currentOrientation[2]


def move_endeffector_to_point(finalPoint, finalOrientation):
	initialPoint = p.getLinkStates(kukaId, [6])[0][0]
	initialOrientation = p.getLinkStates(kukaId, [6])[0][1]
	initialOrientation = p.getEulerFromQuaternion(initialOrientation)
	
	t = 0
	while t<1:
		px, py, pz, pitch, roll, yaw = get_point_parameters(initialPoint, finalPoint, initialOrientation, finalOrientation, t)
		endOrientation = p.getQuaternionFromEuler([pitch, roll, yaw])
		endPosition = [px, py, pz]
		q1, q2, q3, q4, q5, q6, q7, q8 = p.calculateInverseKinematics(kukaId, 6, endPosition, endOrientation)
        
		curr_joint_value[0]= q1; curr_joint_value[1]= q2; curr_joint_value[2]= q3; curr_joint_value[3]= q4; curr_joint_value[4]= q5; curr_joint_value[5]= q6
		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)

		t += .0008/(abs(initialPoint[0]-finalPoint[0]) + abs(initialPoint[1]-finalPoint[1]) + abs(initialPoint[2]-finalPoint[2]) + 
					abs(initialOrientation[0]-finalOrientation[0]) + abs(initialOrientation[1]-finalOrientation[1]) + abs(initialOrientation[2]-finalOrientation[2]))*6
	time.sleep(1)


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
	print('move husky to ',x, y, z)
	move_husky_to_point(x, y, z)

	initialOrientation = p.getLinkStates(kukaId, [6])[0][1]
	initialOrientation = p.getEulerFromQuaternion(initialOrientation)

	print("husky orientation ", initialOrientation)
	time.sleep(.5)
	move_endeffector_to_point([x2, y2, .2], [0, np.pi/2, 0])
	move_endeffector_to_point([x2, y2, .1], [0, np.pi/2, 0])
	hold(1)

	move_endeffector_to_point([x2, y2, 1], initialOrientation)



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

# pick_cube_from(cubePosition)
# move_husky_to_point(0,0,0)

move_endeffector_to_point([0.7, 0, 0.1], [0, np.pi/2, 0])


################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

time.sleep(20)
p.disconnect()
