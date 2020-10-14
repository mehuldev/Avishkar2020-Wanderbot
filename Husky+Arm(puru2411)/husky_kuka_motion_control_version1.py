# code by Purushotam Kumar Agrawal { git ---->  PURU2411 }

import pybullet as p
import time
import math
from datetime import datetime
from datetime import datetime
import pybullet_data
import numpy as np
from invkin_puru import *


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


p.loadURDF("plane.urdf", [0, 0, 0.0])
p.setGravity(0, 0, -10)

p.loadURDF('cube_small.urdf', basePosition=[.8, 0, 0.0], globalScaling=0.8)

# impoting the husky bot at 
huskyCenter = [0.0, 0.0, 0.0]
huskyOrientation = p.getQuaternionFromEuler([0,0,0])
husky = p.loadURDF("husky/husky.urdf", huskyCenter, huskyOrientation)


#importing kuka arm
kukaCenter = [0.0, 0.0, 0.24023]
kukaOrientation = p.getQuaternionFromEuler([0,0,0])
kukaId = p.loadURDF("kuka_experimental-indigo-devel/kuka_kr210_support/urdf/kr210l150.urdf", kukaCenter, kukaOrientation, globalScaling=0.4)
# setting innitialy 0 
curr_joint_value = [0,0,0,0,0,0,0,0,0,0,0]
p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions=curr_joint_value)

# puting kuka on husky
cid = p.createConstraint(husky, 1, kukaId, -1, p.JOINT_FIXED, [0, 0, 0], [0.0, 0.0, 0.14023], [0., 0., 0], [0.0, 0.0, 0.0])


useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)



width = 540
height = 540

fov = 200
aspect = width / height
near = 0.02
far = 10

view_matrix = p.computeViewMatrix([2, 2, .6], [2, 2, 0.1], [1, 0, 0])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)



print("the info of husky joints : ")
for i in range(p.getNumJoints(husky)):
  print(p.getJointInfo(husky, i))



print("the info of kuka joints : ")
for i in range(p.getNumJoints(kukaId)):
  print(p.getJointInfo(kukaId, i))



print(p.getLinkStates(kukaId, [6, 7, 8]))

time.sleep(2)



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
		images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

		deltaX, deltaY, rotation, initialOrientation, initialPosition = get_target_parameters(x, y, z)

		wheelVelocities = [0, 0, 0, 0]

		vr = speed_for_rotation(rotation[2], initialOrientation[2])

		for i in range(len(wheels)):
			wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i]

		for i in range(len(wheels)):
			p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)


	# move kukaId forward to destination 
	while abs(deltaX)>= 0.005 and abs(deltaY)>=0.005 :
		images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

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


def move_end_to(endPosition, endOrientation ):
	print("in move_end_to function")
	endOrientation = p.getQuaternionFromEuler(endOrientation)
	q1, q2, q3, q4, q5, q6, q7, q8 = p.calculateInverseKinematics(kukaId, 6, endPosition, endOrientation)
	targetJointValues = [q1, q2, q3, q4, q5, q6, 0, q7, q8, 0, 0]

	while abs(curr_joint_value[0] - targetJointValues[0]) >= .001 :
		curr_joint_value[0] -= .00001*(curr_joint_value[0] - targetJointValues[0])/abs(curr_joint_value[0] - targetJointValues[0])
		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)
		# time.sleep(.0001)

	while abs(curr_joint_value[5] - targetJointValues[5]) >= .0001 :
		curr_joint_value[5] -= .00001*(curr_joint_value[5] - targetJointValues[5])/abs(curr_joint_value[5] - targetJointValues[5])
		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)
		# time.sleep(.0001)

	while abs(curr_joint_value[4] - targetJointValues[4]) >= .0001 :
		curr_joint_value[4] -= .00001*(curr_joint_value[4] - targetJointValues[4])/abs(curr_joint_value[4] - targetJointValues[4])
		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)
		# time.sleep(.0001)

	while abs(curr_joint_value[3] - targetJointValues[3]) <= .0001 :
		curr_joint_value[3] -= .00001*(curr_joint_value[3] - targetJointValues[3])/abs(curr_joint_value[3] - targetJointValues[3])
		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)
		# time.sleep(.0001)

	while abs(curr_joint_value[2] - targetJointValues[2]) >= .0001 :
		curr_joint_value[2] -= .00001*(curr_joint_value[2] - targetJointValues[2])/abs(curr_joint_value[2] - targetJointValues[2])
		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)
		# time.sleep(.0001)

	while abs(curr_joint_value[1] - targetJointValues[1]) >= .0001 :
		curr_joint_value[1] -= .00001*(curr_joint_value[1] - targetJointValues[1])/abs(curr_joint_value[1] - targetJointValues[1])
		p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)
		# time.sleep(.0001)



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

		t += .001/(abs(initialPoint[0]-finalPoint[0]) + abs(initialPoint[1]-finalPoint[1]) + abs(initialPoint[2]-finalPoint[2]) + 
					abs(initialOrientation[0]-finalOrientation[0]) + abs(initialOrientation[1]-finalOrientation[1]) + abs(initialOrientation[2]-finalOrientation[2]))*6
	time.sleep(1)


def hold(flag):
	if flag:
		curr_joint_value[7] = 1
		curr_joint_value[8] = 1
	else:
		curr_joint_value[7] = 0
		curr_joint_value[8] = 0
	print('curr ', curr_joint_value)
	p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= curr_joint_value)
	time.sleep(1)



move_endeffector_to_point([.8, 0, .1], [0, np.pi/2, 0])

hold(1)


move_endeffector_to_point([.8, 0, 1], [0, 0, 0])

time.sleep(2)
hold(0)


# move_husky_to_point(2.4, 2.4, 0)
# time.sleep(1)

# finalPosition = p.getLinkStates(husky, [0])[0][0]
# finalOrientation = p.getLinkStates(husky, [0])[0][1]
# finalOrientation = p.getEulerFromQuaternion(finalOrientation)

# print("final position and orientation : ", finalPosition, finalOrientation)

# endPosition = [.8, 0, 0.1]
# endOrientation = [ 0.0, np.pi/2, 0.0]
# move_end_to(endPosition, endOrientation)
# time.sleep(1)
# hold(1)

# endPosition = [.8, 0, 1.2]
# endOrientation = [ 0.0, 0, 0.0]
# move_end_to(endPosition, endOrientation)
# time.sleep(1)
# hold(0)


# move_husky_to_point(0.6, .6, 0)

# time.sleep(1)
# hold(0)

# images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

# slider7 =1 ; slider8 =1
# jointValues = [slider1, slider2, slider3, slider4, slider5, slider6, 0, slider7, slider8, 0, 0]
# p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= jointValues)
# time.sleep(1)
# # p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions= [0,0,0,0,0,0,0,0,0,0,0])


# time.sleep(2)
# sliderz1 = p.addUserDebugParameter("slider1", -np.pi, np.pi, slider1)
# sliderz2 = p.addUserDebugParameter("slider2", -np.pi, np.pi, slider2)
# sliderz3 = p.addUserDebugParameter("slider3", -np.pi, np.pi, slider3)
# sliderz4 = p.addUserDebugParameter("slider4", -np.pi, np.pi, slider4)
# sliderz5 = p.addUserDebugParameter("slider5", -np.pi, np.pi, slider5)
# sliderz6 = p.addUserDebugParameter("slider6", -np.pi, np.pi, slider6)
# sliderz7 = p.addUserDebugParameter("slider7", -np.pi, np.pi, slider7)
# q = []

# while 1:
# 	keys = p.getKeyboardEvents()
# 	slider1 = p.readUserDebugParameter(sliderz1)
# 	slider2 = p.readUserDebugParameter(sliderz2)
# 	slider3 = p.readUserDebugParameter(sliderz3)
# 	slider4 = p.readUserDebugParameter(sliderz4)
# 	slider5 = p.readUserDebugParameter(sliderz5)
# 	slider6 = p.readUserDebugParameter(sliderz6)
# 	slider7 = p.readUserDebugParameter(sliderz7)
# 	p.setJointMotorControlArray( kukaId, range(11), p.POSITION_CONTROL, targetPositions=[slider1, slider2, slider3, slider4, slider5, slider6, 0, slider7, slider7, 0, 0])
	
# 	wheels = [2, 3, 4, 5]
# 	wheelVelocities = [0, 0, 0, 0]
# 	for i in range(len(wheels)):
# 		p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)


# 	flag =0
# 	for k in keys:
# 		if ord('s') in keys:
# 			q = [slider1, slider2, slider3, slider4, slider5, slider6, 0, slider7, slider7, 0, 0]
# 			flag =1
# 			print("saving the valuse of joints")
# 			break
# 	if(flag):
# 		break

# print("the vanal valuse of joints are : ")
# print(q)





time.sleep(10)
p.disconnect()
