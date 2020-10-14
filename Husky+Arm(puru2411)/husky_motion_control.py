# code by Purushotam Kumar Agrawal { git ---->  PURU2411 }

import pybullet as p
import time
import math
from datetime import datetime
from datetime import datetime
import pybullet_data
import numpy as np


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0.0])
p.setGravity(0, 0, -10)

# impoting the husky bot at 
huskyCenter = [0.0, 0.0, 0.0]
huskyOrientation = p.getQuaternionFromEuler([0,0,0])
print("husky orientation : ", huskyOrientation)
husky = p.loadURDF("husky/husky.urdf", huskyCenter, huskyOrientation)

useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)

print("the info of other joints: ")
for i in range(p.getNumJoints(husky)):
  print(p.getJointInfo(husky, i))

print(p.getLinkStates(husky, [0])[0][0])
print(p.getLinkStates(husky, [1]))

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



def goTo(x, y, z):

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
	while abs(rotation[2]-initialOrientation[2])>= 0.0001:

		deltaX, deltaY, rotation, initialOrientation, initialPosition = get_target_parameters(x, y, z)

		wheelVelocities = [0, 0, 0, 0]

		vr = speed_for_rotation(rotation[2], initialOrientation[2])

		for i in range(len(wheels)):
			wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i]

		for i in range(len(wheels)):
			p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)


	# move robot forward to destination 
	while abs(deltaX)>= 0.001 and abs(deltaY)>=0.001 :

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




goTo(5, -5, 0)

finalPosition = p.getLinkStates(husky, [0])[0][0]
finalOrientation = p.getLinkStates(husky, [0])[0][1]
finalOrientation = p.getEulerFromQuaternion(finalOrientation)

print(finalPosition, finalOrientation)


time.sleep(20)