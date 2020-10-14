# Wander bot { Team robomania }

import pybullet as p
import time
import pybullet_data


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


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


# creat constrain to mount kuka on husky bot, see pybullet documantation ofr more details


# deactivating real time simulation
useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)




start = time.time()
################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################

# what you want from wanderbot to do for you!
# write you code bellow :-)













################################################################################################################
################################################################################################################
################################################################################################################
################################################################################################################
end = time.time()
print("total time taken : ", end-start)


time.sleep(10)
p.disconnect()
