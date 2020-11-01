import pybullet as p


def spawn_cube_and_patch(physicsClient):
	
	cube_position_ROGYBP = [[1, 1, 0.025], [-9, -2, 0.025], [-2, 5, 0.025], [8, -7, 0.025], [-3, 7, 0.025], [3, -7, 0.025]] 
	patch_position_ROGYBP = [[1.5, 0, 0.00025], [-6, -2, 0.00025], [-8, 8, 0.00025], [4, -2, 0.00025], [10, 0, 0.00025], [5, -7, 0.00025], ]

	# placing patchs to place the cube

	cubeR = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= patch_position_ROGYBP[0], useFixedBase=1, physicsClientId=physicsClient )
	p.changeVisualShape(cubeR , -1, rgbaColor=[1, 0, 0, 1])

	cubeO = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= patch_position_ROGYBP[1], useFixedBase=1, physicsClientId=physicsClient)
	p.changeVisualShape(cubeO , -1, rgbaColor=[1, 130/255, 0, 1])

	cubeG = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= patch_position_ROGYBP[2], useFixedBase=1, physicsClientId=physicsClient)
	p.changeVisualShape(cubeG , -1, rgbaColor=[0, 1, 0, 1])

	cubeY = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= patch_position_ROGYBP[3], useFixedBase=1, physicsClientId=physicsClient)
	p.changeVisualShape(cubeY , -1, rgbaColor=[1, 1, 0, 1])

	cubeB = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= patch_position_ROGYBP[4], useFixedBase=1, physicsClientId=physicsClient)
	p.changeVisualShape(cubeB , -1, rgbaColor=[0, 0, 1, 1])

	cubeP = p.loadURDF("cubes/urdf/placeingSquare.urdf", basePosition= patch_position_ROGYBP[5], useFixedBase=1, physicsClientId=physicsClient)
	p.changeVisualShape(cubeP , -1, rgbaColor=[1, 0, 1, 1])

	# importing cubes

	cube1 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube_position_ROGYBP[0], globalScaling=0.8, physicsClientId=physicsClient)
	p.changeVisualShape(cube1 , -1, rgbaColor=[1, 0, 0, 1])

	cube2 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube_position_ROGYBP[1], globalScaling=0.8, physicsClientId=physicsClient)
	p.changeVisualShape(cube2 , -1, rgbaColor=[1, 130.0/225.0, 0, 1])

	cube3 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube_position_ROGYBP[2], globalScaling=0.8, physicsClientId=physicsClient)
	p.changeVisualShape(cube3 , -1, rgbaColor=[0, 1, 0, 1])

	cube4 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube_position_ROGYBP[3], globalScaling=0.8, physicsClientId=physicsClient)
	p.changeVisualShape(cube4 , -1, rgbaColor=[1, 1, 0, 1])

	cube5 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube_position_ROGYBP[4], globalScaling=0.8, physicsClientId=physicsClient)
	p.changeVisualShape(cube5 , -1, rgbaColor=[0, 0, 1, 1])

	cube6 = p.loadURDF("cubes/urdf/cube_small.urdf", basePosition= cube_position_ROGYBP[5], globalScaling=0.8, physicsClientId=physicsClient)
	p.changeVisualShape(cube6 , -1, rgbaColor=[1, 0, 1, 1])
