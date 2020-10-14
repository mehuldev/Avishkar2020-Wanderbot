import numpy as np
# from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2, Matrix

# c, a, b = symbols('alphaB betaB gamaB', real=True)


# # rotation matrix after rotating around y-axis (pitch)
# A = Matrix([[cos(a), 0, sin(a)], [0, 1, 0], [-sin(a), 0, cos(a)]])

# # rotation matrix after rotating around z-axis (roll)
# B = Matrix([[cos(b), -sin(b), 0], [sin(b), cos(b), 0], [0, 0, 1]])

# # rotation matrix after rotating around x-axis (yaw)
# C = Matrix([[1, 0, 0], [0, cos(c), -sin(c)], [0, sin(c), cos(c)]])

# # after total rotation of pitch, roll and yaw
# D = B*A*C

# print(D)


# arr = np.array([[np.cos(betaP)*np.cos(gamaP), np.sin(alphaP)*np.sin(betaP)*np.cos(gamaP) - np.sin(gamaP)*np.cos(alphaP), np.sin(alphaP)*np.sin(gamaP) + np.sin(betaP)*np.cos(alphaP)*np.cos(gamaP), x], 
# [np.sin(gamaP)*np.cos(betaP), np.sin(alphaP)*np.sin(betaP)*np.sin(gamaP) + np.cos(alphaP)*np.cos(gamaP), -np.sin(alphaP)*np.cos(gamaP) + np.sin(betaP)*np.sin(gamaP)*np.cos(alphaP), y], 
# [-np.sin(betaP), np.sin(alphaP)*np.cos(betaP), np.cos(alphaP)*np.cos(betaP),  z],
# [0, 0, 0, 1]])

arr = np.array([[1, 2, 3, 4 ],
				[2, 3, 4, 5],
				[3, 4, 50, 6],
				[4, 5, 6, 7] ])

[Px, Py, Pz] = arr[:3, 3]

print(Px , Py, Pz)


		kukaBasePosition = p.getLinkStates(kukaId, [0])[0][0]
		kukaBaseOrientation = p.getEulerFromQuaternion(p.getLinkStates(kukaId, [0])[0][1])
		q1, q2, q3, q4, q5, q6 = get_kuka_angles(kukaBasePosition, kukaBaseOrientation, endPosition, endOrientation)
