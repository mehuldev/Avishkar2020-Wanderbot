import numpy as np



scale = 0.4

a1 = 0.75*scale
a2 = 0.35*scale
a3 = 1.25*scale
a4 = 0.054*scale
a5 = 1.5*scale
a6 = 0.303*scale


def get_hypotenuse(a, b):
  # calculate the longest side given the two shorter sides 
  # of a right triangle using pythagorean theorem
  return np.sqrt(a*a + b*b)


def get_cosine_law_angle(a, b, c):
    # given all sides of a triangle a, b, c
    # calculate angle gamma between sides a and b using cosine law

    gamma = np.arccos((a*a + b*b - c*c) / (2*a*b))

    return gamma



def griperCenter(px, py, pz, R06):                 # checked 
    # calculating griper center, see in arm diagram for detail
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

    # print(R36)

    if(R36[2, 2]>=1):
    	R36[2, 2] = 1
    elif(R36[2, 2]<=-1):
    	R36[2, 2] = -1

    q5 = np.arccos(R36[2, 2]) #R36[2, 2]/abs(R36[2, 2])*

    print("R36[2, 2] , arccos(R36[2, 2] : ", R36[2, 2], " ", np.arccos(R36[2, 2]))

    q6 = np.arctan2(-R36[2, 1], R36[2, 0])

    # temp = 0.001

    # if(-R36[2, 1] > temp and R36[2, 0] < -temp):
    # 	q6 = np.arctan(-R36[2, 1]/ R36[2, 0]) + np.pi
    # elif(-R36[2, 1] < -temp and R36[2, 0] < -temp):
    # 	q6 = np.arctan(-R36[2, 1]/ R36[2, 0]) -np.pi


    q4 = np.arctan2(-R36[1,2], -R36[0,2])

    # if(-R36[1,2] > temp and -R36[0,2] < -temp):
    # 	q4 = np.arctan(-R36[1,2]/ -R36[0,2]) + np.pi
    # elif(-R36[1,2] < -temp and -R36[0,2] < -temp):
    # 	q4 = np.arctan(-R36[1,2]/ -R36[0,2]) -np.pi


    # print("q4, q5, q6 : ", q4, q5, q6)
    return q4, q5, q6



def get_angles(px, py, pz, alpha, beta, gama):
	R6b = [[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]]


	R6a = np.array([[np.cos(beta)*np.cos(gama)                                         , -np.cos(beta)*np.sin(gama)                                         , np.sin(beta)              ],
					[np.sin(alpha)*np.sin(beta)*np.cos(gama)+np.cos(alpha)*np.sin(gama), -np.sin(alpha)*np.sin(beta)*np.sin(gama)+np.cos(alpha)*np.cos(gama), np.sin(alpha)*np.cos(beta)],
					[-np.cos(alpha)*np.sin(beta)*np.cos(gama)+np.sin(alpha)*np.sin(gama), np.cos(alpha)*np.sin(beta)*np.sin(gama)+np.sin(alpha)*np.cos(gama), np.cos(alpha)*np.cos(beta)]])

	R06 = np.dot(R6a,R6b)

	Xc, Yc, Zc = griperCenter(px, py, pz, R06)

	q1, q2, q3 = get_first3Angles(Xc, Yc, Zc)

	R03 =  [[np.sin(q2)*np.cos(q1)*np.cos(q3) + np.sin(q3)*np.cos(q1)*np.cos(q2),  np.sin(q1), -np.sin(q2)*np.sin(q3)*np.cos(q1) + np.cos(q1)*np.cos(q2)*np.cos(q3)], 
			[np.sin(q1)*np.sin(q2)*np.cos(q3) + np.sin(q1)*np.sin(q3)*np.cos(q2), -np.cos(q1), -np.sin(q1)*np.sin(q2)*np.sin(q3) + np.sin(q1)*np.cos(q2)*np.cos(q3)], 
			[-np.sin(q2)*np.sin(q3) + np.cos(q2)*np.cos(q3)                     ,  0         , -np.sin(q2)*np.cos(q3) - np.sin(q3)*np.cos(q2)                      ]]

	IR03 = np.transpose(R03)

	R36 = np.dot(IR03, R06)

	q4, q5, q6 = get_last3Angles(R36)

	return q1, q2, q3, q4, q5, q6




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






def main():
	px, py, pz = 1, 0, 1
	rx, ry, rz = 0, 0, 0

	q1, q2, q3, q4, q5, q6 = get_angles(px, py, pz, rx, ry, rz)

	print("q1, q2, q3, q4, q5, q6 : ", q1, q2, q3, q4, q5, q6)




if __name__ == "__main__":
    main()

