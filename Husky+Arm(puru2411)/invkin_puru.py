# code written by Purushotam Kumar Agrawal { git ---> PURU2411 }

from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time
import numpy as np


# declaring the link length of the arm

scale = .4

a1 = 0.75*scale
a2 = 0.35*scale
a3 = 1.25*scale
a4 = 0.054*scale
a5 = 1.5*scale
a6 = 0.303*scale

dt = 0.001  # choosing dt = 1 ms

################################################################################################################
################################################################################################################
# forward kinematics is to verify the output and plot the graph

def forward_kin(q1, q2, q3, q4, q5, q6):
    X = []
    Y = []
    Z = []

    # DH parameter table of the given robotic arm see figure
    DH = np.array([[q1        , -np.pi/2, a2 , a1],
                   [q2-np.pi/2, 0       , a3 , 0],
                   [q3        , -np.pi/2, -a4, 0],
                   [q4        , np.pi/2 , 0  , a5],
                   [q5        , -np.pi/2, 0  , 0 ],
                   [q6        , 0       , 0  , a6]])

    # homogeneous matrices
    H0_0 = Matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    H0_1 = Matrix([[np.cos(DH[0, 0]), -np.sin(DH[0, 0]) * np.cos(DH[0, 1]), np.sin(DH[0, 0]) * np.sin(DH[0, 1]), DH[0, 2] * np.cos(DH[0, 0])],
                   [np.sin(DH[0, 0]), np.cos(DH[0, 0]) * np.cos(DH[0, 1]), -np.cos(DH[0, 0]) * np.sin(DH[0, 1]), DH[0, 2] * np.sin(DH[0, 0])],
                   [0, np.sin(DH[0, 1]), np.cos(DH[0, 1]), DH[0, 3]],
                   [0, 0, 0, 1]])

    H1_2 = Matrix([[np.cos(DH[1, 0]), -np.sin(DH[1, 0]) * np.cos(DH[1, 1]), np.sin(DH[1, 0]) * np.sin(DH[1, 1]), DH[1, 2] * np.cos(DH[1, 0])],
                   [np.sin(DH[1, 0]), np.cos(DH[1, 0]) * np.cos(DH[1, 1]), -np.cos(DH[1, 0]) * np.sin(DH[1, 1]), DH[1, 2] * np.sin(DH[1, 0])],
                   [0, np.sin(DH[1, 1]), np.cos(DH[1, 1]), DH[1, 3]],
                   [0, 0, 0, 1]])

    H0_2 = np.dot(H0_1, H1_2)

    H2_3 = Matrix([[np.cos(DH[2, 0]), -np.sin(DH[2, 0]) * np.cos(DH[2, 1]), np.sin(DH[2, 0]) * np.sin(DH[2, 1]), DH[2, 2] * np.cos(DH[2, 0])],
                   [np.sin(DH[2, 0]), np.cos(DH[2, 0]) * np.cos(DH[2, 1]), -np.cos(DH[2, 0]) * np.sin(DH[2, 1]), DH[2, 2] * np.sin(DH[2, 0])],
                   [0, np.sin(DH[2, 1]), np.cos(DH[2, 1]), DH[2, 3]],
                   [0, 0, 0, 1]])

    H0_3 = np.dot(H0_2, H2_3)

    H3_4 = Matrix([[np.cos(DH[3, 0]), -np.sin(DH[3, 0]) * np.cos(DH[3, 1]), np.sin(DH[3, 0]) * np.sin(DH[3, 1]), DH[3, 2] * np.cos(DH[3, 0])],
                   [np.sin(DH[3, 0]), np.cos(DH[3, 0]) * np.cos(DH[3, 1]), -np.cos(DH[3, 0]) * np.sin(DH[3, 1]), DH[3, 2] * np.sin(DH[3, 0])],
                   [0, np.sin(DH[3, 1]), np.cos(DH[3, 1]), DH[3, 3]],
                   [0, 0, 0, 1]])

    H0_4 = np.dot(H0_3, H3_4)

    H4_5 = Matrix([[np.cos(DH[4, 0]), -np.sin(DH[4, 0]) * np.cos(DH[4, 1]), np.sin(DH[4, 0]) * np.sin(DH[4, 1]), DH[4, 2] * np.cos(DH[4, 0])],
                   [np.sin(DH[4, 0]), np.cos(DH[4, 0]) * np.cos(DH[4, 1]), -np.cos(DH[4, 0]) * np.sin(DH[4, 1]), DH[4, 2] * np.sin(DH[4, 0])],
                   [0, np.sin(DH[4, 1]), np.cos(DH[4, 1]), DH[4, 3]],
                   [0, 0, 0, 1]])

    H0_5 = np.dot(H0_4, H4_5)

    H5_6 = Matrix([[np.cos(DH[5, 0]), -np.sin(DH[5, 0]) * np.cos(DH[5, 1]), np.sin(DH[5, 0]) * np.sin(DH[5, 1]), DH[5, 2] * np.cos(DH[5, 0])],
                   [np.sin(DH[5, 0]), np.cos(DH[5, 0]) * np.cos(DH[5, 1]), -np.cos(DH[5, 0]) * np.sin(DH[5, 1]), DH[5, 2] * np.sin(DH[5, 0])],
                   [0, np.sin(DH[5, 1]), np.cos(DH[5, 1]), DH[5, 3]],
                   [0, 0, 0, 1]])

    H0_6 = np.dot(H0_5, H5_6)

    # print("R0_6 comes out to be: ")
    # print(np.matrix(H0_6[:3, :3]))

    X.append(0)
    X.append(0)
    X.append(H0_1[0, 3])
    X.append(H0_2[0, 3])
    X.append(H0_3[0, 3])
    X.append(H0_4[0, 3])
    X.append(H0_5[0, 3])
    X.append(H0_6[0, 3])

    Y.append(0)
    Y.append(0)
    Y.append(H0_1[1, 3])
    Y.append(H0_2[1, 3])
    Y.append(H0_3[1, 3])
    Y.append(H0_4[1, 3])
    Y.append(H0_5[1, 3])
    Y.append(H0_6[1, 3])

    Z.append(0)
    Z.append(a1)
    Z.append(H0_1[2, 3])
    Z.append(H0_2[2, 3])
    Z.append(H0_3[2, 3])
    Z.append(H0_4[2, 3])
    Z.append(H0_5[2, 3])
    Z.append(H0_6[2, 3])

    # center of all the frames in ground frame
    X = np.reshape(X, (1, 8))
    Y = np.reshape(Y, (1, 8))
    Z = np.reshape(Z, (1, 8))

    return X, Y, Z



def create_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    ax.set_autoscale_on(1)
    # fig.canvas.draw()
    # plt.show()
    return fig, ax


def update_plot(X, Y, Z, X1, Y1, Z1,  fig, ax):
    X = np.reshape(X, (1, 8))
    Y = np.reshape(Y, (1, 8))
    Z = np.reshape(Z, (1, 8))
    ax.cla()
    ax.plot_wireframe(X, Y, Z)
    ax.plot_wireframe(X1, Y1, Z1, color = 'r')
    # print('in update ', X1, Y1, Z1)
    plt.draw()
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    ax.set_autoscale_on(1)
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(.001)
    # ax.cla()
    # ax.plot_wireframe(Z,Y,X,color='r')
    # plt.pause(.5)
    # ax.plot_wireframe(Z,Y,X,color='b')
    # plt.pause(3)
    # plt.show()

################################################################################################################
################################################################################################################


# this function is used only to calculate some of the matrices used in it's original trigonometric form like R0_3 and R3_6
def printMatrices():
    a, b, c = symbols('alpha beta gama', real=True)
    # rotation matrix after rotating around y-axis (pitch)
    A = Matrix([[cos(a), 0, sin(a)], [0, 1, 0], [-sin(a), 0, cos(a)]])

    # rotation matrix after rotating around z-axis (roll)
    B = Matrix([[cos(b), -sin(b), 0], [sin(b), cos(b), 0], [0, 0, 1]])

    # rotation matrix after rotating around x-axis (yaw)
    C = Matrix([[1, 0, 0], [0, cos(c), -sin(c)], [0, sin(c), cos(c)]])

    # after total rotation of pitch, roll and yaw
    D = A * B * C
    # print(D)

    q1, q2, q3, q4, q5, q6 = symbols('q1:7')
    a1, a2, a3, a4, a5, a6, a7 = symbols('a1:8')

    # DH parameter of the given arm
    DH = np.array([[q1     , -pi/2, a2 , a1],
                   [q2-pi/2, 0    , a3 , 0],
                   [q3     , -pi/2, -a4, 0],
                   [q4     , pi/2 , 0  , a5],
                   [q5     , -pi/2, 0  , 0 ],
                   [q6     , 0    , 0  , a6]])

    # homogeneous matrices
    H0_1 = Matrix([[cos(DH[0,0]), -sin(DH[0,0])*cos(DH[0,1]), sin(DH[0,0])*sin(DH[0,1]), DH[0,2]*cos(DH[0,0])],
                  [sin(DH[0,0]), cos(DH[0,0])*cos(DH[0,1]), -cos(DH[0,0])*sin(DH[0,1]), DH[0,2]*sin(DH[0,0])],
                  [0,            sin(DH[0,1]),               cos(DH[0,1]),              DH[0,3]             ],
                  [0,            0,                          0,                         1                   ]])

    H1_2 = Matrix([[cos(DH[1,0]), -sin(DH[1,0])*cos(DH[1,1]), sin(DH[1,0])*sin(DH[1,1]), DH[1,2]*cos(DH[1,0])],
                  [sin(DH[1,0]), cos(DH[1,0])*cos(DH[1,1]), -cos(DH[1,0])*sin(DH[1,1]), DH[1,2]*sin(DH[1,0])],
                  [0,            sin(DH[1,1]),               cos(DH[1,1]),              DH[1,3]             ],
                  [0,            0,                          0,                         1                   ]])

    H2_3 = Matrix([[cos(DH[2,0]), -sin(DH[2,0])*cos(DH[2,1]), sin(DH[2,0])*sin(DH[2,1]), DH[2,2]*cos(DH[2,0])],
                  [sin(DH[2,0]), cos(DH[2,0])*cos(DH[2,1]), -cos(DH[2,0])*sin(DH[2,1]), DH[2,2]*sin(DH[2,0])],
                  [0,            sin(DH[2,1]),               cos(DH[2,1]),              DH[2,3]             ],
                  [0,            0,                          0,                         1                   ]])

    H3_4 = Matrix([[cos(DH[3,0]), -sin(DH[3,0])*cos(DH[3,1]), sin(DH[3,0])*sin(DH[3,1]), DH[3,2]*cos(DH[3,0])],
                  [sin(DH[3,0]), cos(DH[3,0])*cos(DH[3,1]), -cos(DH[3,0])*sin(DH[3,1]), DH[3,2]*sin(DH[3,0])],
                  [0,            sin(DH[3,1]),               cos(DH[3,1]),              DH[3,3]             ],
                  [0,            0,                          0,                         1                   ]])

    H4_5 = Matrix([[cos(DH[4,0]), -sin(DH[4,0])*cos(DH[4,1]), sin(DH[4,0])*sin(DH[4,1]), DH[4,2]*cos(DH[4,0])],
                  [sin(DH[4,0]), cos(DH[4,0])*cos(DH[4,1]), -cos(DH[4,0])*sin(DH[4,1]), DH[4,2]*sin(DH[4,0])],
                  [0,            sin(DH[4,1]),               cos(DH[4,1]),              DH[4,3]             ],
                  [0,            0,                          0,                         1                   ]])

    H5_6 = Matrix([[cos(DH[5,0]), -sin(DH[5,0])*cos(DH[5,1]), sin(DH[5,0])*sin(DH[5,1]), DH[5,2]*cos(DH[5,0])],
                  [sin(DH[5,0]), cos(DH[5,0])*cos(DH[5,1]), -cos(DH[5,0])*sin(DH[5,1]), DH[5,2]*sin(DH[5,0])],
                  [0,            sin(DH[5,1]),               cos(DH[5,1]),              DH[5,3]             ],
                  [0,            0,                          0,                         1                   ]])

    H0_6 = H0_1*H1_2*H2_3*H3_4*H4_5*H5_6
    # print(H0_6)

    # print(H0_1)
    # print(H1_2)
    # print(H2_3)
    # print(H3_4)
    # print(H4_5)
    # print(H5_6)

    # rotation matrices
    R0_1 = H0_1[:3, :3]
    R0_2 = R0_1*H1_2[:3, :3]
    R0_3 = R0_2*H2_3[:3, :3]
    R0_4 = R0_3*H3_4[:3, :3]
    R0_5 = R0_4*H4_5[:3, :3]
    R0_6 = R0_5*H5_6[:3, :3]
    # print(R0_1)
    # print(R0_2)
    print(R0_3)
    # print(R0_4)
    # print(R0_5)
    # print(R0_6)

    R36 = H3_4[:3, :3]*H4_5[:3, :3]*H5_6[:3, :3]
    print(R36)


def get_cosine_law_angle(a, b, c):
    # given all sides of a triangle a, b, c
    # calculate angle gamma between sides a and b using cosine law

    gamma = np.arccos((a*a + b*b - c*c) / (2*a*b))

    return gamma


def griperCenter(px, py, pz, R06):
    # calculating griper center, see in arm diagram for detail
    Xc = px - a6*R06[0,2]
    Yc = py - a6*R06[1,2]
    Zc = pz - a6*R06[2,2]
    return Xc, Yc, Zc


def calcFirst3Angles(Xc, Yc, Zc):
    # doing inverse kinematics on first 3 dof the reach the center of griper
    # see the calculation page of inverse kinematics for more details

    q1 = np.arctan2(Yc, Xc)

    r1 = np.sqrt(Xc**2 + Yc**2)
    r2 = np.sqrt((r1-a2)**2 + (Zc-a1)**2)

    phi1 = np.arctan((Zc-a1)/(r1-a2))
    phi2 = get_cosine_law_angle((a3-a4), r2, a5)
    q2 = np.pi/2 - phi1 - phi2

    phi3 = get_cosine_law_angle((a3-a4), a5, r2)
    q3 = np.pi/2 - phi3

    return q1, q2, q3


def calcLast3Angles(R36):
    # evaluating last 3 angles by comparing the matrices

    # R3_6 =  Matrix([[-np.sin(q4)*np.sin(q6) + np.cos(q4)*np.cos(q5)*np.cos(q6), -np.sin(q4)*np.cos(q6) - np.sin(q6)*np.cos(q4)*np.cos(q5), -np.sin(q5)*np.cos(q4)], 
                      # [np.sin(q4)*np.cos(q5)*np.cos(q6) + np.sin(q6)*np.cos(q4), -np.sin(q4)*np.sin(q6)*np.cos(q5) + np.cos(q4)*np.cos(q6), -np.sin(q4)*np.sin(q5)], 
                      # [np.sin(q5)*np.cos(q6), -np.sin(q5)*np.sin(q6), np.cos(q5)]]
    
    print("R36 : ", R36)



    if(R36[2,2]>1.0):
      # print("R36[2,2] : ", R36[2,2])
      R36[2,2] = 1.0
    elif(R36[2,2]<-1.0):
      # print("R36[2,2] : ", R36[2,2])
      R36[2,2] = -1.0

    if(R36[2,2] >= 0):
      q5 = np.arccos(R36[2,2])
    else:
      q5 = - np.arccos(R36[2,2])









    if(R36[1,2]<.001 or R36[1,2]>-.001):
      R36[1,2] = 0.000001

    # if(R36[0,2]<.001 or R36[0,2]>-.001):
    #   R36[0,2] = 0.0
    
    if(R36[2,1]<.001 or R36[2,1]>-.001):
      R36[2,1] = 0.000001

    # if(R36[2,0]<.001 or R36[2,0]>-.001):
    #   R36[2,0] = 0.0

    q4 = np.arctan2(R36[1,2], R36[0, 2])

    q6 = np.arctan2(R36[2,1], -R36[2,0])

    return q4, q5, q6


def get_angles_puru(px, py, pz, alpha, beta, gama):

    # the frame of griper is pre-rotated from bellow rotation matrix
    R6a = [[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]]

    # after rotation of pitch, roll, yaw
    R6b = [[np.cos(alpha)*np.cos(beta), np.sin(alpha)*np.sin(gama) - np.sin(beta)*np.cos(alpha)*np.cos(gama), np.sin(alpha)*np.cos(gama) + np.sin(beta)*np.sin(gama)*np.cos(alpha)],
           [np.sin(beta)           , np.cos(beta)*np.cos(gama)                                  , -np.sin(gama)*np.cos(beta)],
           [-np.sin(alpha)*np.cos(beta), np.sin(alpha)*np.sin(beta)*np.cos(gama) + np.sin(gama)*np.cos(alpha), -np.sin(alpha)*np.sin(beta)*np.sin(gama) + np.cos(alpha)*np.cos(gama)]]
    # total rotation of griper frame WRT ground frame
    R06 = np.dot(R6b,R6a)
    # print(np.matrix(R06))

    # calculating center of griper
    Xc, Yc, Zc = griperCenter(px, py, pz, R06)
    # print("Xc, Yc, Zc : ", Xc, Yc, Zc)

    # calculating first 3 angles
    q1, q2, q3 = calcFirst3Angles(Xc, Yc, Zc)

    # rotation matrix of 3 wrt 0 frame  see the calculation sheet for more understanding

    R03 =  [[np.sin(q2)*np.cos(q1)*np.cos(q3) + np.sin(q3)*np.cos(q1)*np.cos(q2),  np.sin(q1), -np.sin(q2)*np.sin(q3)*np.cos(q1) + np.cos(q1)*np.cos(q2)*np.cos(q3)], 
            [np.sin(q1)*np.sin(q2)*np.cos(q3) + np.sin(q1)*np.sin(q3)*np.cos(q2), -np.cos(q1), -np.sin(q1)*np.sin(q2)*np.sin(q3) + np.sin(q1)*np.cos(q2)*np.cos(q3)], 
            [-np.sin(q2)*np.sin(q3) + np.cos(q2)*np.cos(q3)                     ,  0         , -np.sin(q2)*np.cos(q3) - np.sin(q3)*np.cos(q2)                      ]]

    IR03 = np.transpose(R03)

    R36 = np.dot(IR03, R06)

    q4, q5, q6 = calcLast3Angles(R36)

    return q1, q2, q3, q4, q5, q6


def get_kuka_angles(basePosition, baseOrientation, point, orientation):
  
  xB = basePosition[0] + 0.015772399999437497
  yB = basePosition[1] + 0.009488456000838417
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

  Hgb = np.array([[np.cos(betaB)*np.cos(gamaB)                                            , -np.cos(betaB)*np.sin(gamaB)                                            , np.sin(betaB)               , xB],
                  [np.sin(alphaB)*np.sin(betaB)*np.cos(gamaB)+np.cos(alphaB)*np.sin(gamaB), -np.sin(alphaB)*np.sin(betaB)*np.sin(gamaB)+np.cos(alphaB)*np.cos(gamaB), np.sin(alphaB)*np.cos(betaB), yB],
                  [-np.cos(alphaB)*np.sin(betaB)*np.cos(gamaB)+np.sin(alphaB)*np.sin(gamaB), np.cos(alphaB)*np.sin(betaB)*np.sin(gamaB)+np.sin(alphaB)*np.cos(gamaB), np.cos(alphaB)*np.cos(betaB), zB],
                  [0, 0, 0, 1]])
  

  R6b = [[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]]


  R6a = np.array([[np.cos(betaP)*np.cos(gamaP)                                         , -np.cos(betaP)*np.sin(gamaP)                                         , np.sin(betaP)              ],
                  [np.sin(alphaP)*np.sin(betaP)*np.cos(gamaP)+np.cos(alphaP)*np.sin(gamaP), -np.sin(alphaP)*np.sin(betaP)*np.sin(gamaP)+np.cos(alphaP)*np.cos(gamaP), np.sin(alphaP)*np.cos(betaP)],
                  [-np.cos(alphaP)*np.sin(betaP)*np.cos(gamaP)+np.sin(alphaP)*np.sin(gamaP), np.cos(alphaP)*np.sin(betaP)*np.sin(gamaP)+np.sin(alphaP)*np.cos(gamaP), np.cos(alphaP)*np.cos(betaP)]])

  # Rgp = np.dot(R6a, R6b)
  Rgp = R6a

  Hgp = np.concatenate(( Rgp, [[xP], [yP], [zP]] ), axis= 1)
  Hgp = np.concatenate(( Hgp, [[0, 0, 0, 1]] ), axis= 0)

  # Hgp = np.array([[np.cos(alphaP)*np.cos(betaP), np.sin(alphaP)*np.sin(gamaP) - np.sin(betaP)*np.cos(alphaP)*np.cos(gamaP), np.sin(alphaP)*np.cos(gamaP) + np.sin(betaP)*np.sin(gamaP)*np.cos(alphaP), xP],
  #                 [np.sin(betaP)           , np.cos(betaP)*np.cos(gamaP)                                  , -np.sin(gamaP)*np.cos(betaP), yP],
  #                 [-np.sin(alphaP)*np.cos(betaP), np.sin(alphaP)*np.sin(betaP)*np.cos(gamaP) + np.sin(gamaP)*np.cos(alphaP), -np.sin(alphaP)*np.sin(betaP)*np.sin(gamaP) + np.cos(alphaP)*np.cos(gamaP), zP],
  #                 [0, 0, 0, 1]])

  IHgb = np.linalg.inv(Hgb)
  Hbp = np.dot(IHgb, Hgp)

  px, py, pz = Hbp[0, 3], Hbp[1, 3], Hbp[2, 3]

  R06 = Hbp[:3, :3]

  # print("base : ", basePosition, baseOrientation)
  # print("point in global: ", point, orientation)
  # print("point in base : ", [px, py, pz], R06)
  # R06 = np.dot(R6b, R6a)
  # R06 = R6a

  # calculating center of griper
  Xc, Yc, Zc = griperCenter(px, py, pz, R06)
  # print("Xc, Yc, Zc : ", Xc, Yc, Zc)

  # calculating first 3 angles
  q1, q2, q3 = calcFirst3Angles(Xc, Yc, Zc)

  # rotation matrix of 3 wrt 0 frame  see the calculation sheet for more understanding

  R03 =  [[np.sin(q2)*np.cos(q1)*np.cos(q3) + np.sin(q3)*np.cos(q1)*np.cos(q2), np.sin(q1), -np.sin(q2)*np.sin(q3)*np.cos(q1) + np.cos(q1)*np.cos(q2)*np.cos(q3)], 
          [np.sin(q1)*np.sin(q2)*np.cos(q3) + np.sin(q1)*np.sin(q3)*np.cos(q2), -np.cos(q1), -np.sin(q1)*np.sin(q2)*np.sin(q3) + np.sin(q1)*np.cos(q2)*np.cos(q3)], 
          [-np.sin(q2)*np.sin(q3) + np.cos(q2)*np.cos(q3)                     , 0          , -np.sin(q2)*np.cos(q3) - np.sin(q3)*np.cos(q2)]]

  IR03 = np.transpose(R03)

  R36 = np.dot(IR03, R06)

  q4, q5, q6 = calcLast3Angles(R36)

  return q1, q2, q3, q4, q5, q6





def main():

    printMatrices()

    # # # position of end effector
    # # px, py, pz = 2.15299998923500, 0.000215299999641204, 1.94600000000000

    # # # value of orientation of the end effector pich(alpha) roll(beta) yaw(gama) have there usual meaning
    # # roll, pitch, yaw = 0, 0, 0 
    # px, py, pz = 2, 1, 1
    # roll, pitch, yaw = 0, 0, 0

    # q1, q2, q3, q4, q5, q6 = get_angles_puru(px, py, pz, pitch, roll, yaw)
    # q11, q12, q13, q14, q15, q16 = get_angles(px, py, pz, pitch, roll, yaw)

    # print("from puru ", q1, np.pi/2 - q2, -np.pi -q3, q4, -q5, q6)
    # print("from zedd ", q11, q12, q13, q14, q15, q16)

    # print("q1 : ", q1)
    # print("q2 : ", q2)
    # print("q3 : ", q3)
    # print("q4 : ", q4)
    # print("q5 : ", q5)
    # print("q6 : ", q6)

    # # to check the output
    # X, Y, Z = forward_kin(q1, q2, q3, q4, q5, q6)
    # print("X : ", X)
    # print("Y : ", Y)
    # print("Z : ", Z)

    # fig, ax = create_plot()
    # update_plot(X, Y, Z, X, Y, Z, fig, ax)
    # plt.show()


if __name__ == "__main__":
    main()

