#!/usr/bin/env python
import numpy as np
from functools import reduce
from scipy.linalg import expm
import rospkg

# messages for student to use
from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

def Get_M():
	# Initialize return values of M
	M = np.eye(4)

	##### Your Code Starts Here #####
	# Fill in scripts from lab3 here

	

	##### Your Code Ends Here #####
	return M



HALF_PI = np.pi / 2
DEG_TO_RAD = np.pi / 180
sin = np.sin
cos = np.cos
arcsin = np.arcsin
arccos = np.arccos
arctan = np.arctan
norm = np.linalg.norm
#list of parameters per joint [r, d, alpha]
NUM_JOINTS = 6
PRECISION = 7

PARAMETERS_UR3e = [[0,       .15185, HALF_PI],
              [-.24355, 0,      0],
              [-.2132,  0,      0],
              [0,       .13105, HALF_PI],
              [0,       .08535, -HALF_PI],
              [0,       .0921,  0],
							[-.0535, 	.059, 	0]]

PARAMETERS_UR3 = [[0,       .1519, HALF_PI],
              [-.24365, 0,      0],
              [-.21325,  0,      0],
              [0,       .11235, HALF_PI],
              [0,       .08535, -HALF_PI],
              [0,       .0819,  0],
							[-.0535, 	.059, 	0]]

trans_params = lambda theta, r, d, alpha: np.array(
  [[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),  r * cos(theta)],
   [sin(theta), cos(theta) * cos(alpha),  -cos(theta) * sin(alpha), r * sin(theta)],
   [0,          sin(alpha),               cos(alpha),               d],
   [0,          0,                        0,                        1]])
trans_theta_joint = lambda theta, joint_index: trans_params(
  theta * np.pi / 180, 
  *(PARAMETERS_UR3[joint_index]))

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
	# Initialize the angles 
	angles = [None, None, None, None, None, None]
	print("Forward kinematics calculated:\n")

	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6, 0])
	T = np.eye(4)

	##### Your Code Starts Here #####
	# Fill in scripts from lab3 here
	for i in range(NUM_JOINTS):
		T = np.matmul(T, trans_theta_joint(theta[i], i))

	##### Your Code Ends Here #####
	print(str(T) + "\n")

	angles = theta + [HALF_PI * 2, 0, 0, -HALF_PI, 0, 0]
	# angles[0] = theta1 + np.pi
	# angles[1] = theta2
	# angles[2] = theta3
	# angles[3] = theta4 - (0.5*np.pi)
	# angles[4] = theta5
	# angles[5] = theta6
	return angles

BASE_COORD = np.array([-0.15, 0.15, 0.01])
GRIPPER_PLATE_LENGTH = 0.0535
LL = [0.152, 0.120, 0.244, 0.083, 0.213, 0.083, 0.083, 0.082, 0.0535, 0.052]


def inverse_kinematics(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	angles = np.array([0, 0, 0, 0, 0, 0])

	##### Your Code Starts Here #####
	# TODO: Function that calculates an elbow up 
	# inverse kinematics solution for the UR3

	# Step 1: find gripper position relative to the base of UR3,
	# and set theta_5 equal to -pi/2
	grip = np.array([xWgrip, yWgrip, zWgrip]) - BASE_COORD
	angles[4] = -HALF_PI

	# Step 2: find x_cen, y_cen, z_cen
	yaw = yaw_WgripDegree * DEG_TO_RAD
	wrist_center = grip - np.array([LL[8] * cos(yaw), 
													LL[8] * sin(yaw), 
													0]) 

	# Step 3: find theta_1
	d = norm(wrist_center * [1,1,0])
	x, y = wrist_center[0], wrist_center[1]
	rectL = LL[1] - LL[3] + LL[5]
	angles[0] = HALF_PI - arctan(y / x) - arcsin(rectL / d)

	# Step 4: find theta_6 
		# based on theta 1 and yaw angle, equals 0 when 9 parallel to 4 and 6
	angles[5] = HALF_PI + angles[0] - yaw

	# Step 5: find x3_end, y3_end, z3_end
	x3 = wrist_center - [LL[7]*cos(angles[0]) - rectL * sin(angles[0]),
											LL[7]*cos(angles[0]) - rectL * cos(angles[0]),
																							-LL[9] - LL[7]]

	# Step 6: find theta_2, theta_3, theta_4
	L1, L3, L5 = LL[0], LL[2], LL[4]
	z = x3[2] - L1
	l = norm(x3 - [0,0,L1])
	angles[2] = -arccos((L3*L3+L5*L5-l*l) / (2*L3*L5))
	angles[1] = -(arcsin(L3*sin(angles[2]) / l) + arcsin(z / l))
	angles[3] = -angles[1] - angles[2]

	##### Your Code Ends Here #####

	# print theta values (in degree) calculated from inverse kinematics
	print(reduce(lambda x, y: x + f" {y / DEG_TO_RAD}", angles[1:], str(angles[0] / DEG_TO_RAD)))

	# obtain angles from forward kinematics function
	angles = lab_fk(*angles)

	return angles
