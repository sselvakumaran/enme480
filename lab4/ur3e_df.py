import numpy as np
from functools import reduce
HALF_PI = np.pi / 2
sin = np.sin
cos = np.cos
#list of parameters per joint [r, d, alpha]
NUM_JOINTS = 6
PRECISION = 7
#r, d, alpha
PARAMETERS = [[0,       .15185, HALF_PI],
              [-.24355, 0,      0],
              [-.2132,  0,      0],
              [0,       .13105, HALF_PI],
              [0,       .08535, -HALF_PI],
              [0,       .0921,  0]]

trans_params = lambda theta, r, d, alpha: np.array(
  [[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),  r * cos(theta)],
   [sin(theta), cos(theta) * cos(alpha),  -cos(theta) * sin(alpha), r * sin(theta)],
   [0,          sin(alpha),               cos(alpha),               d],
   [0,          0,                        0,                        1]])

trans_theta_joint = lambda theta, joint_index: trans_params(
  theta * np.pi / 180, 
  *(PARAMETERS[joint_index]))

def calc_total_transformation(theta_vector):
  joint_transformations = []
  m = trans_theta_joint(theta_vector[0], 0)
  for i in range(1, NUM_JOINTS):
    m = np.matmul(m, trans_theta_joint(theta_vector[i], i))
  res = np.round(m, PRECISION)
  return res[0:3,0:3], res[0:3,3], res

test_vectors = [[-20, -70, 110, -15, 0, 20],
                [-40, -60, 80, -10, -90, -30],
                [30, -70, 80, -10, -90, 10],
                [0, -30, 60, -60, -60, -60]]

for v in test_vectors:
  res = calc_total_transformation(v)
  print(res[2])
  # print(f"Position: {str(res[1])}")
  # print(f"Orientation:\n{str(res[0])}")