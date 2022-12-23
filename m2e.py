import mathutils as mu
import numpy as np
import math


R = mu.Matrix([[-0.66446302,  0.70710678, -0.24184476],
 [-0.66446302, -0.70710678, -0.24184476],
 [-0.34202014,  0., 0.93969262]])
 
 # xyz

phi_y = np.arctan2(R[0][2], math.sqrt(1 - (R[0][2])**2))  #angle beta in wiki
# or just phi_y = np.arcsin(R[0,2])
phi_x = np.arctan2(-R[1][2],R[2][2])    #angle alpha in wiki
phi_z = np.arctan2(-R[0][1],R[0][0])    #angle gamma in wiki


print('Manually computed XYZ Euler angles =')
print([phi_x, phi_y, phi_z])

print('Euler angles using to_euler function in Blender=')
#print(R.to_euler('XYZ'))
#print(R.to_euler('ZYX'))


def rotation_angles(matrix, order):
    """
    input
        matrix = 3x3 rotation matrix (numpy array)
        oreder(str) = rotation order of x, y, z : e.g, rotation XZY -- 'xzy'
    output
        theta1, theta2, theta3 = rotation angles in rotation order
    """
    r11, r12, r13 = matrix[0]
    r21, r22, r23 = matrix[1]
    r31, r32, r33 = matrix[2]

    if order == 'xzx':
        theta1 = np.arctan(r31 / r21)
        theta2 = np.arctan(r21 / (r11 * np.cos(theta1)))
        theta3 = np.arctan(-r13 / r12)

    elif order == 'xyx':
        theta1 = np.arctan(-r21 / r31)
        theta2 = np.arctan(-r31 / (r11 *np.cos(theta1)))
        theta3 = np.arctan(r12 / r13)

    elif order == 'yxy':
        theta1 = np.arctan(r12 / r32)
        theta2 = np.arctan(r32 / (r22 *np.cos(theta1)))
        theta3 = np.arctan(-r21 / r23)

    elif order == 'yzy':
        theta1 = np.arctan(-r32 / r12)
        theta2 = np.arctan(-r12 / (r22 *np.cos(theta1)))
        theta3 = np.arctan(r23 / r21)

    elif order == 'zyz':
        theta1 = np.arctan(r23 / r13)
        theta2 = np.arctan(r13 / (r33 *np.cos(theta1)))
        theta3 = np.arctan(-r32 / r31)

    elif order == 'zxz':
        theta1 = np.arctan(-r13 / r23)
        theta2 = np.arctan(-r23 / (r33 *np.cos(theta1)))
        theta3 = np.arctan(r31 / r32)

    elif order == 'xzy':
        theta1 = np.arctan(r32 / r22)
        theta2 = np.arctan(-r12 * np.cos(theta1) / r22)
        theta3 = np.arctan(r13 / r11)

    elif order == 'xyz':
        theta1 = np.arctan(-r23 / r33)
        theta2 = np.arctan(r13 * np.cos(theta1) / r33)
        theta3 = np.arctan(-r12 / r11)

    elif order == 'yxz':
        theta1 = np.arctan(r13 / r33)
        theta2 = np.arctan(-r23 * np.cos(theta1) / r33)
        theta3 = np.arctan(r21 / r22)

    elif order == 'yzx':
        theta1 = np.arctan(-r31 / r11)
        theta2 = np.arctan(r21 * np.cos(theta1) / r11)
        theta3 = np.arctan(-r23 / r22)

    elif order == 'zyx':
        theta1 = np.arctan(r21 / r11)
        theta2 = np.arctan(-r31 * np.cos(theta1) / r11)
        theta3 = np.arctan(r32 / r33)

    elif order == 'zxy':
        theta1 = np.arctan(-r12 / r22)
        theta2 = np.arctan(r32 * np.cos(theta1) / r22)
        theta3 = np.arctan(-r31 / r33)

    theta1 = theta1 * 180 / np.pi
    theta2 = theta2 * 180 / np.pi
    theta3 = theta3 * 180 / np.pi

    return (theta1, theta2, theta3)
    
    
print(rotation_angles(R, 'zyx'))

from scipy.spatial.transform import Rotation as RR

r = RR.from_matrix([[-6.64463024e-01,  7.07106781e-01, -2.41844763e-01],
 [-6.64463024e-01, -7.07106781e-01, -2.41844763e-01],
 [-3.42020143e-01,  1.38777878e-17,  9.39692621e-01]])
 
print(r.as_euler('xyz', degrees=True))

r1 = RR.from_euler('zyx', [-135, 20, 0], degrees=True)
print(r1.as_matrix())
r2 = RR.from_euler('zyx', [-133.2191789,   -13.99544536,   14.43275505], degrees=True)

print(r2.as_matrix())

print(r1.as_matrix() - r2.as_matrix())