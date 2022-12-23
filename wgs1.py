import numpy
from numpy import cos, sin
from matplotlib import pyplot
from scipy.spatial.transform import Rotation as RTT


def R_(theta, axis):
    if axis == 'x':
        R = [[1, 0, 0], 
        [0, cos(theta), -sin(theta)],
        [0, sin(theta), cos(theta)]]
    elif axis == 'y':
        R = [[cos(theta), 0, sin(theta)],
        [0, 1, 0],
        [-sin(theta), 0, cos(theta)]]
    elif axis == 'z':
        R = [[cos(theta), -sin(theta), 0],
        [sin(theta), cos(theta), 0],
        [0, 0, 1]]
        
    return numpy.array(R)
    

D2R = numpy.pi/180
R2D = 180/numpy.pi

alpha = -30*D2R
pose_evcs = [-45*D2R, 100*D2R, 15*D2R]

M_evcs = RTT.from_euler('zyx', pose_evcs, degrees=False)


Rz_ee = R_(alpha, 'z')
#Ry_ee = R_(pose_evcs[1], 'y')
#Rx_ee = R_(pose_evcs[2], 'x')

#R_ee = (Rz_ee.dot(Ry_ee)).dot(Rx_ee)
R_ee = Rz_ee


pose_enu1 = RTT.from_matrix(R_ee.dot(M_evcs.as_matrix())).as_euler('xyz')[::-1]

pose_enu = RTT.from_matrix(R_ee).as_euler('xyz')[::-1]

print(R_ee.dot(M_evcs.as_matrix()))
print(pose_enu * R2D)
print(pose_enu1 * R2D)

r1 = RTT.from_euler('zyx', [-75, 100, 15], degrees=True)
r2 = RTT.from_euler('zyx', [105, 80, -165], degrees=True)
#print(r1.as_matrix())
#print(r2.as_matrix()-r1.as_matrix())

print('\n\n')

Rz_we = R_(-numpy.pi/2, 'z')
Ry_we = R_(numpy.pi, 'y')
Rx_we = R_(pose_enu[2], 'x')

R_we = (Rz_we.dot(Ry_we))#.dot(Rx_we)
R_we = R_we.dot(R_ee)
pose_wgs = RTT.from_matrix(R_we).as_euler('xyz')[::-1]

print(pose_wgs * R2D)



#pose_enu = [15*D2R, 35*D2R, 25*D2R]

Rz_we = R_(numpy.pi/2 - pose_enu[0], 'z')
Ry_we = R_(-pose_enu[1], 'y')
Rx_we = R_(numpy.pi+pose_enu[2], 'x')

R_we = (Rz_we.dot(Ry_we)).dot(Rx_we)
pose_wgs = RTT.from_matrix(R_we).as_euler('xyz')[::-1]

print(pose_wgs * R2D)