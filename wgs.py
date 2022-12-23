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

alpha = 10*D2R
pose_evcs = [190*D2R, -160*D2R, 0*D2R]

Rz_ee = R_(alpha + pose_evcs[0], 'z')
Ry_ee = R_(pose_evcs[1], 'y')
Rx_ee = R_(pose_evcs[2], 'x')

R_ee = (Rz_ee.dot(Ry_ee)).dot(Rx_ee)
pose_enu = RTT.from_matrix(R_ee).as_euler('xyz')[::-1]

print(pose_enu * R2D)

r1 = RTT.from_euler('zyx', [-75, 100, 15], degrees=True)
r2 = RTT.from_euler('zyx', [105, 80, -165], degrees=True)
r3 = RTT.from_euler('zyx', pose_enu, degrees=False)
print(r3.as_matrix())
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

########## REVERSE ###########

Rz_ew = R_(numpy.pi/2 - pose_wgs[0], 'z')
Ry_ew = R_(-pose_wgs[1], 'y')
Rx_ew = R_(numpy.pi+pose_wgs[2], 'x')

R_ew = (Rz_ew.dot(Ry_ew)).dot(Rx_ew)
pose_enu = RTT.from_matrix(R_ew).as_euler('xyz')[::-1]

print(pose_enu * R2D)

Rz_ree = R_(-alpha, 'z')

R_ree = Rz_ree

R_rew = R_ree.dot(R_ew)
pose_evcs_rev = RTT.from_matrix(R_rew).as_euler('xyz')[::-1]

print(pose_evcs_rev * R2D)

print(numpy.array(pose_evcs) * R2D)

rr1 = RTT.from_euler('zyx', pose_evcs).as_matrix()
rr2 = RTT.from_euler('zyx', pose_evcs_rev).as_matrix()

print(rr1 - rr2)

print(numpy.abs(numpy.sum(rr1-rr2)) < 0.000000001)

#print(numpy.sum(pose_evcs - pose_evcs_rev))
