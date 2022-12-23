import numpy
from numpy import cos, sin
from matplotlib import pyplot
from scipy.spatial.transform import Rotation as RTT
import math as m


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
    
def matrix2euler(R):
    import sys
    tol = sys.float_info.epsilon * 10
    
    if abs(R.item(0,0))< tol and abs(R.item(1,0)) < tol:
       eul1 = 0
       eul2 = m.atan2(-R.item(2,0), R.item(0,0))
       eul3 = m.atan2(-R.item(1,2), R.item(1,1))
    else:   
       eul1 = m.atan2(R.item(1,0),R.item(0,0))
       sp = m.sin(eul1)
       cp = m.cos(eul1)
       eul2 = m.atan2(-R.item(2,0),cp*R.item(0,0)+sp*R.item(1,0))
       eul3 = m.atan2(sp*R.item(0,2)-cp*R.item(1,2),cp*R.item(1,1)-sp*R.item(0,1))
   
    return numpy.array([psi, theta, phi])
    

D2R = numpy.pi/180
R2D = 180/numpy.pi


# given parameters
alpha = 35 * D2R
pose_evcs = numpy.array([55 * D2R, 85 * D2R, 5 * D2R])
print('EVCS angle is : ', alpha * R2D)
print('EVCS pose is : ', pose_evcs * R2D)

# rotation matrix from evcs to enu
Rz_evcs2enu = R_(alpha + pose_evcs[0], 'z')
Ry_evcs2enu = R_(pose_evcs[1], 'y')
Rx_evcs2enu = R_(pose_evcs[2], 'x')

R_evcs2enu = (Rz_evcs2enu.dot(Ry_evcs2enu)).dot(Rx_evcs2enu)
pose_enu = RTT.from_matrix(R_evcs2enu).as_euler('xyz')[::-1]

print('ENU pose is : ', pose_enu * R2D)

# rotation matrix from enu to wgs84 for axis
Rz_enu2wgs = R_(-numpy.pi/2, 'z')
Ry_enu2wgs = R_(numpy.pi, 'y')


R_enu2wgs = Rz_enu2wgs.dot(Ry_enu2wgs)

# rotation matrix from evcs to wgs84
R_evcs2wgs = R_enu2wgs.dot(R_evcs2enu)

pose_wgs = RTT.from_matrix(R_evcs2wgs).as_euler('xyz')[::-1]

print('WGS84 pose is : ', pose_wgs * R2D)


########## REVERSE ###########

# rotation matix from wgs84 to enu
Rz_wgs2enu = R_(numpy.pi/2 - pose_wgs[0], 'z')
Ry_wgs2enu = R_(-pose_wgs[1], 'y')
Rx_wgs2enu = R_(numpy.pi+pose_wgs[2], 'x')

R_wgs2enu = (Rz_wgs2enu.dot(Ry_wgs2enu)).dot(Rx_wgs2enu)

pose_enu_rev = RTT.from_matrix(R_wgs2enu).as_euler('xyz')[::-1]

print('Reversed ENU pose is : ', pose_enu * R2D)

# rotation matrix from enu to evcs
Rz_enu2evcs = R_(-alpha, 'z')

R_enu2evcs = Rz_enu2evcs

# rotation matrix from wgs to evcs
R_wgs2evcs = R_enu2evcs.dot(R_wgs2enu)

pose_evcs_rev = RTT.from_matrix(R_wgs2evcs).as_euler('xyz')[::-1]

print('Reversed EVCS pose is : ', pose_evcs_rev * R2D)



# verification for reverse using rotation matrices
rr1 = RTT.from_euler('zyx', pose_evcs).as_matrix()
rr2 = RTT.from_euler('zyx', pose_evcs_rev).as_matrix()

print(numpy.abs(numpy.sum(rr1-rr2)) < 0.000000001)

#print(numpy.sum(pose_evcs - pose_evcs_rev))
