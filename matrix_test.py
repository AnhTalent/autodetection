import numpy
from numpy import cos, sin
from matplotlib import pyplot

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

R_z = R_(-135*D2R, 'z') # implementation of the three 3D rotation matrices
R_y = R_(20*D2R, 'y')

RR = R_z.dot(R_y)
print(RR)


from scipy.spatial.transform import Rotation as RTT

r = RTT.from_matrix(RR)


angles = r.as_euler('xyz', degrees=False) # https://www.gregslabaugh.net/publications/euler.pdf
for a in angles: print(a*R2D)

fig = pyplot.figure()
ax = fig.gca(projection='3d')
ax.plot([0,5],[0,0],[0,0],c='k')
ax.plot([0,0],[0,5],[0,0],c='k')
ax.plot([0,0],[0,0],[0,5],c='k')

ax.plot([0,RR[0,0]],[0,RR[1,0]],[0,RR[2,0]],c='r') # recall columns of R are
ax.plot([0,RR[0,1]],[0,RR[1,1]],[0,RR[2,1]],c='g') # unit coordinate vectors
ax.plot([0,RR[0,2]],[0,RR[1,2]],[0,RR[2,2]],c='b') # of frame b in world frame

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
pyplot.show()