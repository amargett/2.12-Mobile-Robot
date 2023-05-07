import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

x_i = 0
y_i = 0
x_f = 4
y_f = 2
heading_i = 0
heading_f = 1

path_x = np.asarray((x_i, x_f),dtype=float)
path_y = np.asarray((y_i, y_f),dtype=float)

# defining arbitrary parameter to parameterize the curve
path_t = np.linspace(0,1,path_x.size)

# this is the position vector with
# x coord (1st row) given by path_x, and
# y coord (2nd row) given by path_y
r = np.vstack((path_x.reshape((1,path_x.size)),path_y.reshape((1,path_y.size))))


cs = CubicSpline(path_t, r, bc_type=((1, heading_i), (1, heading_f)))
# creating the spline object

# defining values of the arbitrary parameter over which
# you want to interpolate x and y
# it MUST be within 0 and 1, since you defined
# the spline between path_t=0 and path_t=1
t = np.linspace(np.min(path_t),np.max(path_t),100)


# interpolating along t
# r[0,:] -> interpolated x coordinates
# r[1,:] -> interpolated y coordinates
r = cs(t)

plt.plot(path_x,path_y,'or')
plt.plot(r[0,:],r[1,:],'-k')
plt.xlabel('x')
plt.ylabel('y')
plt.show()