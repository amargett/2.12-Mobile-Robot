import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def cubic_spline(x_i, y_i, heading_i, x_f, y_f, heading_f):
    '''
    Generates a cubic spline curve based on 2 points.

    Args:
        x_i (array): coordinates of the initial point
        x_f (array): coordinates of the final point
        theta_i (float): orientation at initial point
    
    Returns: []
        f (function): spline curve as a function of x
        k (function): curvature as a function of x
    '''
    path_x = [x_i, x_f]
    path_y = [y_i, y_f]
    cs = CubicSpline(path_x, path_y, bc_type=((1, heading_i), (1, heading_f)))
    t = np.linspace(0,1,len(path_x))
    print(t)

    a, b, c, d = cs.c

    def f(x):
        return a*x*x*x + b*x*x + c*x + d

    def df(x):
        return 3*a*x*x + 2*b*x + c

    def ddf(x):
        return 6*a*x + 2*b

    def k(x):
        radius = np.power(1 + np.power(df(x), 2), 3/2)/ddf(x)
        return 1/radius

    return f, k


x_i = 0
y_i = 0
x_f = 4
y_f = 2
heading_i = 0
heading_f = 1

f, k = cubic_spline(x_i, y_i, heading_i, x_f, y_f, heading_f)

x = np.linspace(x_i, x_f, 100)
y = f(x)
k = k(x)
plt.plot(x, y, label='path')
plt.plot(x, k, label='curvature')
plt.legend()
plt.show()


# V = 0.5 # constant linear speed V
# r = 0.05 # diameter of the wheel
# d = 0.2 # 1/2 the width of the robot

# def omegaL(x):
#     return (V/(2*np.pi*r))*(1 - d/R(x))

# def omegaR(x):
#     return (V/(2*np.pi*r))*(1 + d/R(x))
