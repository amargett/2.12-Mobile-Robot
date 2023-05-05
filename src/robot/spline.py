import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def cubic_spline(x_i, x_f, theta_i):
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

    cs = CubicSpline([x_i[0], x_f[0]], [x_i[1], x_f[1]], bc_type=((1, theta_i), (1, 0)))

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


x_i = [0, 0]
# x_w = [2, 4]
x_f = [4, 2]
theta_i = 0

f, k = cubic_spline(x_i, x_f, theta_i)

x = np.linspace(0, 4, 100)
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

