import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from pylab import *
from mpc_solver import mpc_solver

def get_ReferenceTrajectory(K,a,h,r,step_n):

    x = []
    y = []
    z = []
    c = 0

    verical = np.linspace(20, 0 , step_n);
    for t in verical:
        z.append(t)
        x.append((h - t)/h * r * cos(a * c * 0.2))
        y.append((h - t)/h * r * sin(a * c * 0.2))
        c = c+1

    for i in range(K):
        z.append(z[-1])
        x.append(x[-1])
        y.append(y[-1])

    return x, y, z




