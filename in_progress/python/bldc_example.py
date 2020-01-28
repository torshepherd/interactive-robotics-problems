import bldc
import numpy as np
from numpy import sin, cos, pi
import matplotlib.pyplot as plt

tspan = np.linspace(0, 2*pi, 100)
F = 2 # rev/s
A = sin(F * (tspan) - 0*pi/3)+1
B = sin(F * (tspan) - 2*pi/3)+1
C = sin(F * (tspan) - 4*pi/3)+1
# THETA = np.ones(np.shape(tspan))
THETA = (F * tspan) - 1

X, Y, Z = bldc.clarke(A, B, C)
D, Q = bldc.dq0(A, B, C, THETA)

ax1 = plt.subplot(1,3,1)
ax2 = plt.subplot(1,3,2)
ax3 = plt.subplot(1,3,3)

ax1.plot(tspan, A)
ax1.plot(tspan, B)
ax1.plot(tspan, C)
ax1.plot(tspan, A + B + C)
ax2.plot(tspan, X)
ax2.plot(tspan, Y)
ax2.plot(tspan, Z)
ax3.plot(tspan, D)
ax3.plot(tspan, Q)
ax1.legend(['A','B','C','SUM'])
ax2.legend(['X','Y','Z'])
ax3.legend(['D','Q'])
plt.show()
