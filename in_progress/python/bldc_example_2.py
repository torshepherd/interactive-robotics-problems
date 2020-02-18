import bldc
import numpy as np
from numpy import sin, cos, pi, sqrt
import matplotlib.pyplot as plt

params = {'text.usetex' : True,
          'font.size' : 11,
          }
plt.rcParams.update(params) 

tspan = np.linspace(0, 2*pi, 100)
F = 1 # rev/s
D = 0*tspan
Q = 0*tspan + 1
THETA = (F * tspan)

A, B, C = bldc.dq0_inverse(D, Q, THETA)

ax1 = plt.subplot(1,1,1)
ax1.plot(tspan, A)
ax1.plot(tspan, B)
ax1.plot(tspan, C)
ax1.plot(tspan, Q, '--')
ax1.legend(['$I_A$','$I_B$','$I_C$', '$I_Q$'], loc='lower right')
plt.yticks([])
plt.xticks([])

ann = ax1.annotate('$I_{p,p}$', xy=(3.2, sqrt(2/3)), xycoords='data',
                  xytext=(3.05, -sqrt(2/3)-.02), textcoords='data',
                  arrowprops=dict(arrowstyle="<->",
                                  connectionstyle="bar",
                                  ec="k",
                                  shrinkA=5, shrinkB=5))

plt.show()
