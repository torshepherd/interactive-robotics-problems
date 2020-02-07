import numpy as np
from numpy import pi, sin, cos
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

t = np.linspace(0, 1, 15)

x = sin(t*2*pi)
y = cos(t*2*pi)
y *= (y > 0)

x_1 = interp1d(t, x, kind='cubic')
y_1 = interp1d(t, y, kind='cubic')

x_2 = x_1(np.linspace(0,1,100))
y_2 = y_1(np.linspace(0,1,100))

# plt.plot(x, y)
plt.plot(x_2, y_2)
plt.show()

with open('leg_0.traj', 'w') as traj_file:
    for pt in zip(list(x_2.astype(str)), list(y_2.astype(str))):
        traj_file.write(','.join(pt)+'\n')
