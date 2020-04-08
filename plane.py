import numpy as np
import matplotlib.pyplot as plt
import math

# Constants
s=260 # m2, surface area of the wing
rho = 0.5 # kg/m3, density
v=200 # m/s, velocity of aircraft
C_d0=0.02 # no unit
c=-1 # no unit
k=0.05 # no unit
m = 165e3 # kg, initial weight of plane + fuel
mfinal = m/2.0 # initially, half mass is fuel
g = 9.81 # m/s^2, acceleration of gravity

# Maximum Time for simulation
tmax = 24*3600 # seconds
dt = 60 # seconds: time step is one min
niter = int(tmax/dt)

# Allocate memory for arrays
time = np.zeros(niter)
w = np.zeros(niter)
w[0] = m*g

# Loop for W as time progress
ifinal = -1
for i in range(1, niter):
    C_L = w[i-1] / (0.5 * rho * s * v**2)
    dw = c * (0.5) * rho * s * v**2 * (C_d0 + k* (C_L)**2)
    w[i] = w[i-1] + dw * dt
    time[i] = time[i-1] + dt

    if w[i] <= mfinal*g:
        ifinal = i
        break

# Endurance
endurance = time[ifinal]
plt.plot(time[0:ifinal],w[0:ifinal])
plt.show()
