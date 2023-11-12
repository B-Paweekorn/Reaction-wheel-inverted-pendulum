import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete 
import math
import time

# RWIP param
L1 = 0.11 # Length of Pendulum from origin to center of mass (m)
L2 = 0.18 # Length of Pendulum (m)
m1 = 0.96 # Mass of Pendulum (kg)
m2 = 0.35 # Mass of Wheel (kg)
I1 = 0.0212 # Innertia moment of Pendulum (Kg*m^2)
I2 = 0.0027 # Innertia moment of Wheel (Kg*m^2)
g = 9.81 # Gravitational acceleration

# Motor param
kt = 0.0649 # Mechanical constant of DC Motor (Nm/A)
ke = 0.0649 # Electrical constant of DC Motor (Vs/rad)
Ng = 1 # Transmission ratio of DC Motor
R = 6.83 # Resistor of DC Motor (Ohm)

a = m1*(L1**2) + m2*(L2**2) + I1
b = (m1*L1 + m2*L2) * g

def qp_dd_cal(q, Tr):
    qdd = ((m1*g*L1*math.sin(q)) + (m2*g*L2*math.sin(q)) - Tr)/((m1*L1**2) + (m2*L2) + I1)
    return qdd

def Forwardkinematics(q):
    x = L2 * math.sin(q)
    y = L2 * math.cos(q)
    return x, y

def PendulumEnergy(q,qd):
    K = 0.5*m1*(qd*L1)**2 # Kinetic energy
    P = (m1 + m2)*g*L2*math.cos(q) # Potential energy
    return K + P

qp = math.pi # initial pendulum angle
qp_d = 0.0 # initial pendulum spe

Tr = 1.0

timestamp = 0
dt = 0.01
fig, ax = plt.subplots()

ax.grid(True)

while True:
    if time.time() - timestamp >= dt:
        timestamp = time.time()
        T = PendulumEnergy(qp, qp_d)
        if T < (m1 + m2)*g*L2*math.cos(0) and qp >= math.pi and qp_d < 0.01:
            Tr = 1
        else:
            Tr = 0

        qp_dd = qp_dd_cal(qp,Tr)
        qp_d = qp_d + (qp_dd * dt)
        qp = qp + (qp_d * dt)

        x, y = Forwardkinematics(qp)

        print(qp)
        ax.clear()
        ax.plot([0, x], 
                [0, y], 'o-')  # Plot robot

        ax.set_xlim(-2*L1, 2*L1)
        ax.set_ylim(-2*L1, 2*L1)
        ax.set_aspect('equal')
        plt.pause(dt)