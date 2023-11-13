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
J = 1.7e-6 # Innertia Motor (Kgm^2)
Ng = 0.83 # Transmission ratio of DC Motor
ke = 3.69e-2 # Electrical constant of DC Motor (Vs/rad)
kt = ke * Ng # Mechanical constant of DC Motor (Nm/A)
R = 2.85 # Resistor of DC Motor (Ohm)
L = 3.73e-4 # Inductance of DC Motor (Henry)
B = 3.85e-6 # Damped of DC Motor (Nn/v)

def qp_dd_cal(q, Tr):
    qdd = ((m1*g*L1*math.sin(q)) + (m2*g*L2*math.sin(q)) - Tr)/((m1*L1**2.0) + (m2*L2) + I1)
    return qdd

def Forwardkinematics(q):
    x = L2 * math.sin(q)
    y = L2 * math.cos(q)
    return x, y

def PendulumEnergy(q,qd):
    K = 0.5*m1*(qd*L1)**2.0 # Kinetic energy
    P = (m1 + m2)*g*L2*math.cos(q) # Potential energy
    return K + P

qr_d = 0
def MotorDynamics(Vin, dt):
    global qr_d
    Tm = ((Vin - (qr_d*ke))/R) * kt
    qr_dd = (Tm - B*qr_d)/J
    qr_d = qr_d + (qr_dd * dt)
    return Tm

qp = math.pi # initial pendulum angle
qp_d = 0.0 # initial pendulum spe

Tr = 1

timestamp = 0
dt = 0.0001
fig, ax = plt.subplots()

ax.grid(True)

timep = 0

while True:
    if time.time() - timestamp >= dt:
        timestamp = time.time()
        # T = PendulumEnergy(qp, qp_d)
        # if T < (m1 + m2)*g*L2*math.cos(0) and qp >= math.pi and qp_d < 0.01:
        #     Tr = 1
        # else:
        #     Tr = 0

        Tm = MotorDynamics(24,dt)
        timep += dt
        print(Tm,qr_d)
        # plt.plot(timep, qr_dp, label='qr_d')

        # qp_dd = qp_dd_cal(qp,Tr)
        # qp_d = qp_d + (qp_dd * dt)
        # qp = qp + (qp_d * dt)

        # x, y = Forwardkinematics(qp)

        # print(qp)
        # ax.clear()
        # ax.plot([0, x], 
        #         [0, y], 'o-')  # Plot robot

        # ax.set_xlim(-2*L1, 2*L1)
        # ax.set_ylim(-2*L1, 2*L1)
        # ax.set_aspect('equal')
        plt.pause(dt)