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

def MatrixGenerator():
    a = m1*(L1**2) + m2*(L2**2) + I1
    b = (m1*L1 + m2*L2) * g

    a21 = b/a
    a24 = kt*ke*(Ng**2)/(a * R)
    a41 = -b/a
    a44 = -1 * ((a + I2)/(a * I2)) * ((kt*ke*Ng**2)/R)

    b2 = -(kt*Ng)/(a*R)
    b4 = ((a + I2)/(a*I2)) * ((kt*Ng)/R)

    A = np.array([[0, 1, 0, 0],
                  [a21, 0, 0, a24],
                  [0, 0, 0, 1],
                  [a41, 0, 0, a44]])

    B = np.array([[0],
                  [b2],
                  [0],
                  [b4]])
    
    C = np.array([[1, 0, 0, 0]])

    D = np.array([[0, 0, 0, 0]])

    return A, B, C, D

def Forwardkinematics(theta):
    x_pos = L2 * math.sin(theta)
    y_pos = L2 * math.cos(theta)
    return x_pos, y_pos

# Loop configuration
dt = 0.01
timestamp = 0
fig, ax = plt.subplots()
ax.grid(True)

# State space
State_x = np.array([[0],
                    [0],
                    [0],
                    [0]])
Vin = 300

A, B, C, D = MatrixGenerator()
Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D),dt)
while True:
    if time.time() - timestamp >= dt:
        timestamp = time.time()

        # State Space
        State_x_dot = np.dot(Ad, State_x) + np.dot(Bd, Vin)
        State_theta = np.dot(Cd, State_x)[0,0]
        State_x = State_x + np.dot(State_x_dot, dt)
        
        print(State_theta)
        Pos_x, Pos_y = Forwardkinematics(State_theta)

        ax.clear()
        ax.plot([0, Pos_x], 
                [0, Pos_y], 'o-')  # Plot robot

        ax.set_xlim(-2*L1, 2*L1)
        ax.set_ylim(-2*L1, 2*L1)
        ax.set_aspect('equal')
        plt.pause(dt)
        



