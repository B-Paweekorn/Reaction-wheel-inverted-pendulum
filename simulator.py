import numpy as np
import matplotlib.pyplot as plt
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
wheelradius = 0.05 # Radius of Reaction wheel

# Motor param
J = 0.0027 # Innertia Motor (Kgm^2)
Ng = 0.83 # Transmission ratio of DC Motor
ke = 3.69e-2 # Electrical constant of DC Motor (Vs/rad)
kt = ke * Ng # Mechanical constant of DC Motor (Nm/A)
R = 2.85 # Resistor of DC Motor (Ohm)
L = 3.73e-4 # Inductance of DC Motor (Henry)
B = 3.85e-3 # Damped of DC Motor (Nn/v)

def Forwardkinematics(q):
    x = L2 * math.sin(q)
    y = L2 * math.cos(q)
    return x, y

def PendulumEnergy(q,qd):
    K = 0.5*m1*(qd*L1)**2.0 # Kinetic energy
    P = (m1 + m2)*g*L2*math.cos(q) # Potential energy
    return K + P

def MotorDynamics(Vin, dt):
    global qr_d,qr
    Tm = ((Vin - (qr_d*ke))/R) * kt
    qr_dd = (Tm - B*qr_d)/J
    qr_d = qr_d + (qr_dd * dt)
    qr = qr + (qr_d * dt)
    return Tm

def RwipDynamics(q, Tr):
    qdd = ((m1*g*L1*math.sin(q)) + (m2*g*L2*math.sin(q)) - Tr)/((m1*L1**2.0) + (m2*L2) + I1)
    return qdd

qp = np.deg2rad(180) # Initial pendulum angle
qp_d = 0.0 # Initial pendulum speed

qr = 0 # Initial reaction wheel angle
qr_d = 0 # Initial reaction wheel speed

Tm = 0 # Initial reaction wheel torque

timestamp = 0
timestart = time.time()
fig, ax = plt.subplots()

ax.grid(True)

dt = 1/100 # frequency (Hz)

reqE = (m1 + m2)*g*L2*math.cos(0)
while True:
    if time.time() - timestamp >= dt:
        timestamp = time.time()
        
        E = PendulumEnergy(q = qp,qd= qp_d)

        if abs(qp) % (2*math.pi) <= np.deg2rad(20) or abs(qp) % (2*math.pi) >= np.deg2rad(340):
            controller_mode = "LQR"
            setpoint = 0
            e = setpoint - qp
            print(e,Tm)
            Vin = e * -700 + 50 * qp_d

        else:
            controller_mode = "Bang-bang"
            if((qp_d < 0 and E < reqE) or (qp_d >=0 and E >= reqE)):
                Vin = 12
            elif((qp_d>=0 and E < reqE) or (qp_d < 0 and E >= reqE)):
                Vin = -12

        Tm = MotorDynamics(Vin,dt)

        qp_dd = RwipDynamics(qp,Tm)
        qp_d = qp_d + (qp_dd * dt)
        qp = qp + (qp_d * dt)

        # Draw RWIP
        x, y = Forwardkinematics(qp)

        ax.clear()
        plt.grid()
        ax.plot([0, x], 
                [0, y], 'o-')
        
        wheel_circle = plt.Circle((x, y), wheelradius, color='grey', fill=False)  # Plot reaction wheel
        ax.add_patch(wheel_circle)
        cross_length = wheelradius
        cross_dx = cross_length * np.sin(qr)
        cross_dy = -cross_length * np.cos(qr)
        plt.plot([x - cross_dx, x + cross_dx], [y - cross_dy, y + cross_dy], color='black')
        plt.plot([x - cross_dy, x + cross_dy], [y + cross_dx, y - cross_dx], color='black')

        plt.text(-0.26, 0.25, f'Pendulum Angle (deg): {round(np.rad2deg(qp), 2)}', fontsize=7, color='black')
        plt.text(-0.26, 0.23, f'Pendulum Speed (deg/s) : {round(np.rad2deg(qp_d), 2)}', fontsize=7, color='black')
        plt.text(-0.26, 0.21, f'Controller: {controller_mode}', fontsize=7, color='blue')

        plt.text(0.05, 0.23, f'Apply Torque (Nm): {round(Tm, 2)}', fontsize=7, color='black')
        plt.text(0.05, 0.21, f'Vin (V): {round(Vin, 2)}', fontsize=7, color='black')
        plt.text(0.05, 0.25, f'Motorspeed (rad/s): {round(qr_d * 60/(math.pi * 2), 2)}', fontsize=7, color='black')
        

        ax.set_xlim(-2.5*L1, 2.5*L1)
        ax.set_ylim(-2.5*L1, 2.5*L1)
        ax.set_aspect('equal')
        plt.pause(dt)