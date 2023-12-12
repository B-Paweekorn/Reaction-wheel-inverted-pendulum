import numpy as np
# Reaction Wheel Inverted Pendulum param
L1 = 0.11  # Length of Pendulum from origin to center of mass (m)
L2 = 0.18  # Length of Pendulum (m)
m1 = 0.96  # Mass of Pendulum (kg)
m2 = 0.35  # Mass of Wheel (kg)
I1 = 0.0212  # Innertia moment of Pendulum (Kg*m^2)
I2 = 0.0027  # Innertia moment of Wheel (Kg*m^2)
g = 9.81  # Gravitational acceleration (m*s^2)
dp = 0.01  # RWIP damped
wheelradius = 0.05  # Radius of Reaction wheel (m)

# Brushed DC motor param
J = 0.0027  # Innertia Motor (Kgm^2)
Ng = 0.83  # Transmission ratio of DC Motor
ke = 3.69e-2  # Electrical constant of DC Motor (Vs/rad)
kt = ke * Ng  # Mechanical constant of DC Motor (Nm/A)
R = 2.85  # Resistor of DC Motor (Ohm)
L = 3.73e-4  # Inductance of DC Motor (Henry)
B = 3.85e-3  # Damped of DC Motor (Nn/v)

# Sound 
Sound = True

# Initial state
init_qp = 180.0 # Initial pendulum angle (degree)
init_qp_d = 0.0 # Initial pendulum speed

init_qr = 0.0  # Initial reaction wheel angle
init_qr_d = 0.0  # Initial reaction wheel speed

init_Tm = 0.0  # Initial reaction wheel torque
init_Tp = 0.0  # Initial disturbance torque

# Controller
Q_LQR = np.array([[23, 0, 0, 0], # qp
                  [0, 1, 0, 0],  # qp_d
                  [0, 0, 2, 0],  # qr
                  [0, 0, 0, 1]]) # qr_d
    
R_LQR = 100

N_LQR = np.array([[0],
                  [0],
                  [0],
                  [0]])

LQR_StabilizeBound = 25 # Angle that LQR start to stabilize (degree)