"""
Reaction Wheel Inverted Pendulum Simulation

This script defines parameters and dynamics for simulating a Reaction Wheel Inverted Pendulum (RWIP).
The simulation includes the pendulum, reaction wheel, and a brushed DC motor model.

Author: [Paweekorn Buasakorn]
        [Nopparuj Ananvoranich]
Date: [11 Dec 2023]

"""

import numpy as np

# Reaction Wheel Inverted Pendulum parameters
L1 = 0.11  # Length of Pendulum from origin to center of mass (m)
L2 = 0.18  # Length of Pendulum (m)
m1 = 0.96  # Mass of Pendulum (kg)
m2 = 0.35  # Mass of Wheel (kg)
I1 = 0.0212  # Inertia moment of Pendulum (Kg*m^2)
I2 = 0.0027  # Inertia moment of Wheel (Kg*m^2)
g = 9.81  # Gravitational acceleration (m*s^2)
dp = 0.01  # RWIP damping
wheelradius = 0.05  # Radius of Reaction wheel (m)

# Brushed DC motor parameters
J = 0.0027  # Inertia Motor (Kgm^2)
Ng = 0.83  # Transmission ratio of DC Motor
ke = 3.69e-2  # Electrical constant of DC Motor (Vs/rad)
kt = ke * Ng  # Mechanical constant of DC Motor (Nm/A)
R = 2.85  # Resistor of DC Motor (Ohm)
L = 3.73e-4  # Inductance of DC Motor (Henry)
B = 3.85e-3  # Damping of DC Motor (Nn/v)

# Sound 
Sound = True  # Flag to enable/disable sound

# Initial state
init_qp = 180.0  # Initial pendulum angle (degree)
init_qp_d = 0.0  # Initial pendulum speed

init_qr = 0.0  # Initial reaction wheel angle
init_qr_d = 0.0  # Initial reaction wheel speed

init_Tm = 0.0  # Initial reaction wheel torque
init_Tp = 0.0  # Initial disturbance torque

StabilizeBound = 25  # Angle at which LQR starts to stabilize (degree)
Stabilize_Controller = "LQR" # Stabilize mode "LQR" or "PID"

# Controller parameters for PID
plot_rootlocus = False # Plot root locus of system
Kp = 2730

# Controller parameters for LQR
Q_LQR = np.array([[342, 0, 0, 0],  # Weight for qp
                  [0, 541, 0, 0],  # Weight for qp_d
                  [0, 0, 1, 0],  # Weight for qr
                  [0, 0, 0, 1]])  # Weight for qr_d
    
R_LQR = 100  # Weight for the control input

N_LQR = np.array([[0],  # No cross-coupling terms for LQR
                  [0],
                  [0],
                  [0]])

StabilizeBound = 25  # Angle at which LQR starts to stabilize (degree)
