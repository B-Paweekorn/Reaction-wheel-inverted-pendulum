import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN, KEYDOWN
import io
import sys
import math

# RWIP param
L1 = 0.11  # Length of Pendulum from origin to center of mass (m)
L2 = 0.18  # Length of Pendulum (m)
m1 = 0.96  # Mass of Pendulum (kg)
m2 = 0.35  # Mass of Wheel (kg)
I1 = 0.0212  # Innertia moment of Pendulum (Kg*m^2)
I2 = 0.0027  # Innertia moment of Wheel (Kg*m^2)
g = 9.81  # Gravitational acceleration
dp = 1e-5  # RWIP damped
wheelradius = 0.05  # Radius of Reaction wheel

# Motor param
J = 0.0027  # Innertia Motor (Kgm^2)
Ng = 0.83  # Transmission ratio of DC Motor
ke = 3.69e-2  # Electrical constant of DC Motor (Vs/rad)
kt = ke * Ng  # Mechanical constant of DC Motor (Nm/A)
R = 2.85  # Resistor of DC Motor (Ohm)
L = 3.73e-4  # Inductance of DC Motor (Henry)
B = 3.85e-3  # Damped of DC Motor (Nn/v)

def Forwardkinematics(q):
    x = L2 * math.sin(q)
    y = L2 * math.cos(q)
    return x, y

def PendulumEnergy(q, qd):
    K = 0.5 * m1 * (qd * L1) ** 2.0  # Kinetic energy
    P = (m1 + m2) * g * L2 * math.cos(q)  # Potential energy
    return K + P

def MotorDynamics(Vin, dt):
    global qr_d, qr
    Tm = ((Vin - (qr_d * ke)) / R) * kt
    qr_dd = (Tm - B * qr_d) / J
    qr_d = qr_d + (qr_dd * dt)
    qr = qr + (qr_d * dt)
    return Tm

def RwipDynamics(q, Tr, Tp):
    qdd = ((m1 * g * L1 * math.sin(q)) + (m2 * g * L2 * math.sin(q)) - Tr + Tp - dp * qp_d) / ((m1 * L1 ** 2.0) + (m2 * L2) + I1)
    return qdd

def plot_figure(qp, qp_d, qr_d, Tm, Vin, Tp):
    fig, ax = plt.subplots(figsize=(8, 6))

    # Draw RWIP
    x, y = Forwardkinematics(qp)
    ax.plot([0, x], [0, y], 'o-')
    plt.grid()
    
    wheel_circle = plt.Circle((x, y), wheelradius, color='grey', fill=False)
    ax.add_patch(wheel_circle)

    cross_length = wheelradius
    cross_dx = cross_length * np.sin(qr)
    cross_dy = -cross_length * np.cos(qr)
    ax.plot([x - cross_dx, x + cross_dx], [y - cross_dy, y + cross_dy], color='black')
    ax.plot([x - cross_dy, x + cross_dy], [y + cross_dx, y - cross_dx], color='black')

    ax.set_xlim(-2.5 * L1, 2.5 * L1)
    ax.set_ylim(-2.5 * L1, 2.5 * L1)
    ax.set_aspect('equal')

    ax.text(-0.26, 0.25, f'Pendulum Angle (Deg): {round(np.rad2deg(qp), 2)}', fontsize=7, color='black')
    ax.text(-0.26, 0.23, f'Pendulum Speed (Deg/s) : {round(np.rad2deg(qp_d), 2)}', fontsize=7, color='black')
    ax.text(-0.26, 0.21, f'Controller Mode : {controller_mode}', fontsize=7, color='BLUE')

    ax.text(0.11, 0.25, f'Motorspeed (RPM): {round(qr_d * 60 / (math.pi * 2), 2)}', fontsize=7, color='black')
    ax.text(0.11, 0.23, f'Apply Torque (Nm): {round(Tm, 2)}', fontsize=7, color='black')
    ax.text(0.11, 0.21, f'Vin (V): {round(Vin, 2)}', fontsize=7, color='black')
    ax.text(0.11, 0.19, f'Disturbance Torque (Tp): {round(Tp, 2)}', fontsize=7, color='black')

    buffer = io.BytesIO()
    canvas = FigureCanvas(fig)
    canvas.print_png(buffer)
    plt.close(fig)
    buffer.seek(0)
    return pygame.image.load(buffer, 'PNG')

pygame.init()

width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Reaction Wheel Inverted Pendulum')

WHITE = (255, 255, 255)
GREY = (156, 156, 154)
font = pygame.font.Font(None, 36)
clock = pygame.time.Clock()

qp = np.deg2rad(180)  # Initial pendulum angle
qp_d = 0.0  # Initial pendulum speed

qr = 0  # Initial reaction wheel angle
qr_d = 0  # Initial reaction wheel speed

Tm = 0  # Initial reaction wheel torque
Tp = 0  # Initial disturbance torque

dt = 1 / 100  # frequency (Hz)
reqE = (m1 + m2) * g * L2 * math.cos(0)

d_flag = 0

running = True
input_flag = False
input_string = ''

while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == MOUSEBUTTONDOWN:
            if 180 < event.pos[0] < 280 and 10 < event.pos[1] < 60:
                input_flag = True

        elif event.type == KEYDOWN:      
            
            if event.key == pygame.K_BACKSPACE:
                input_string = input_string[:-1]
            else:
                input_string += event.unicode
    
    if input_flag == True:
        try:
            Tp = float(input_string)
        except:
            Tp = 0
            input_string = ''
        input_flag = False
    else:
        Tp = 0
    E = PendulumEnergy(q=qp, qd=qp_d)

    if abs(qp) % (2 * math.pi) <= np.deg2rad(20) or abs(qp) % (2 * math.pi) >= np.deg2rad(340):
        controller_mode = "PID"
        setpoint = 0
        e = setpoint - qp
        Vin = e * -200 + 50 * qp_d

    else:
        controller_mode = "Bang-bang"
        if (qp_d < 0 and E < reqE) or (qp_d >= 0 and E >= reqE):
            Vin = 24
        elif (qp_d >= 0 and E < reqE) or (qp_d < 0 and E >= reqE):
            Vin = -24

    if Vin > 24:
        Vin = 24
    elif Vin < -24:
        Vin = -24

    Tm = MotorDynamics(Vin, dt)

    qp_dd = RwipDynamics(qp, Tm, Tp)
    qp_d = qp_d + (qp_dd * dt)
    qp = qp + (qp_d * dt)

    # Draw Matplotlib figure
    figure_surface = plot_figure(qp, qp_d, qr_d, Tm, Vin, Tp)
    figure_rect = figure_surface.get_rect(center=(width // 2, height // 2))
    screen.blit(figure_surface, figure_rect)

    # Draw button
    pygame.draw.rect(screen, GREY, (180, 10, 100, 50))
    text = font.render('Inject', True, (0, 0, 0))
    screen.blit(text, (195, 22))

    # Draw disturbance input field
    pygame.draw.rect(screen, GREY, (300, 10, 150, 50))
    pygame.draw.rect(screen, WHITE, (315, 20, 120, 30))
    text = font.render(input_string, True, (0, 0, 0))
    screen.blit(text, (320, 25))

    pygame.display.flip()
    clock.tick(100)

pygame.quit()
sys.exit()